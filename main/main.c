#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "app_config.h"
#include "rescue_types.h"

#include "imu_driver.h"
#include "mag_driver.h"
#include "gnss_driver.h"
#include "lora_driver.h"
#include "display_driver.h"
#include "activity_classifier.h"
#include "pdr.h"
#include "state_machine.h"
#include "battery_monitor.h"
#include "power_mgmt.h"
#include "baro_driver.h"

static const char *TAG = "main";

/* ===================== Khởi tạo bus dùng chung ===================== */

static void init_i2c_bus(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_BUS_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_BUS_PORT, cfg.mode, 0, 0, 0));
}

static void init_buttons(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_SOS_GPIO) | (1ULL << BTN_CANCEL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&cfg);
}

/* ===================== Quét bus I2C ===================== */

static void i2c_scan_bus(i2c_port_t port, const char *name)
{
    ESP_LOGI(TAG, "Scanning %s...", name);
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  %s: Found device at 0x%02X", name, addr);
        }
    }
    ESP_LOGI(TAG, "%s scan done.", name);
}

static void i2c_scan(void)
{
    i2c_scan_bus(I2C_BUS_PORT, "I2C0");
    /* I2C1 (OLED) driver chưa được install ở giai đoạn này → skip scan */
}

/* ===================== Shared sensor data (cho OLED) ===================== */
static imu_sample_t s_latest_imu;
static float s_latest_heading = 0.0f;
static float s_latest_pressure = 0.0f;
static float s_latest_temp = 0.0f;
static float s_latest_alt = 0.0f;
static bool s_mag_ok = false;
static bool s_gnss_fix = false;
static bool s_display_ok = false;

/* ===================== Task: đọc IMU + phân loại hoạt động ===================== */

static void imu_task(void *arg)
{
    activity_classifier_init();
    imu_sample_t sample;
    for (;;) {
        if (imu_driver_read(&sample) == ESP_OK) {
            uint8_t confidence;
            activity_class_t act = activity_classifier_update(&sample, &confidence);

            app_event_t evt = { .type = -1 };
            switch (act) {
                case ACTIVITY_WALKING:
                case ACTIVITY_RUNNING:
                    evt.type = EVT_STEP_DETECTED;
                    break;
                case ACTIVITY_FALL:
                case ACTIVITY_DEVICE_DROPPED:
                    evt.type = EVT_FALL_SUSPECTED;
                    break;
                default:
                    break;
            }
            if (evt.type != -1) {
                state_machine_post_event(evt);
            }

            float heading = 0;
            esp_err_t mag_err = mag_driver_read_heading(sample.ax, sample.ay, sample.az, &heading);
            s_mag_ok = (mag_err == ESP_OK);
            if ((act == ACTIVITY_WALKING || act == ACTIVITY_RUNNING) && s_mag_ok) {
                pdr_on_step(heading);
            }

            s_latest_imu = sample;
            s_latest_heading = heading;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===================== Task: GNSS ===================== */

static void gnss_task(void *arg)
{
    for (;;) {
        gnss_fix_t fix;
        esp_err_t err = gnss_driver_read_fix(&fix, 1000);
        s_gnss_fix = (err == ESP_OK && fix.has_fix);
        if (s_gnss_fix) {
            state_machine_post_event((app_event_t){ .type = EVT_GNSS_FIX_ACQUIRED });
        } else {
            state_machine_post_event((app_event_t){ .type = EVT_GNSS_FIX_LOST });
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ===================== Task: LoRa ===================== */

static void lora_task(void *arg)
{
    int64_t last_heartbeat_us = 0;
    for (;;) {
        system_state_t st = state_machine_get_state();

        if (st == SYS_STATE_SOS) {
            sos_packet_t pkt = {
                .source_id = 0x0001,
                .packet_type = PKT_TYPE_SOS,
                .hop_limit = 5,
                .battery_pct = battery_monitor_read_percent(),
            };
            lora_driver_send_sos(&pkt);
        }
        (void)last_heartbeat_us;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ===================== Task: hiển thị OLED ===================== */

static void display_task(void *arg)
{
    for (;;) {
        if (!s_display_ok) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (display_driver_should_update(500)) {
            system_state_t st = state_machine_get_state();
            uint8_t batt = battery_monitor_read_percent();

            display_driver_update(st, batt, s_latest_heading,
                                   s_latest_imu.ax, s_latest_imu.ay, s_latest_imu.az,
                                   s_latest_imu.gx, s_latest_imu.gy, s_latest_imu.gz,
                                   s_latest_alt, s_latest_temp, s_latest_pressure,
                                   s_gnss_fix, s_mag_ok);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ===================== Task: state machine trung tâm ===================== */

static void state_machine_task(void *arg)
{
    static system_state_t last_state = SYS_STATE_IDLE;
    for (;;) {
        state_machine_process();
        system_state_t st = state_machine_get_state();
        if (st != last_state) {
            power_mgmt_on_state_change(st);
            last_state = st;
        }
    }
}

/* ===================== Task: theo dõi pin ===================== */

static void battery_task(void *arg)
{
    for (;;) {
        uint8_t pct = battery_monitor_read_percent();
        if (pct < 15) {
            state_machine_post_event((app_event_t){ .type = EVT_BATTERY_LOW });
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* ===================== Task: đọc cảm biến liên tục (debug) ===================== */

static void sensors_debug_task(void *arg)
{
    for (;;) {
        ESP_LOGI("SENSORS", "IMU  ax=%.2f ay=%.2f az=%.2f gx=%.1f gy=%.1f gz=%.1f",
                 s_latest_imu.ax, s_latest_imu.ay, s_latest_imu.az,
                 s_latest_imu.gx, s_latest_imu.gy, s_latest_imu.gz);

        ESP_LOGI("SENSORS", "MAG  heading=%.1f deg", s_latest_heading);

        float pressure, temp;
        if (baro_driver_read(&pressure, &temp) == ESP_OK) {
            s_latest_pressure = pressure;
            s_latest_temp = temp;
            s_latest_alt = baro_driver_altitude_m(pressure, 101325.0f);
            ESP_LOGI("SENSORS", "BARO p=%.1f Pa  t=%.2f C  alt=%.1f m",
                     pressure, temp, s_latest_alt);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ===================== app_main ===================== */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    init_i2c_bus();

    /* ---- Scan I2C để debug ---- */
    vTaskDelay(pdMS_TO_TICKS(200));
    i2c_scan();

    /* ---- Khởi tạo toàn bộ hệ thống ---- */
    init_buttons();

    ESP_ERROR_CHECK(imu_driver_init());
    ESP_ERROR_CHECK(imu_driver_calibrate(200));

    esp_err_t mag_err = mag_driver_init();
    if (mag_err != ESP_OK) {
        ESP_LOGE(TAG, "mag_driver_init FAILED: %s - tiếp tục chạy không có mag", esp_err_to_name(mag_err));
    }

    ESP_ERROR_CHECK(baro_driver_init());
    ESP_ERROR_CHECK(gnss_driver_init());
    ESP_ERROR_CHECK(lora_driver_init());
    s_display_ok = (display_driver_init() == ESP_OK);
    if (!s_display_ok) {
        ESP_LOGW(TAG, "display_driver_init FAILED - tiếp tục không có OLED");
    }
    ESP_ERROR_CHECK(battery_monitor_init());
    ESP_ERROR_CHECK(power_mgmt_init());

    state_machine_init();
    pdr_reset();

    xTaskCreate(imu_task,           "imu_task",  4096, NULL, 5, NULL);
    xTaskCreate(gnss_task,          "gnss_task", 4096, NULL, 4, NULL);
    xTaskCreate(lora_task,          "lora_task", 4096, NULL, 4, NULL);
    xTaskCreate(display_task,       "disp_task", 4096, NULL, 3, NULL);
    xTaskCreate(state_machine_task, "sm_task",   4096, NULL, 6, NULL);
    xTaskCreate(battery_task,       "batt_task", 2048, NULL, 2, NULL);
    xTaskCreate(sensors_debug_task, "sens_task", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "esp32_rescue_node started");
}