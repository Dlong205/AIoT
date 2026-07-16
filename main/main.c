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
        .intr_type = GPIO_INTR_NEGEDGE, /* TODO: gắn ISR handler, post
                                            EVT_BTN_SOS_PRESSED / CANCEL vào
                                            state_machine từ ISR-safe queue */
    };
    gpio_config(&cfg);
}

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

            /* Mỗi bước đi -> cập nhật PDR bằng heading hiện tại */
            if (act == ACTIVITY_WALKING || act == ACTIVITY_RUNNING) {
                float heading;
                mag_driver_read_heading(sample.ax, sample.ay, sample.az, &heading);
                pdr_on_step(heading);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); /* ~50Hz, chỉnh theo yêu cầu classifier */
    }
}

/* ===================== Task: GNSS (chạy khi cần P0) ===================== */

static void gnss_task(void *arg)
{
    for (;;) {
        gnss_fix_t fix;
        esp_err_t err = gnss_driver_read_fix(&fix, 1000);
        if (err == ESP_OK && fix.has_fix) {
            /* TODO: buffer GNSS_FIX_BUFFER_COUNT fix liên tiếp có HDOP tốt
             * trước khi chấp nhận làm P0 mới, rồi pdr_reset(). */
            state_machine_post_event((app_event_t){ .type = EVT_GNSS_FIX_ACQUIRED });
        } else {
            state_machine_post_event((app_event_t){ .type = EVT_GNSS_FIX_LOST });
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ===================== Task: LoRa (chỉ gửi khi SOS/heartbeat) ===================== */

static void lora_task(void *arg)
{
    int64_t last_heartbeat_us = 0;
    for (;;) {
        system_state_t st = state_machine_get_state();

        if (st == SYS_STATE_SOS) {
            sos_packet_t pkt = {
                .source_id = 0x0001, /* TODO: ID duy nhất từ efuse/MAC */
                .packet_type = PKT_TYPE_SOS,
                .hop_limit = 5,
                .battery_pct = battery_monitor_read_percent(),
                /* TODO: điền lat/lon (nếu absolute) hoặc rel_x/rel_y từ pdr_get_position() */
            };
            lora_driver_send_sos(&pkt);
        }
        /* TODO: heartbeat định kỳ theo HEARTBEAT_INTERVAL_NORMAL_MS /
         * HEARTBEAT_INTERVAL_LOWBAT_MS tùy trạng thái pin. */
        (void)last_heartbeat_us;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ===================== Task: hiển thị OLED ===================== */

static void display_task(void *arg)
{
    for (;;) {
        if (display_driver_should_update(500)) {
            system_state_t st = state_machine_get_state();
            uint8_t batt = battery_monitor_read_percent();

            if (st == SYS_STATE_RETURN) {
                float heading_to_target, distance;
                pdr_get_return_direction(0 /* TODO: heading hiện tại từ mag */,
                                          &heading_to_target, &distance);
                display_driver_draw_status(st, batt, true, heading_to_target);
            } else {
                display_driver_draw_status(st, batt, false, 0);
            }
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
        if (pct < 15) { /* TODO: đưa ngưỡng này vào app_config.h */
            state_machine_post_event((app_event_t){ .type = EVT_BATTERY_LOW });
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* ===================== app_main ===================== */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    init_i2c_bus();
    init_buttons();

    ESP_ERROR_CHECK(imu_driver_init());
    ESP_ERROR_CHECK(imu_driver_calibrate(200));
    ESP_ERROR_CHECK(mag_driver_init());
    ESP_ERROR_CHECK(gnss_driver_init());
    ESP_ERROR_CHECK(lora_driver_init());
    ESP_ERROR_CHECK(display_driver_init());
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

    ESP_LOGI(TAG, "esp32_rescue_node started");
}
