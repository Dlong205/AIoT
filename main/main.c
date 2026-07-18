#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

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
#include "mesh_protocol.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

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
        esp_err_t err = gnss_driver_read_fix(&fix, 100);
        s_gnss_fix = (err == ESP_OK && fix.has_fix);
        if (s_gnss_fix) {
            state_machine_post_event((app_event_t){ .type = EVT_GNSS_FIX_ACQUIRED });
        } else {
            /* Không spam event khi chưa có fix */
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ===================== Task: LoRa ===================== */

static void lora_task(void *arg)
{
    static uint32_t s_msg_id = 0;
    sos_packet_t rx_pkt;
    for (;;) {
        if (state_machine_get_state() == SYS_STATE_SOS) {
            if (s_msg_id == 0 || (s_msg_id % 5 == 0)) {
                sos_packet_t pkt = {
                    .source_id = 0x0001,
                    .message_id = s_msg_id,
                    .packet_type = PKT_TYPE_SOS,
                    .hop_limit = 5,
                    .battery_pct = battery_monitor_read_percent(),
                };
                lora_driver_send_sos(&pkt);
            }
            s_msg_id++;
        }

        uint8_t buf[sizeof(sos_packet_t)];
        int len = uart_read_bytes(AS32_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len >= (int)sizeof(sos_packet_t)) {
            if (mesh_protocol_unpack(buf, len, &rx_pkt)) {
                ESP_LOGI("AS32", "RX type=%d src=%lu",
                         rx_pkt.packet_type, (unsigned long)rx_pkt.source_id);
                if (rx_pkt.packet_type == PKT_TYPE_SOS &&
                    mesh_protocol_should_forward(&rx_pkt))
                {
                    lora_driver_send_sos(&rx_pkt);
                    ESP_LOGI("AS32", "FWD hop=%d/%d",
                             rx_pkt.hop_count, rx_pkt.hop_limit);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
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

        /* Doc raw MAG truc tiep de debug */
        {
            uint8_t reg = 0x01, raw6[6];
            if (i2c_master_write_read_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, &reg, 1, raw6, 6, pdMS_TO_TICKS(50)) == ESP_OK) {
                int16_t mx = (int16_t)((raw6[1] << 8) | raw6[0]);
                int16_t my = (int16_t)((raw6[3] << 8) | raw6[2]);
                int16_t mz = (int16_t)((raw6[5] << 8) | raw6[4]);
                uint8_t st = 0;
                reg = 0x09;
                i2c_master_write_read_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, &reg, 1, &st, 1, pdMS_TO_TICKS(50));
                ESP_LOGI("SENSORS", "MAG_RAW mx=%d my=%d mz=%d status=0x%02X", mx, my, mz, st);
            } else {
                ESP_LOGW("SENSORS", "MAG_RAW I2C read FAILED");
            }
        }

        float pressure, temp;
        if (baro_driver_read(&pressure, &temp) == ESP_OK) {
            s_latest_pressure = pressure;
            s_latest_temp = temp;
            s_latest_alt = baro_driver_altitude_m(pressure, 101325.0f);
            ESP_LOGI("SENSORS", "BARO p=%.1f Pa  t=%.2f C  alt=%.1f m",
                     pressure, temp, s_latest_alt);
        }

        /* Đọc raw NMEA từ GPS - in ra để debug */
        uint8_t raw[256];
        int len = uart_read_bytes(GNSS_UART_PORT, raw, sizeof(raw) - 1, pdMS_TO_TICKS(200));
        if (len > 0) {
            raw[len] = '\0';
            ESP_LOGI("GNSS_RAW", "%s", (char *)raw);
        } else {
            ESP_LOGI("GNSS_RAW", "no UART data");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ===================== SD Card: mount + ghi log ===================== */

static sdmmc_card_t *s_sd_card = NULL;

static esp_err_t sd_card_init(void)
{
    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    host.max_freq_khz = 400; /* tốc độ thấp cho init ổn định */

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS_GPIO;
    slot_cfg.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &s_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint64_t size = ((uint64_t)s_sd_card->csd.capacity) * s_sd_card->csd.sector_size;
    ESP_LOGI(TAG, "SD card mounted: %llu MB", size / (1024 * 1024));

    /* Ghi header CSV nếu file chưa tồn tại */
    FILE *f = fopen(SD_MOUNT_POINT "/sensor_log.csv", "r");
    if (!f) {
        f = fopen(SD_MOUNT_POINT "/sensor_log.csv", "w");
        if (f) {
            fprintf(f, "timestamp_ms,ax,ay,az,gx,gy,gz,heading_deg,press_pa,temp_c,alt_m,gnss_lat,gnss_lon,gnss_hdop,gnss_sat\n");
            fclose(f);
            ESP_LOGI(TAG, "CSV header written");
        }
    } else {
        fclose(f);
    }
    return ESP_OK;
}

static void sd_log_task(void *arg)
{
    int64_t start = esp_timer_get_time();
    for (;;) {
        if (!s_sd_card) {
            ESP_LOGW(TAG, "SD card not mounted - skip log");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        FILE *f = fopen(SD_MOUNT_POINT "/sensor_log.csv", "a");
        if (!f) {
            ESP_LOGW(TAG, "Cannot open CSV for append");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        int64_t now = esp_timer_get_time();
        int64_t t = (now - start) / 1000;

        gnss_fix_t fix;
        fix.has_fix = false;
        fix.lat = 0; fix.lon = 0; fix.hdop = 0; fix.satellites = 0;
        if (gnss_driver_read_fix(&fix, 200) != ESP_OK) {
            fix.has_fix = false;
        }

        fprintf(f, "%lld,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%.6f,%.6f,%.1f,%d\n",
                t,
                s_latest_imu.ax, s_latest_imu.ay, s_latest_imu.az,
                s_latest_imu.gx, s_latest_imu.gy, s_latest_imu.gz,
                s_latest_heading,
                s_latest_pressure, s_latest_temp, s_latest_alt,
                fix.lat, fix.lon, fix.hdop, fix.satellites);
        fflush(f);
        fsync(fileno(f));
        fclose(f);
        ESP_LOGI(TAG, "SD log written: line %lld", t / 5000);
        vTaskDelay(pdMS_TO_TICKS(5000));
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

    esp_err_t imu_err = imu_driver_init();
    if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "imu_driver_init FAILED: %s - tiếp tục không có IMU", esp_err_to_name(imu_err));
    } else {
        ESP_ERROR_CHECK(imu_driver_calibrate(200));
    }

    esp_err_t mag_err = mag_driver_init();
    if (mag_err != ESP_OK) {
        ESP_LOGW(TAG, "mag_driver_init FAILED: %s - tiếp tục không có MAG", esp_err_to_name(mag_err));
    }

    esp_err_t baro_err = baro_driver_init();
    if (baro_err != ESP_OK) {
        ESP_LOGW(TAG, "baro_driver_init FAILED: %s - tiếp tục không có BARO", esp_err_to_name(baro_err));
    }

    ESP_ERROR_CHECK(gnss_driver_init());
    esp_err_t lora_err = lora_driver_init();
    if (lora_err != ESP_OK) {
        ESP_LOGW(TAG, "lora_driver_init FAILED: %s - tiếp tục không có AS32", esp_err_to_name(lora_err));
    }
    s_display_ok = (display_driver_init() == ESP_OK);
    if (!s_display_ok) {
        ESP_LOGW(TAG, "display_driver_init FAILED - tiếp tục không có OLED");
    }
    ESP_ERROR_CHECK(battery_monitor_init());
    ESP_ERROR_CHECK(power_mgmt_init());

    esp_err_t sd_err = sd_card_init();
    if (sd_err != ESP_OK) {
        ESP_LOGW(TAG, "SD card init FAILED - tiếp tục không ghi log");
    }

    state_machine_init();
    pdr_reset();

    xTaskCreate(imu_task,           "imu_task",  4096, NULL, 5, NULL);
    xTaskCreate(gnss_task,          "gnss_task", 4096, NULL, 4, NULL);
    xTaskCreate(lora_task,          "lora_task", 4096, NULL, 4, NULL);
    xTaskCreate(display_task,       "disp_task", 4096, NULL, 3, NULL);
    xTaskCreate(state_machine_task, "sm_task",   4096, NULL, 6, NULL);
    xTaskCreate(battery_task,       "batt_task", 2048, NULL, 2, NULL);
    xTaskCreate(sensors_debug_task, "sens_task", 4096, NULL, 2, NULL);
    xTaskCreate(sd_log_task,        "sd_task",   4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "esp32_rescue_node started");
}