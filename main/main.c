#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"

#include "app_config.h"
#include "rescue_types.h"

#include "i2c_shared.h"
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
    ESP_ERROR_CHECK(i2c_shared_init());
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

/* I2C scan removed — new driver uses bus probe. Devices found during init. */

/* ===================== Shared sensor data (cho OLED) ===================== */
static imu_sample_t s_latest_imu;
static float s_latest_heading = 0.0f;
static float s_latest_pressure = 0.0f;
static float s_latest_temp = 0.0f;
static float s_latest_alt = 0.0f;
static bool s_mag_ok = false;
static bool s_gnss_fix = false;
static bool s_display_ok = false;
static gnss_fix_t s_latest_gnss_fix;  /* BUG5 FIX: lưu GNSS fix tập trung */

/* BUG8 FIX: Mutex bảo vệ biến shared giữa các task (data race trên dual-core) */
static SemaphoreHandle_t s_sensor_mutex = NULL;

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
            bool mag_ok_local = (mag_err == ESP_OK);
            if ((act == ACTIVITY_WALKING || act == ACTIVITY_RUNNING) && mag_ok_local) {
                pdr_on_step(heading);
            }

            /* BUG8 FIX: bảo vệ ghi biến shared bằng mutex */
            if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_latest_imu = sample;
                s_latest_heading = heading;
                s_mag_ok = mag_ok_local;
                xSemaphoreGive(s_sensor_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===================== Task: GNSS ===================== */

static void gnss_task(void *arg)
{
    for (;;) {
        gnss_fix_t fix;
        /* BUG5 FIX: chỉ task này đọc GNSS UART, lưu kết quả vào biến shared */
        esp_err_t err = gnss_driver_read_fix(&fix, 100);
        bool has_fix = (err == ESP_OK && fix.has_fix);

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_gnss_fix = has_fix;
            if (has_fix) {
                s_latest_gnss_fix = fix;
            }
            xSemaphoreGive(s_sensor_mutex);
        }

        if (has_fix) {
            state_machine_post_event((app_event_t){ .type = EVT_GNSS_FIX_ACQUIRED });
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ===================== Task: LoRa ===================== */

static uint32_t s_node_source_id = 0;

static uint32_t get_node_id(void)
{
    if (s_node_source_id != 0) return s_node_source_id;
    /* Tạo source_id duy nhất từ 4 byte cuối MAC address */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    s_node_source_id = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                        ((uint32_t)mac[4] << 8)  | (uint32_t)mac[5];
    if (s_node_source_id == 0) s_node_source_id = 1;  /* tránh ID = 0 */
    return s_node_source_id;
}

static void lora_task(void *arg)
{
    static uint32_t s_msg_id = 0;
    sos_packet_t rx_pkt;
    for (;;) {
        if (state_machine_get_state() == SYS_STATE_SOS) {
            if (s_msg_id == 0 || (s_msg_id % 5 == 0)) {

                /* Lấy dữ liệu vị trí từ shared vars */
                gnss_fix_t gnss_copy;
                bool gnss_has_fix = false;
                if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gnss_copy = s_latest_gnss_fix;
                    gnss_has_fix = s_gnss_fix;
                    xSemaphoreGive(s_sensor_mutex);
                }

                /* Lấy vị trí PDR tương đối */
                position_t pdr_pos = pdr_get_position();

                /* Điền ĐẦY ĐỦ gói SOS */
                sos_packet_t pkt = {
                    .source_id = get_node_id(),
                    .message_id = s_msg_id,
                    .packet_type = PKT_TYPE_SOS,
                    .confidence = 0,  /* TODO: lấy từ activity_classifier khi implement */
                    .lat = gnss_has_fix ? gnss_copy.lat : 0.0,
                    .lon = gnss_has_fix ? gnss_copy.lon : 0.0,
                    .rel_x_m = pdr_pos.rel_x_m,
                    .rel_y_m = pdr_pos.rel_y_m,
                    .position_is_absolute = gnss_has_fix,
                    .battery_pct = battery_monitor_read_percent(),
                    .hop_count = 0,
                    .hop_limit = 5,
                };

                lora_driver_send_sos(&pkt);
                ESP_LOGI("SOS", "TX id=%lu src=0x%08lX pos=%s lat=%.6f lon=%.6f rel=(%.1f,%.1f) batt=%d%%",
                         (unsigned long)s_msg_id, (unsigned long)pkt.source_id,
                         gnss_has_fix ? "GPS" : "PDR",
                         pkt.lat, pkt.lon, pkt.rel_x_m, pkt.rel_y_m, pkt.battery_pct);
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

            /* BUG8 FIX: đọc shared vars dưới mutex */
            imu_sample_t imu_copy;
            float heading_copy, alt_copy, temp_copy, press_copy;
            bool gnss_copy, mag_copy;
            if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                imu_copy = s_latest_imu;
                heading_copy = s_latest_heading;
                alt_copy = s_latest_alt;
                temp_copy = s_latest_temp;
                press_copy = s_latest_pressure;
                gnss_copy = s_gnss_fix;
                mag_copy = s_mag_ok;
                xSemaphoreGive(s_sensor_mutex);
            } else {
                memset(&imu_copy, 0, sizeof(imu_copy));
                heading_copy = alt_copy = temp_copy = press_copy = 0;
                gnss_copy = mag_copy = false;
            }

            display_driver_update(st, batt, heading_copy,
                                   imu_copy.ax, imu_copy.ay, imu_copy.az,
                                   imu_copy.gx, imu_copy.gy, imu_copy.gz,
                                   alt_copy, temp_copy, press_copy,
                                   gnss_copy, mag_copy);
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
        /* BUG4 FIX: tránh chiếm CPU khi queue có event liên tục */
        vTaskDelay(pdMS_TO_TICKS(10));
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

/* ===================== Task: đọc nút SOS ===================== */

static void button_task(void *arg)
{
    for (;;) {
        if (gpio_get_level(BTN_SOS_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(BTN_SOS_GPIO) == 0) {
                ESP_LOGI("BTN", "SOS pressed!");
                state_machine_post_event((app_event_t){ .type = EVT_BTN_SOS_PRESSED });
                while (gpio_get_level(BTN_SOS_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        if (gpio_get_level(BTN_CANCEL_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (gpio_get_level(BTN_CANCEL_GPIO) == 0) {
                ESP_LOGI("BTN", "CANCEL pressed!");
                state_machine_post_event((app_event_t){ .type = EVT_BTN_CANCEL_PRESSED });
                while (gpio_get_level(BTN_CANCEL_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===================== Task: Buzzer cảnh báo ===================== */

static void buzzer_task(void *arg)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(BUZZER_GPIO, 0);

    for (;;) {
        if (state_machine_get_state() == SYS_STATE_SOS) {
            /* Kêu bíp bíp liên tục khi đang SOS */
            gpio_set_level(BUZZER_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(BUZZER_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            gpio_set_level(BUZZER_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* ===================== Task: đọc cảm biến liên tục (debug) ===================== */

static void sensors_debug_task(void *arg)
{
    for (;;) {
        /* BUG8 FIX: đọc shared vars dưới mutex */
        imu_sample_t imu_dbg;
        float heading_dbg;
        bool gnss_dbg;
        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu_dbg = s_latest_imu;
            heading_dbg = s_latest_heading;
            gnss_dbg = s_gnss_fix;
            xSemaphoreGive(s_sensor_mutex);
        } else {
            memset(&imu_dbg, 0, sizeof(imu_dbg));
            heading_dbg = 0;
            gnss_dbg = false;
        }

        ESP_LOGI("SENSORS", "IMU  ax=%.2f ay=%.2f az=%.2f gx=%.1f gy=%.1f gz=%.1f",
                 imu_dbg.ax, imu_dbg.ay, imu_dbg.az,
                 imu_dbg.gx, imu_dbg.gy, imu_dbg.gz);

        ESP_LOGI("SENSORS", "MAG  heading=%.1f deg", heading_dbg);

        float pressure, temp;
        if (baro_driver_read(&pressure, &temp) == ESP_OK) {
            float alt = baro_driver_altitude_m(pressure, 101325.0f);
            /* BUG8 FIX: ghi baro data dưới mutex */
            if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_latest_pressure = pressure;
                s_latest_temp = temp;
                s_latest_alt = alt;
                xSemaphoreGive(s_sensor_mutex);
            }
            ESP_LOGI("SENSORS", "BARO p=%.1f Pa  t=%.2f C  alt=%.1f m",
                     pressure, temp, alt);
        }

        /* BUG5 FIX: không đọc GNSS UART trực tiếp nữa, dùng shared var từ gnss_task */
        ESP_LOGI("SENSORS", "GNSS fix=%s", gnss_dbg ? "YES" : "NO");

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

        /* BUG5 FIX: đọc GNSS fix từ shared var thay vì gọi UART trực tiếp */
        gnss_fix_t fix;
        imu_sample_t imu_log;
        float heading_log, press_log, temp_log, alt_log;

        if (xSemaphoreTake(s_sensor_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            fix = s_latest_gnss_fix;
            imu_log = s_latest_imu;
            heading_log = s_latest_heading;
            press_log = s_latest_pressure;
            temp_log = s_latest_temp;
            alt_log = s_latest_alt;
            xSemaphoreGive(s_sensor_mutex);
        } else {
            memset(&fix, 0, sizeof(fix));
            memset(&imu_log, 0, sizeof(imu_log));
            heading_log = press_log = temp_log = alt_log = 0;
        }

        fprintf(f, "%lld,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%.6f,%.6f,%.1f,%d\n",
                t,
                imu_log.ax, imu_log.ay, imu_log.az,
                imu_log.gx, imu_log.gy, imu_log.gz,
                heading_log,
                press_log, temp_log, alt_log,
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

    /* ---- Khởi tạo toàn bộ hệ thống ---- */
    init_buttons();

    esp_err_t imu_err = imu_driver_init();
    if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "imu_driver_init FAILED: %s - tiếp tục không có IMU", esp_err_to_name(imu_err));
    } else {
        esp_err_t cal_err = imu_driver_calibrate(200);
        if (cal_err != ESP_OK) {
            ESP_LOGW(TAG, "imu_calibrate FAILED: %s - tiếp tục", esp_err_to_name(cal_err));
        }
    }

    esp_err_t mag_err = mag_driver_init();
    if (mag_err != ESP_OK) {
        ESP_LOGW(TAG, "mag_driver_init FAILED: %s - tiếp tục không có MAG", esp_err_to_name(mag_err));
    }

    esp_err_t baro_err = baro_driver_init();
    if (baro_err != ESP_OK) {
        ESP_LOGW(TAG, "baro_driver_init FAILED: %s - tiếp tục không có BARO", esp_err_to_name(baro_err));
    }

    esp_err_t gnss_err = gnss_driver_init();
    if (gnss_err != ESP_OK) {
        ESP_LOGW(TAG, "gnss_driver_init FAILED: %s - tiếp tục không có GNSS", esp_err_to_name(gnss_err));
    }
    esp_err_t lora_err = lora_driver_init();
    if (lora_err != ESP_OK) {
        ESP_LOGW(TAG, "lora_driver_init FAILED: %s - tiếp tục không có AS32", esp_err_to_name(lora_err));
    }
    s_display_ok = (display_driver_init() == ESP_OK);
    if (!s_display_ok) {
        ESP_LOGW(TAG, "display_driver_init FAILED - tiếp tục không có OLED");
    }
    esp_err_t batt_err = battery_monitor_init();
    if (batt_err != ESP_OK) {
        ESP_LOGW(TAG, "battery_monitor_init FAILED: %s", esp_err_to_name(batt_err));
    }
    esp_err_t pwr_err = power_mgmt_init();
    if (pwr_err != ESP_OK) {
        ESP_LOGW(TAG, "power_mgmt_init FAILED: %s", esp_err_to_name(pwr_err));
    }

    esp_err_t sd_err = sd_card_init();
    if (sd_err != ESP_OK) {
        ESP_LOGW(TAG, "SD card init FAILED - tiếp tục không ghi log");
    }

    /* BUG8 FIX: tạo mutex trước khi bắt đầu các task */
    s_sensor_mutex = xSemaphoreCreateMutex();
    configASSERT(s_sensor_mutex);

    /* Khởi tạo GNSS fix mặc định */
    memset(&s_latest_gnss_fix, 0, sizeof(s_latest_gnss_fix));

    state_machine_init();
    pdr_reset();

    xTaskCreate(imu_task,           "imu_task",  4096, NULL, 5, NULL);
    xTaskCreate(gnss_task,          "gnss_task", 4096, NULL, 4, NULL);
    xTaskCreate(lora_task,          "lora_task", 4096, NULL, 4, NULL);
    xTaskCreate(display_task,       "disp_task", 4096, NULL, 3, NULL);
    xTaskCreatePinnedToCore(state_machine_task, "state_machine", 3072, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(battery_task, "battery", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "buttons", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(buzzer_task, "buzzer", 2048, NULL, 3, NULL, 0);
    xTaskCreate(sensors_debug_task, "sens_task", 4096, NULL, 2, NULL);
    xTaskCreate(sd_log_task,        "sd_task",   4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "esp32_rescue_node started");
}