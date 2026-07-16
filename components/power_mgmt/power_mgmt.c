#include "power_mgmt.h"
#include "app_config.h"
#include "esp_sleep.h"
#include "esp_log.h"

static const char *TAG = "power_mgmt";

esp_err_t power_mgmt_init(void)
{
    /* TODO: cấu hình wakeup source từ ngắt GPIO của IMU (INT pin MPU6050)
     * bằng esp_sleep_enable_ext1_wakeup() hoặc GPIO wakeup tương ứng trên
     * ESP32-S3, để MCU tự thức khi có chuyển động thay vì poll liên tục. */
    ESP_LOGI(TAG, "power_mgmt init");
    return ESP_OK;
}

void power_mgmt_on_state_change(system_state_t new_state)
{
    switch (new_state) {
    case SYS_STATE_IDLE:
        /* TODO: gnss_driver_power(false); giảm imu_task xuống sample rate
         * thấp hoặc chờ ngắt wake-on-motion thay vì poll. */
        ESP_LOGI(TAG, "policy: IDLE -> tiết kiệm tối đa, GNSS off");
        break;
    case SYS_STATE_WALKING:
        ESP_LOGI(TAG, "policy: WALKING -> active, GNSS bật định kỳ");
        break;
    case SYS_STATE_FALL_SUSPECTED:
    case SYS_STATE_SOS:
    case SYS_STATE_RETURN:
        ESP_LOGI(TAG, "policy: full active, khong duoc sleep");
        break;
    case SYS_STATE_LOW_POWER:
        ESP_LOGI(TAG, "policy: LOW_POWER -> giam tan suat heartbeat/display, GNSS off");
        break;
    }
}

void power_mgmt_enter_deep_sleep(void)
{
    /* TODO: gọi trước khi sleep: tắt GNSS/LoRa nếu không cần wake bởi chúng,
     * lưu breadcrumb/state cần thiết vào NVS vì Deep Sleep mất RAM. */
    ESP_LOGI(TAG, "entering deep sleep...");
    esp_deep_sleep_start();
}
