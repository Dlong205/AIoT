#include "display_driver.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "display_driver";
static int64_t s_last_update_us = 0;

static const char *state_to_str(system_state_t s)
{
    switch (s) {
        case SYS_STATE_IDLE: return "IDLE";
        case SYS_STATE_WALKING: return "WALKING";
        case SYS_STATE_FALL_SUSPECTED: return "FALL?";
        case SYS_STATE_SOS: return "SOS";
        case SYS_STATE_RETURN: return "RETURN";
        case SYS_STATE_LOW_POWER: return "LOW POWER";
        default: return "?";
    }
}

esp_err_t display_driver_init(void)
{
    /* TODO: gửi chuỗi lệnh init chuẩn SSD1306 (charge pump, addressing mode,
     * contrast...) qua I2C @SSD1306_I2C_ADDR, dùng thư viện font đơn giản
     * (5x7) để vẽ chữ + 1 icon mũi tên cho chế độ Return. */
    ESP_LOGI(TAG, "SSD1306 init @0x%02X", SSD1306_I2C_ADDR);
    return ESP_OK;
}

esp_err_t display_driver_draw_status(system_state_t state,
                                      uint8_t battery_pct,
                                      bool show_return_arrow,
                                      float heading_to_target_deg)
{
    /* TODO: vẽ text "STATE" + "%pin" lên buffer, nếu show_return_arrow thì
     * vẽ thêm 1 mũi tên xoay theo heading_to_target_deg, rồi flush buffer
     * ra OLED qua I2C. */
    ESP_LOGD(TAG, "state=%s batt=%d%% arrow=%d heading=%.1f",
             state_to_str(state), battery_pct, show_return_arrow, heading_to_target_deg);
    return ESP_OK;
}

bool display_driver_should_update(uint32_t min_interval_ms)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_update_us < (int64_t)min_interval_ms * 1000) {
        return false;
    }
    s_last_update_us = now;
    return true;
}
