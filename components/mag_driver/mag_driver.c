#include "mag_driver.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "mag_driver";

esp_err_t mag_driver_init(void)
{
    /* TODO: ghi thanh ghi Control Register 1 (0x09): mode=continuous,
     * ODR=200Hz, full scale=8G, OSR=512. QMC5883L chân "compass" - nên
     * hiệu chuẩn (calibration hình 8) trước khi dùng cho PDR heading. */
    ESP_LOGI(TAG, "QMC5883L init @0x%02X", QMC5883L_I2C_ADDR);
    return ESP_OK;
}

esp_err_t mag_driver_read_heading(float ax, float ay, float az, float *out_heading_deg)
{
    /* TODO:
     * 1. Đọc raw X/Y/Z từ thanh ghi 0x00-0x05.
     * 2. Áp offset/scale đã hiệu chuẩn.
     * 3. Bù nghiêng (tilt compensation) bằng ax, ay, az từ IMU.
     * 4. atan2 -> heading, quy về [0, 360).
     */
    (void)ax; (void)ay; (void)az;
    *out_heading_deg = 0.0f;
    return ESP_OK;
}
