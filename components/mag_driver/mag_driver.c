#include "mag_driver.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "mag_driver";

/* ==================== QMC5883P Register Map ====================
 * Chip: QMC5883P (GY-271)
 * I2C address: 0x2C
 * NOTE: register map KHAC QMC5883L!
 * ================================================================ */

/* Data output (6 bytes tu 0x01) */
#define REG_DATA_X_LSB      0x01
#define REG_DATA_X_MSB      0x02
#define REG_DATA_Y_LSB      0x03
#define REG_DATA_Y_MSB      0x04
#define REG_DATA_Z_LSB      0x05
#define REG_DATA_Z_MSB      0x06

#define REG_STATUS          0x09
#define STATUS_DRDY         (1 << 0)

#define REG_CTRL1           0x0A
#define REG_CTRL2           0x0B
#define REG_SIGN            0x29

/* CTRL2 bits */
#define CTRL2_SOFT_RST      (1 << 7)
#define CTRL2_SET_RESET_ON  (1 << 3)
#define CTRL2_RNG_8G        (1 << 2)

/* 0xC3 = mode=cont(11), ODR=10Hz(00), OSR=64(11) */
#define CTRL1_VAL           0xC3

static float s_offset[3] = {0,0,0};
static float s_scale[3]  = {1,1,1};
static float s_lsb_per_g = 3000.0f; /* ±8G */

/* ==================== I2C ==================== */

static esp_err_t wr(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

static uint8_t rd1(uint8_t reg) {
    uint8_t v;
    esp_err_t e = i2c_master_write_read_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, &reg, 1, &v, 1, pdMS_TO_TICKS(100));
    return e == ESP_OK ? v : 0xFF;
}

/* ==================== Init ==================== */

esp_err_t mag_driver_init(void)
{
    ESP_LOGI(TAG, "Init QMC5883P @0x%02X...", QMC5883L_I2C_ADDR);

    esp_err_t e;

    e = wr(REG_CTRL2, CTRL2_SOFT_RST);  /* soft reset */
    if (e != ESP_OK) { ESP_LOGE(TAG, "RST fail"); return e; }
    vTaskDelay(pdMS_TO_TICKS(20));

    e = wr(REG_SIGN, 0x06);
    if (e != ESP_OK) { ESP_LOGE(TAG, "SIGN fail"); return e; }

    e = wr(REG_CTRL2, CTRL2_SET_RESET_ON | CTRL2_RNG_8G); /* 0x08 */
    if (e != ESP_OK) { ESP_LOGE(TAG, "CTRL2 fail"); return e; }

    e = wr(REG_CTRL1, CTRL1_VAL); /* 0xC3 */
    if (e != ESP_OK) { ESP_LOGE(TAG, "CTRL1 fail"); return e; }

    vTaskDelay(pdMS_TO_TICKS(200));

    uint8_t s = rd1(REG_STATUS);
    ESP_LOGI(TAG, "Init done, status=0x%02X", s);
    return ESP_OK;
}

/* ==================== Read raw ==================== */

esp_err_t mag_read_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t s = rd1(REG_STATUS);
    if (!(s & STATUS_DRDY)) {
        return ESP_ERR_NOT_FOUND;  /* non-blocking */
    }

    uint8_t raw[6];
    esp_err_t e = rd(REG_DATA_X_LSB, raw, 6);
    if (e != ESP_OK) return e;

    *mx = (int16_t)((raw[1] << 8) | raw[0]);
    *my = (int16_t)((raw[3] << 8) | raw[2]);
    *mz = (int16_t)((raw[5] << 8) | raw[4]);
    return ESP_OK;
}

/* ==================== Heading ==================== */

esp_err_t mag_driver_read_heading(float ax, float ay, float az, float *out)
{
    int16_t rx, ry, rz;
    esp_err_t e = mag_read_raw(&rx, &ry, &rz);
    if (e != ESP_OK) { *out = 0; return e; }

    float mx = rx / s_lsb_per_g - s_offset[0];
    float my = ry / s_lsb_per_g - s_offset[1];
    float mz = rz / s_lsb_per_g - s_offset[2];

    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-6f) {  /* no accel -> simple heading */
        float h = atan2f(my, mx) * 180.0f / (float)M_PI;
        if (h < 0) h += 360.0f;
        *out = h;
        ESP_LOGI("MAG", "no_accel heading=%.1f mx=%.1f my=%.1f", h, mx, my);
        return ESP_OK;
    }

    float axn = ax/n, ayn = ay/n;
    float pitch = asinf(-axn);
    float roll  = asinf(ayn / cosf(pitch));
    float xh =  mx*cosf(pitch) + mz*sinf(pitch);
    float yh =  mx*sinf(roll)*sinf(pitch) + my*cosf(roll) - mz*sinf(roll)*cosf(pitch);

    float h = atan2f(yh, xh) * 180.0f / (float)M_PI;
    if (h < 0) h += 360.0f;
    *out = h;
    return ESP_OK;
}
