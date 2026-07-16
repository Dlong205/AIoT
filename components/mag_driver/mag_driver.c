#include "mag_driver.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "mag_driver";

/* ==================== QMC5883L Register Map ====================
 * Chip: QMC5883L (GY-271 module)
 * I2C address: 0x2C (theo scan thực tế)
 * ================================================================ */

/* Data output registers (6 bytes, read from 0x00) */
#define REG_DATA_X_LSB      0x00
#define REG_DATA_X_MSB      0x01
#define REG_DATA_Y_LSB      0x02
#define REG_DATA_Y_MSB      0x03
#define REG_DATA_Z_LSB      0x04
#define REG_DATA_Z_MSB      0x05

#define REG_STATUS          0x06
#define STATUS_DRDY         (1 << 0)   /* data ready */
#define STATUS_OVL          (1 << 1)   /* overflow */
#define STATUS_DOR          (1 << 2)   /* data skipped */

/* Temperature (not always available) */
#define REG_TEMP_LSB        0x07
#define REG_TEMP_MSB        0x08

/* Control registers */
#define REG_CTRL1           0x0B
#define CTRL1_MODE_STANDBY   (0x00)
#define CTRL1_MODE_CONT      (0x01)
#define CTRL1_ODR_10HZ      (0x00 << 2)
#define CTRL1_ODR_50HZ      (0x01 << 2)
#define CTRL1_ODR_100HZ     (0x02 << 2)
#define CTRL1_ODR_200HZ     (0x03 << 2)
#define CTRL1_RNG_2G        (0x00 << 4)
#define CTRL1_RNG_8G        (0x01 << 4)
#define CTRL1_OSR_512       (0x00 << 6)
#define CTRL1_OSR_256       (0x01 << 6)
#define CTRL1_OSR_128       (0x02 << 6)
#define CTRL1_OSR_64        (0x03 << 6)

#define REG_CTRL2           0x0C
#define CTRL2_SOFT_RST      (1 << 0)
#define CTRL2_I2C_DISABLE   (1 << 1)

#define REG_SET_RESET       0x0D
#define SR_SET              0x01
#define SR_RESET            0x00

/* Sensitivity: LSB per Gauss */
#define LSB_PER_GAUSS_2G    12000.0f
#define LSB_PER_GAUSS_8G    3000.0f

static float s_mag_offset[3] = {0.0f, 0.0f, 0.0f};
static float s_mag_scale[3]  = {1.0f, 1.0f, 1.0f};
static float s_lsb_per_gauss = LSB_PER_GAUSS_2G;

/* ==================== I2C Helpers ==================== */

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_BUS_PORT, QMC5883L_I2C_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

/* ==================== Init ==================== */

esp_err_t mag_driver_init(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing QMC5883L @0x%02X ...", QMC5883L_I2C_ADDR);

    /* 1. Soft reset */
    err = i2c_write_reg(REG_CTRL2, CTRL2_SOFT_RST);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 2. Set/Reset for degaussing */
    err = i2c_write_reg(REG_SET_RESET, SR_SET);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));
    err = i2c_write_reg(REG_SET_RESET, SR_RESET);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 3. CTRL1: continuous mode, 50Hz ODR, ±2G, OSR=512 */
    uint8_t ctrl1 = CTRL1_MODE_CONT | CTRL1_ODR_50HZ | CTRL1_RNG_2G | CTRL1_OSR_512;
    err = i2c_write_reg(REG_CTRL1, ctrl1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write CTRL1 failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 4. CTRL2: default (I2C enabled, no reset) */
    err = i2c_write_reg(REG_CTRL2, 0x00);
    if (err != ESP_OK) return err;

    s_lsb_per_gauss = LSB_PER_GAUSS_2G;

    ESP_LOGI(TAG, "QMC5883L init OK (50Hz, ±2G, OSR=512)");
    return ESP_OK;
}

/* ==================== Read raw data ==================== */

static esp_err_t mag_read_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t raw[6];
    esp_err_t err = i2c_read_regs(REG_DATA_X_LSB, raw, 6);
    if (err != ESP_OK) return err;

    *mx = (int16_t)((raw[1] << 8) | raw[0]);
    *my = (int16_t)((raw[3] << 8) | raw[2]);
    *mz = (int16_t)((raw[5] << 8) | raw[4]);
    return ESP_OK;
}

/* ==================== Read heading ==================== */

esp_err_t mag_driver_read_heading(float ax, float ay, float az, float *out_heading_deg)
{
    int16_t rx, ry, rz;
    esp_err_t err = mag_read_raw(&rx, &ry, &rz);
    if (err != ESP_OK) {
        *out_heading_deg = 0.0f;
        return err;
    }

    float mx = (rx / s_lsb_per_gauss - s_mag_offset[0]) * s_mag_scale[0];
    float my = (ry / s_lsb_per_gauss - s_mag_offset[1]) * s_mag_scale[1];
    float mz = (rz / s_lsb_per_gauss - s_mag_offset[2]) * s_mag_scale[2];

    /* Tilt compensation using accelerometer */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-3f) norm = 1.0f;
    float axn = ax / norm;
    float ayn = ay / norm;

    float pitch = asinf(-axn);
    float roll  = asinf(ayn / cosf(pitch));

    float xh =  mx * cosf(pitch) + mz * sinf(pitch);
    float yh =  mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    float heading = atan2f(yh, xh) * 180.0f / (float)M_PI;
    if (heading < 0) heading += 360.0f;

    *out_heading_deg = heading;
    return ESP_OK;
}