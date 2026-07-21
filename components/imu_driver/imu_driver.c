#include "imu_driver.h"
#include "app_config.h"
#include "i2c_shared.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "imu_driver";
static i2c_master_dev_handle_t s_dev = NULL;

/* BUG13 FIX: lưu gyro offset từ calibration */
static float s_gyro_offset_x = 0, s_gyro_offset_y = 0, s_gyro_offset_z = 0;

/* MPU9250 (dung nhu MPU6500: accel+gyro 6-axis) */
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_XOUT  0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_CONFIG       0x1A
#define REG_INT_ENABLE   0x38
#define REG_INT_PIN_CFG  0x37
#define REG_ACCEL_INTEL  0x69

#define MPU_DEV_ID      0x68    /* MPU6050 */
#define MPU6500_DEV_ID  0x70    /* MPU6500 */

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, 100);
}

esp_err_t imu_driver_init(void)
{
    if (i2c0_bus == NULL) {
        ESP_LOGE(TAG, "I2C0 bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(i2c0_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Probing MPU9250 @0x%02X...", MPU6050_I2C_ADDR);

    uint8_t who;
    err = read_regs(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read WHO_AM_I failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    /* BUG6 FIX: chấp nhận cả MPU6050 (0x68) lẫn MPU6500 (0x70) */
    if (who != MPU_DEV_ID && who != MPU6500_DEV_ID) {
        ESP_LOGE(TAG, "Unknown WHO_AM_I=0x%02X (expected 0x68 or 0x70)", who);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (%s)", who,
             who == MPU_DEV_ID ? "MPU6050" : "MPU6500");

    err = write_reg(REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    err = write_reg(REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    err = write_reg(REG_CONFIG, 0x01);
    if (err != ESP_OK) return err;
    err = write_reg(REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) return err;
    err = write_reg(REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) return err;

    /* Wake-on-motion: INT pin on motion */
    write_reg(REG_INT_PIN_CFG, 0x20);
    write_reg(REG_INT_ENABLE, 0x40);
    write_reg(REG_ACCEL_INTEL, 0xC0);

    ESP_LOGI(TAG, "MPU9250 init OK");
    return ESP_OK;
}

esp_err_t imu_driver_read(imu_sample_t *out)
{
    uint8_t raw[14];
    esp_err_t err = read_regs(REG_ACCEL_XOUT, raw, 14);
    if (err != ESP_OK) return err;

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    out->ax = ax / 16384.0f;
    out->ay = ay / 16384.0f;
    out->az = az / 16384.0f;
    /* BUG13 FIX: trừ gyro offset đã calibrate */
    out->gx = gx / 131.0f - s_gyro_offset_x;
    out->gy = gy / 131.0f - s_gyro_offset_y;
    out->gz = gz / 131.0f - s_gyro_offset_z;
    out->timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

esp_err_t imu_driver_calibrate(uint16_t sample_count)
{
    float sx = 0, sy = 0, sz = 0;
    uint32_t valid = 0;
    imu_sample_t s;
    for (uint32_t i = 0; i < sample_count; i++) {
        if (imu_driver_read(&s) == ESP_OK) {
            sx += s.gx; sy += s.gy; sz += s.gz;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (valid > 0) {
        /* BUG13 FIX: lưu offset để trừ khi đọc */
        s_gyro_offset_x = sx / valid;
        s_gyro_offset_y = sy / valid;
        s_gyro_offset_z = sz / valid;
        ESP_LOGI(TAG, "calibrate: gyro offset (%.2f, %.2f, %.2f) deg/s - SAVED",
                 s_gyro_offset_x, s_gyro_offset_y, s_gyro_offset_z);
    } else {
        ESP_LOGW(TAG, "calibrate: no valid samples!");
    }
    return ESP_OK;
}

esp_err_t imu_driver_enable_wake_on_motion(uint8_t threshold_mg)
{
    esp_err_t err;
    err = write_reg(REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) return err;
    err = write_reg(REG_INT_PIN_CFG, 0x20);
    if (err != ESP_OK) return err;
    err = write_reg(REG_INT_ENABLE, 0x40);
    if (err != ESP_OK) return err;
    (void)threshold_mg;
    err = write_reg(REG_ACCEL_INTEL, 0xC0);
    return err;
}
