#include "imu_driver.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "imu_driver";

/* ==================== Biến static lưu calibration bias ==================== */
static float s_accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float s_gyro_bias[3]  = {0.0f, 0.0f, 0.0f};
static bool s_calibrated = false;

/* ==================== Helpers I2C ==================== */
static inline esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_BUS_PORT, MPU6050_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static inline esp_err_t i2c_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_BUS_PORT, MPU6050_I2C_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

/* ==================== Init ==================== */
esp_err_t imu_driver_init(void)
{
    uint8_t who = 0;
    esp_err_t err = i2c_read_regs(MPU9250_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read WHO_AM_I failed: %s", esp_err_to_name(err));
        return err;
    }
    if (who != MPU9250_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X (expect 0x%02X) - vẫn tiếp tục, có thể clone", who, MPU9250_WHO_AM_I_VAL);
    } else {
        ESP_LOGI(TAG, "MPU9250 detected (WHO_AM_I=0x71)");
    }

    /* 1) Reset device */
    err = i2c_write_reg(MPU9250_REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 2) Wake up: auto-select clock source (PLL with X gyro ref) */
    err = i2c_write_reg(MPU9250_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 3) Disable secondary I2C (I2C master), không dùng AK8963 */
    err = i2c_write_reg(MPU9250_REG_INT_PIN_CFG, 0x00);
    if (err != ESP_OK) return err;

    /* 4) Sample rate divider -> 50Hz (gyro output rate = 1kHz/(1+19) = 50Hz) */
    err = i2c_write_reg(MPU9250_REG_SMPLRT_DIV, 19);
    if (err != ESP_OK) return err;

    /* 5) CONFIG: DLPF 44Hz cho accel+gyro (bit 2:0 = 3) */
    err = i2c_write_reg(MPU9250_REG_CONFIG, 0x03);
    if (err != ESP_OK) return err;

    /* 6) Gyro full-scale ±2000dps */
    err = i2c_write_reg(MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_FS_2000DPS);
    if (err != ESP_OK) return err;

    /* 7) Accel full-scale ±8g */
    err = i2c_write_reg(MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_FS_8G);
    if (err != ESP_OK) return err;

    /* 8) Accel config 2: DLPF 44Hz (bit 2:0 = 3) */
    err = i2c_write_reg(MPU9250_REG_ACCEL_CONFIG2, 0x03);
    if (err != ESP_OK) return err;

    /* 9) Power mgmt 2: disable sleep/standby */
    err = i2c_write_reg(MPU9250_REG_PWR_MGMT_2, 0x00);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "MPU9250 init done (50Hz, ±8g/±2000dps)");
    return ESP_OK;
}

/* ==================== Read sample ==================== */
esp_err_t imu_driver_read(imu_sample_t *out_sample)
{
    if (!out_sample) return ESP_ERR_INVALID_ARG;

    uint8_t raw[14];
    esp_err_t err = i2c_read_regs(MPU9250_REG_ACCEL_XOUT_H, raw, 14);
    if (err != ESP_OK) return err;

    int16_t ax = (raw[0]  << 8) | raw[1];
    int16_t ay = (raw[2]  << 8) | raw[3];
    int16_t az = (raw[4]  << 8) | raw[5];
    /* raw[6], raw[7] = temp (bỏ qua) */
    int16_t gx = (raw[8]  << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    out_sample->ax = (ax / MPU9250_ACCEL_SENSITIVITY_LSB_PER_G) - s_accel_bias[0];
    out_sample->ay = (ay / MPU9250_ACCEL_SENSITIVITY_LSB_PER_G) - s_accel_bias[1];
    out_sample->az = (az / MPU9250_ACCEL_SENSITIVITY_LSB_PER_G) - s_accel_bias[2];

    out_sample->gx = (gx / MPU9250_GYRO_SENSITIVITY_LSB_PER_DPS) - s_gyro_bias[0];
    out_sample->gy = (gy / MPU9250_GYRO_SENSITIVITY_LSB_PER_DPS) - s_gyro_bias[1];
    out_sample->gz = (gz / MPU9250_GYRO_SENSITIVITY_LSB_PER_DPS) - s_gyro_bias[2];

    out_sample->timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

/* ==================== Calibration ==================== */
esp_err_t imu_driver_calibrate(uint16_t num_samples)
{
    if (num_samples == 0) return ESP_ERR_INVALID_ARG;

    float sum_accel[3] = {0};
    float sum_gyro[3]  = {0};

    ESP_LOGI(TAG, "Calibrating... giữ board đứng yên (%u mẫu)", num_samples);

    for (uint16_t i = 0; i < num_samples; i++) {
        imu_sample_t sample;
        if (imu_driver_read(&sample) == ESP_OK) {
            sum_accel[0] += sample.ax;
            sum_accel[1] += sample.ay;
            sum_accel[2] += sample.az;
            sum_gyro[0]  += sample.gx;
            sum_gyro[1]  += sample.gy;
            sum_gyro[2]  += sample.gz;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); /* ~50Hz */
    }

    s_accel_bias[0] = sum_accel[0] / num_samples;
    s_accel_bias[1] = sum_accel[1] / num_samples;
    s_accel_bias[2] = sum_accel[2] / num_samples - 1.0f; /* trừ 1g ở trục Z */
    s_gyro_bias[0]  = sum_gyro[0]  / num_samples;
    s_gyro_bias[1]  = sum_gyro[1]  / num_samples;
    s_gyro_bias[2]  = sum_gyro[2]  / num_samples;

    s_calibrated = true;
    ESP_LOGI(TAG, "Cal done: accel_bias=[%.4f, %.4f, %.4f] g, gyro_bias=[%.4f, %.4f, %.4f] dps",
             s_accel_bias[0], s_accel_bias[1], s_accel_bias[2],
             s_gyro_bias[0],  s_gyro_bias[1],  s_gyro_bias[2]);
    return ESP_OK;
}

/* ==================== Wake-on-Motion (Accel Intelligence) ==================== */
esp_err_t imu_driver_enable_wake_on_motion(uint8_t threshold_mg)
{
    /* WOM_THR LSB = ~4mg */
    uint8_t wom_thr = (threshold_mg + 2) / 4;
    if (wom_thr == 0) wom_thr = 1;

    esp_err_t err;

    /* 1) Enable Accel Intelligence (WOM mode) */
    err = i2c_write_reg(MPU9250_REG_ACCEL_INTEL_CTRL,
                        MPU9250_ACCEL_INTEL_EN_BIT | MPU9250_ACCEL_INTEL_MODE_BIT);
    if (err != ESP_OK) return err;

    /* 2) Set threshold */
    err = i2c_write_reg(MPU9250_REG_WOM_THR, wom_thr);
    if (err != ESP_OK) return err;

    /* 3) Sample rate divider cho WOM (chỉ accel chạy, gyro off) */
    err = i2c_write_reg(MPU9250_REG_SMPLRT_DIV, 19); /* ~50Hz accel */
    if (err != ESP_OK) return err;

    /* 4) Enable WOM interrupt trên INT pin */
    err = i2c_write_reg(MPU9250_REG_INT_ENABLE, MPU9250_INT_ENABLE_WOM_BIT);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Wake-on-Motion enabled (thr=~%dmg, WOM_THR=%d)", threshold_mg, wom_thr);
    return ESP_OK;
}