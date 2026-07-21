#include "baro_driver.h"
#include "app_config.h"
#include "i2c_shared.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "baro_driver";
static i2c_master_dev_handle_t s_dev = NULL;

/* BMP280 registers */
#define REG_CHIP_ID     0xD0
#define REG_RESET       0xE0
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_PRESS_MSB   0xF7
#define REG_TEMP_MSB    0xFA
#define REG_CALIB_START 0x88

#define BMP280_CHIP_ID  0x58

/* Calibration data */
typedef struct {
    uint16_t T1;
    int16_t  T2, T3;
    uint16_t P1;
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calib_t;

static bmp280_calib_t s_cal;
static int32_t s_t_fine;

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, 100);
}

static esp_err_t read_calib(void)
{
    uint8_t buf[24];
    esp_err_t err = read_regs(REG_CALIB_START, buf, 24);
    if (err != ESP_OK) return err;

    s_cal.T1 = (uint16_t)(buf[0] | (buf[1] << 8));
    s_cal.T2 = (int16_t)(buf[2] | (buf[3] << 8));
    s_cal.T3 = (int16_t)(buf[4] | (buf[5] << 8));
    s_cal.P1 = (uint16_t)(buf[6] | (buf[7] << 8));
    s_cal.P2 = (int16_t)(buf[8] | (buf[9] << 8));
    s_cal.P3 = (int16_t)(buf[10] | (buf[11] << 8));
    s_cal.P4 = (int16_t)(buf[12] | (buf[13] << 8));
    s_cal.P5 = (int16_t)(buf[14] | (buf[15] << 8));
    s_cal.P6 = (int16_t)(buf[16] | (buf[17] << 8));
    s_cal.P7 = (int16_t)(buf[18] | (buf[19] << 8));
    s_cal.P8 = (int16_t)(buf[20] | (buf[21] << 8));
    s_cal.P9 = (int16_t)(buf[22] | (buf[23] << 8));

    return ESP_OK;
}

static int32_t compensate_temp(int32_t adc_T)
{
    int32_t var1 = (((adc_T >> 3) - ((int32_t)s_cal.T1 << 1)) * ((int32_t)s_cal.T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_cal.T1)) * ((adc_T >> 4) - ((int32_t)s_cal.T1))) >> 12) * ((int32_t)s_cal.T3)) >> 14;
    s_t_fine = var1 + var2;
    return (s_t_fine * 5 + 128) >> 8;
}

static uint32_t compensate_pressure(int32_t adc_P)
{
    int64_t var1 = ((int64_t)s_t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_cal.P6;
    var2 = var2 + ((var1 * (int64_t)s_cal.P5) << 17);
    var2 = var2 + (((int64_t)s_cal.P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_cal.P3) >> 8) + ((var1 * (int64_t)s_cal.P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_cal.P1) >> 33;

    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_cal.P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_cal.P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_cal.P7) << 4);
    return (uint32_t)p;
}

esp_err_t baro_driver_init(void)
{
    if (i2c0_bus == NULL) {
        ESP_LOGE(TAG, "I2C0 bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(i2c0_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t chip_id;
    err = read_regs(REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "I2C read CHIP_ID failed: %s (id=0x%02X)", esp_err_to_name(err), chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP280 CHIP_ID=0x%02X", chip_id);

    /* BUG11 FIX: reset TRƯỚC, rồi mới đọc calibration */
    write_reg(REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(50));

    err = read_calib();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read calibration failed");
        return err;
    }

    /* crtl_meas: oversampling x16 for both, normal mode */
    write_reg(REG_CTRL_MEAS, 0x57);
    /* config: standby 500ms, filter x16 */
    write_reg(REG_CONFIG, 0x18);

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "BMP280 init OK (oversampling x16, normal mode)");
    return ESP_OK;
}

esp_err_t baro_driver_read(float *out_pressure_pa, float *out_temp_c)
{
    uint8_t raw[6];
    esp_err_t err = read_regs(REG_PRESS_MSB, raw, 6);
    if (err != ESP_OK) return err;

    int32_t adc_P = (int32_t)((raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4));
    int32_t adc_T = (int32_t)((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4));

    int32_t t = compensate_temp(adc_T);
    uint32_t p = compensate_pressure(adc_P);

    *out_temp_c = t / 100.0f;
    *out_pressure_pa = p / 256.0f;

    return ESP_OK;
}

float baro_driver_altitude_m(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 1.0f / 5.255f));
}
