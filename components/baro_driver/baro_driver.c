#include "baro_driver.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "baro_driver";

#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_CHIP_ID_VAL     0x58
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7   /* press(3B) + temp(3B) liên tiếp */
#define BMP280_REG_CALIB_START 0x88   /* 24 byte calib, xem datasheet 3.11.2 */

/* osrs_t=x1(001) osrs_p=x1(001) mode=normal(11) -> 0b00100111 */
#define BMP280_CTRL_MEAS_VAL   0x27
/* t_sb=0.5ms(000) filter=off(000) spi3w=0 -> 0b00000000 */
#define BMP280_CONFIG_VAL      0x00

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

static bmp280_calib_t s_calib;
static int32_t s_t_fine; /* dùng chung giữa compensate T và P, theo đúng datasheet */

static inline esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_BUS_PORT, BMP280_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static inline esp_err_t i2c_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_BUS_PORT, BMP280_I2C_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

static esp_err_t read_calib(void)
{
    uint8_t c[24];
    esp_err_t err = i2c_read_regs(BMP280_REG_CALIB_START, c, sizeof(c));
    if (err != ESP_OK) return err;

    s_calib.dig_T1 = (uint16_t)(c[1] << 8 | c[0]);
    s_calib.dig_T2 = (int16_t)(c[3] << 8 | c[2]);
    s_calib.dig_T3 = (int16_t)(c[5] << 8 | c[4]);
    s_calib.dig_P1 = (uint16_t)(c[7] << 8 | c[6]);
    s_calib.dig_P2 = (int16_t)(c[9] << 8 | c[8]);
    s_calib.dig_P3 = (int16_t)(c[11] << 8 | c[10]);
    s_calib.dig_P4 = (int16_t)(c[13] << 8 | c[12]);
    s_calib.dig_P5 = (int16_t)(c[15] << 8 | c[14]);
    s_calib.dig_P6 = (int16_t)(c[17] << 8 | c[16]);
    s_calib.dig_P7 = (int16_t)(c[19] << 8 | c[18]);
    s_calib.dig_P8 = (int16_t)(c[21] << 8 | c[20]);
    s_calib.dig_P9 = (int16_t)(c[23] << 8 | c[22]);
    return ESP_OK;
}

esp_err_t baro_driver_init(void)
{
    uint8_t chip_id = 0;
    esp_err_t err = i2c_read_regs(BMP280_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read CHIP_ID failed: %s", esp_err_to_name(err));
        return err;
    }
    if (chip_id != BMP280_CHIP_ID_VAL) {
        ESP_LOGW(TAG, "CHIP_ID = 0x%02X (expect 0x%02X)", chip_id, BMP280_CHIP_ID_VAL);
    }

    err = i2c_write_reg(BMP280_REG_RESET, 0xB6);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    err = read_calib();
    if (err != ESP_OK) return err;

    err = i2c_write_reg(BMP280_REG_CONFIG, BMP280_CONFIG_VAL);
    if (err != ESP_OK) return err;

    err = i2c_write_reg(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VAL);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "BMP280 init OK @0x%02X (normal mode, osrs x1)", BMP280_I2C_ADDR);
    return ESP_OK;
}

/* Công thức compensate lấy nguyên theo Bosch datasheet BMP280 §3.11.3 */
static float compensate_temperature(int32_t adc_T)
{
    float var1 = (((float)adc_T) / 16384.0f - ((float)s_calib.dig_T1) / 1024.0f) * ((float)s_calib.dig_T2);
    float var2 = ((((float)adc_T) / 131072.0f - ((float)s_calib.dig_T1) / 8192.0f) *
                  (((float)adc_T) / 131072.0f - ((float)s_calib.dig_T1) / 8192.0f)) * ((float)s_calib.dig_T3);
    s_t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

static float compensate_pressure(int32_t adc_P)
{
    float var1 = ((float)s_t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)s_calib.dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)s_calib.dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)s_calib.dig_P4) * 65536.0f);
    var1 = (((float)s_calib.dig_P3) * var1 * var1 / 524288.0f + ((float)s_calib.dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)s_calib.dig_P1);
    if (var1 == 0.0f) return 0.0f; /* tránh chia 0 */

    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)s_calib.dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)s_calib.dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)s_calib.dig_P7)) / 16.0f;
    return p; /* Pa */
}

esp_err_t baro_driver_read(float *out_pressure_pa, float *out_temperature_c)
{
    uint8_t raw[6];
    esp_err_t err = i2c_read_regs(BMP280_REG_PRESS_MSB, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

    /* PHẢI compensate temperature trước để có s_t_fine cho compensate pressure */
    *out_temperature_c = compensate_temperature(adc_T);
    *out_pressure_pa = compensate_pressure(adc_P);
    return ESP_OK;
}

float baro_driver_altitude_m(float pressure_pa, float sea_level_pa)
{
    /* Công thức barometric chuẩn, dùng cho ước lượng độ cao tương đối */
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}