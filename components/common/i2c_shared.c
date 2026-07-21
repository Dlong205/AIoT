#include "i2c_shared.h"
#include "app_config.h"
#include "esp_log.h"

static const char *TAG = "i2c_shared";

i2c_master_bus_handle_t i2c0_bus = NULL;
i2c_master_bus_handle_t i2c1_bus = NULL;

esp_err_t i2c_shared_init(void)
{
    esp_err_t err;

    /* I2C0: sensors (IMU, MAG, BARO) */
    i2c_master_bus_config_t cfg0 = {
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&cfg0, &i2c0_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus(I2C0) failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C0 init OK (SDA=%d, SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);

    /* I2C1: OLED only */
    i2c_master_bus_config_t cfg1 = {
        .i2c_port = I2C_OLED_BUS_PORT,
        .sda_io_num = I2C_OLED_SDA_GPIO,
        .scl_io_num = I2C_OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&cfg1, &i2c1_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus(I2C1) failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C1 init OK (SDA=%d, SCL=%d)", I2C_OLED_SDA_GPIO, I2C_OLED_SCL_GPIO);

    return ESP_OK;
}
