#include "gnss_driver.h"
#include "nmea_parser.h"
#include "app_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "gnss_driver";
#define GNSS_RX_BUF_SIZE 512
#define GNSS_LINE_BUF_SIZE 256

esp_err_t gnss_driver_init(void)
{
    uart_config_t cfg = {
        .baud_rate = GNSS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_PORT, GNSS_UART_TX_GPIO, GNSS_UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_PORT, GNSS_RX_BUF_SIZE * 2, 0, 0, NULL, 0));

    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << GNSS_POWER_EN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));
    gpio_set_level(GNSS_POWER_EN_GPIO, 1);

    ESP_LOGI(TAG, "GY-NEO7M UART init (baud=%d)", GNSS_UART_BAUD);
    return ESP_OK;
}

esp_err_t gnss_driver_power(bool on)
{
    gpio_set_level(GNSS_POWER_EN_GPIO, on ? 1 : 0);
    ESP_LOGI(TAG, "GNSS power %s", on ? "ON" : "OFF");
    return ESP_OK;
}

static bool read_line(char *out_line, size_t max_len, uint32_t timeout_ms)
{
    size_t idx = 0;
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < (int64_t)timeout_ms * 1000) {
        uint8_t byte;
        int len = uart_read_bytes(GNSS_UART_PORT, &byte, 1, pdMS_TO_TICKS(50));
        if (len <= 0) continue;

        if (byte == '\n') {
            out_line[idx] = '\0';
            return idx > 0;
        }
        if (byte != '\r' && idx < max_len - 1) {
            out_line[idx++] = (char)byte;
        }
    }
    out_line[idx] = '\0';
    return false;
}

esp_err_t gnss_driver_read_fix(gnss_fix_t *out_fix, uint32_t timeout_ms)
{
    char line[GNSS_LINE_BUF_SIZE];
    out_fix->has_fix = false;
    out_fix->timestamp_us = esp_timer_get_time();

    if (!read_line(line, sizeof(line), timeout_ms)) {
        return ESP_ERR_TIMEOUT;
    }

    nmea_fix_t nmea;
    if (!nmea_parse_line(line, &nmea)) {
        return ESP_ERR_TIMEOUT;
    }

    out_fix->has_fix = nmea.valid;
    out_fix->lat = nmea.lat;
    out_fix->lon = nmea.lon;
    out_fix->hdop = nmea.hdop;
    out_fix->satellites = nmea.satellites;
    out_fix->timestamp_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Fix: lat=%.6f lon=%.6f hdop=%.1f sat=%d",
             nmea.lat, nmea.lon, nmea.hdop, nmea.satellites);
    return ESP_OK;
}
