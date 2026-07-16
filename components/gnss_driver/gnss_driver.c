#include "gnss_driver.h"
#include "nmea_parser.h"
#include "app_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gnss_driver";
#define GNSS_RX_BUF_SIZE 512

esp_err_t gnss_driver_init(void)
{
    /* GY-NEO7M mặc định 9600 baud, NMEA 0183. */
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

    ESP_LOGI(TAG, "GY-NEO7M UART init (baud=%d)", GNSS_UART_BAUD);
    return ESP_OK;
}

esp_err_t gnss_driver_power(bool on)
{
    /* GY-NEO7M không có chân EN riêng -> điều khiển qua load-switch ngoài.
     * Nếu prototype chưa có mạch cắt nguồn, hàm này có thể tạm no-op
     * và chỉ dừng đọc UART để tiết kiệm CPU (vẫn tốn dòng tĩnh của GPS). */
    gpio_set_level(GNSS_POWER_EN_GPIO, on ? 1 : 0);
    ESP_LOGI(TAG, "GNSS power %s", on ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t gnss_driver_read_fix(gnss_fix_t *out_fix, uint32_t timeout_ms)
{
    char line[GNSS_RX_BUF_SIZE];
    /* TODO:
     * 1. uart_read_bytes() đọc đến khi gặp '\n', build thành 1 dòng NMEA.
     * 2. Gọi nmea_parse_line() để parse.
     * 3. GY-NEO7M bắt tín hiệu cold-start có thể mất 30s-1 phút, và
     *    theo mô tả prototype: cần buffer GNSS_FIX_BUFFER_COUNT fix
     *    liên tiếp có HDOP tốt trước khi chấp nhận làm P0.
     */
    (void)line; (void)timeout_ms;
    out_fix->has_fix = false;
    out_fix->timestamp_us = esp_timer_get_time();
    return ESP_ERR_TIMEOUT;
}
