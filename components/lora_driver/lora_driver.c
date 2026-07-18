#include "lora_driver.h"
#include "app_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lora_driver";
static lora_rx_cb_t s_rx_cb = NULL;
#define AS32_RX_BUF_SIZE 512

static void as32_set_mode_normal(void)
{
    gpio_set_level(AS32_M0_GPIO, 0);
    gpio_set_level(AS32_M1_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t lora_driver_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << AS32_M0_GPIO) | (1ULL << AS32_M1_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    uart_config_t cfg = {
        .baud_rate = AS32_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(AS32_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(AS32_UART_PORT, AS32_UART_TX_GPIO, AS32_UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(AS32_UART_PORT, AS32_RX_BUF_SIZE * 2, 0, 0, NULL, 0));

    as32_set_mode_normal();

    ESP_LOGI(TAG, "AS32 init OK (UART%d, baud=%d, normal mode)", AS32_UART_PORT, AS32_UART_BAUD);
    return ESP_OK;
}

esp_err_t lora_driver_send_sos(const sos_packet_t *pkt)
{
    uint8_t buf[sizeof(sos_packet_t)];
    size_t n = mesh_protocol_pack(pkt, buf, sizeof(buf));
    if (n == 0) return ESP_ERR_INVALID_ARG;

    int written = uart_write_bytes(AS32_UART_PORT, (const char *)buf, n);
    if (written != (int)n) {
        ESP_LOGW(TAG, "TX chua gui du: %d/%d bytes", written, (int)n);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TX SOS packet, %d bytes, hop_limit=%d", (int)n, pkt->hop_limit);
    return ESP_OK;
}

esp_err_t lora_driver_set_rx_callback(lora_rx_cb_t cb)
{
    s_rx_cb = cb;
    return ESP_OK;
}
