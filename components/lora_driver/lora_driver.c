#include "lora_driver.h"
#include "app_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "lora_driver";
static lora_rx_cb_t s_rx_cb = NULL;

esp_err_t lora_driver_init(void)
{
    /* TODO:
     * 1. spi_bus_initialize() trên LORA_SPI_HOST với SCK/MOSI/MISO.
     * 2. spi_bus_add_device() cho SX1262 (CS = LORA_CS_GPIO).
     * 3. Reset chip qua LORA_RST_GPIO, đợi BUSY (LORA_BUSY_GPIO) xuống thấp.
     * 4. Set packet type LoRa, tần số (vd 433MHz/868MHz/915MHz theo vùng),
     *    SF/BW/CR, công suất phát, cấu hình DIO1 báo TX/RX done.
     * 5. Gắn ISR trên LORA_DIO1_GPIO -> queue xử lý trong lora_task.
     */
    ESP_LOGI(TAG, "SX1262 init");
    return ESP_OK;
}

esp_err_t lora_driver_send_sos(const sos_packet_t *pkt)
{
    uint8_t buf[sizeof(sos_packet_t)];
    size_t n = mesh_protocol_pack(pkt, buf, sizeof(buf));
    if (n == 0) return ESP_ERR_INVALID_ARG;

    /* TODO: ghi buf vào FIFO của SX1262, set chế độ TX, đợi DIO1 báo TxDone
     * (hoặc timeout). Gói SOS nên gửi lặp lại vài lần (retry) vì mesh không
     * đảm bảo delivery. */
    ESP_LOGI(TAG, "TX SOS packet, %d bytes, hop_limit=%d", (int)n, pkt->hop_limit);
    return ESP_OK;
}

esp_err_t lora_driver_set_rx_callback(lora_rx_cb_t cb)
{
    s_rx_cb = cb;
    return ESP_OK;
}
