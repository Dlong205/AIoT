#pragma once
#include "esp_err.h"
#include "mesh_protocol.h"

esp_err_t lora_driver_init(void);

/* Gửi 1 gói SOS/heartbeat vào mesh. Blocking đến khi truyền xong (~vài trăm ms
 * tùy Spreading Factor). */
esp_err_t lora_driver_send_sos(const sos_packet_t *pkt);

/* Non-blocking: đăng ký callback nhận gói từ node khác (để làm repeater). */
typedef void (*lora_rx_cb_t)(const sos_packet_t *pkt, int8_t rssi);
esp_err_t lora_driver_set_rx_callback(lora_rx_cb_t cb);
