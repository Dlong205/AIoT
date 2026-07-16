#pragma once
#include "rescue_types.h"

typedef enum {
    PKT_TYPE_SOS = 0,
    PKT_TYPE_HEARTBEAT = 1,
    PKT_TYPE_ACK = 2,
    PKT_TYPE_REPEATER_FORWARD = 3,
} mesh_packet_type_t;

/* Đóng gói sos_packet_t thành buffer bytes để gửi qua SX1262.
 * out_buf phải có ít nhất sizeof(sos_packet_t) byte. Trả về số byte đã ghi. */
size_t mesh_protocol_pack(const sos_packet_t *pkt, uint8_t *out_buf, size_t buf_len);

/* Giải mã buffer nhận được từ LoRa thành sos_packet_t. */
bool mesh_protocol_unpack(const uint8_t *buf, size_t len, sos_packet_t *out_pkt);

/* Kiểm tra xem gói này có nên forward tiếp không (hop_count < hop_limit),
 * và tăng hop_count lên 1 nếu forward. Dùng ở các node đóng vai repeater. */
bool mesh_protocol_should_forward(sos_packet_t *pkt);
