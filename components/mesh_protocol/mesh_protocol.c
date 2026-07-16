#include "mesh_protocol.h"
#include <string.h>

size_t mesh_protocol_pack(const sos_packet_t *pkt, uint8_t *out_buf, size_t buf_len)
{
    /* TODO: nếu cần payload nhỏ gọn hơn cho LoRa (airtime thấp), thay
     * struct binary trực tiếp bằng bit-packing/CBOR thay vì memcpy thô. */
    if (buf_len < sizeof(sos_packet_t)) return 0;
    memcpy(out_buf, pkt, sizeof(sos_packet_t));
    return sizeof(sos_packet_t);
}

bool mesh_protocol_unpack(const uint8_t *buf, size_t len, sos_packet_t *out_pkt)
{
    if (len < sizeof(sos_packet_t)) return false;
    memcpy(out_pkt, buf, sizeof(sos_packet_t));
    return true;
}

bool mesh_protocol_should_forward(sos_packet_t *pkt)
{
    if (pkt->hop_count >= pkt->hop_limit) return false;
    pkt->hop_count++;
    return true;
}
