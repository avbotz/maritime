#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_MAX_SIZE 7
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_SIGNATURE (0x821AE2F525F69F21ULL)
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_ID 390

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISCOVERY_BROADCASTING_PERIOD_MS 1000

struct uavcan_protocol_dynamic_node_id_server_Discovery {
    uint8_t configured_cluster_size;
    struct { uint8_t len; uint8_t data[5]; }known_nodes;
};

uint32_t uavcan_protocol_dynamic_node_id_server_Discovery_encode(struct uavcan_protocol_dynamic_node_id_server_Discovery* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_dynamic_node_id_server_Discovery_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_dynamic_node_id_server_Discovery* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_dynamic_node_id_server_Discovery_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_Discovery* msg, bool tao);
static inline void _uavcan_protocol_dynamic_node_id_server_Discovery_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_Discovery* msg, bool tao);
void _uavcan_protocol_dynamic_node_id_server_Discovery_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_Discovery* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->configured_cluster_size);
    *bit_ofs += 8;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 3, &msg->known_nodes.len);
        *bit_ofs += 3;
    }
    for (size_t i=0; i < msg->known_nodes.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->known_nodes.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_dynamic_node_id_server_Discovery_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_dynamic_node_id_server_Discovery* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->configured_cluster_size);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->known_nodes.len);
        *bit_ofs += 3;
    } else {
        msg->known_nodes.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->known_nodes.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->known_nodes.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_dynamic_node_id_server_Discovery sample_uavcan_protocol_dynamic_node_id_server_Discovery_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
