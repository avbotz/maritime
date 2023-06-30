#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE 63
#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE (0xE02F25D6E0C98AE0ULL)
#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID 16370

struct uavcan_protocol_debug_KeyValue {
    float value;
    struct { uint8_t len; uint8_t data[58]; }key;
};

uint32_t uavcan_protocol_debug_KeyValue_encode(struct uavcan_protocol_debug_KeyValue* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_debug_KeyValue_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_debug_KeyValue* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_debug_KeyValue_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao);
static inline void _uavcan_protocol_debug_KeyValue_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao);
void _uavcan_protocol_debug_KeyValue_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->value);
    *bit_ofs += 32;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 6, &msg->key.len);
        *bit_ofs += 6;
    }
    for (size_t i=0; i < msg->key.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->key.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_debug_KeyValue_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_debug_KeyValue* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->value);
    *bit_ofs += 32;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->key.len);
        *bit_ofs += 6;
    } else {
        msg->key.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->key.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->key.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_debug_KeyValue sample_uavcan_protocol_debug_KeyValue_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
