#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.protocol.param.Value.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE 224
#define UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_SIGNATURE (0xA7B622F939D1A4D5ULL)
#define UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID 11

struct uavcan_protocol_param_GetSetRequest {
    uint16_t index;
    struct uavcan_protocol_param_Value value;
    struct { uint8_t len; uint8_t data[92]; }name;
};

uint32_t uavcan_protocol_param_GetSetRequest_encode(struct uavcan_protocol_param_GetSetRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_param_GetSetRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_GetSetRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_param_GetSetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetRequest* msg, bool tao);
static inline void _uavcan_protocol_param_GetSetRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetRequest* msg, bool tao);
void _uavcan_protocol_param_GetSetRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 13, &msg->index);
    *bit_ofs += 13;
    _uavcan_protocol_param_Value_encode(buffer, bit_ofs, &msg->value, false);
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 7, &msg->name.len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < msg->name.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_param_GetSetRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_param_GetSetRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 13, false, &msg->index);
    *bit_ofs += 13;

    _uavcan_protocol_param_Value_decode(transfer, bit_ofs, &msg->value, false);

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->name.len);
        *bit_ofs += 7;
    } else {
        msg->name.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->name.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->name.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_param_GetSetRequest sample_uavcan_protocol_param_GetSetRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
