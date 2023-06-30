#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.protocol.DataTypeKind.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_MAX_SIZE 84
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_SIGNATURE (0x1B283338A7BED2D8ULL)
#define UAVCAN_PROTOCOL_GETDATATYPEINFO_REQUEST_ID 2

struct uavcan_protocol_GetDataTypeInfoRequest {
    uint16_t id;
    struct uavcan_protocol_DataTypeKind kind;
    struct { uint8_t len; uint8_t data[80]; }name;
};

uint32_t uavcan_protocol_GetDataTypeInfoRequest_encode(struct uavcan_protocol_GetDataTypeInfoRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_GetDataTypeInfoRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_GetDataTypeInfoRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_GetDataTypeInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoRequest* msg, bool tao);
static inline void _uavcan_protocol_GetDataTypeInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoRequest* msg, bool tao);
void _uavcan_protocol_GetDataTypeInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->id);
    *bit_ofs += 16;
    _uavcan_protocol_DataTypeKind_encode(buffer, bit_ofs, &msg->kind, false);
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 7, &msg->name.len);
        *bit_ofs += 7;
    }
    for (size_t i=0; i < msg->name.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->name.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_GetDataTypeInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetDataTypeInfoRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->id);
    *bit_ofs += 16;

    _uavcan_protocol_DataTypeKind_decode(transfer, bit_ofs, &msg->kind, false);

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
struct uavcan_protocol_GetDataTypeInfoRequest sample_uavcan_protocol_GetDataTypeInfoRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
