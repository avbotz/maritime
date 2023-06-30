#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_MAX_SIZE 0
#define UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE (0xEE468A8121C46A9EULL)
#define UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID 1

struct uavcan_protocol_GetNodeInfoRequest {
};

uint32_t uavcan_protocol_GetNodeInfoRequest_encode(struct uavcan_protocol_GetNodeInfoRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_GetNodeInfoRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_GetNodeInfoRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_GetNodeInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoRequest* msg, bool tao);
static inline void _uavcan_protocol_GetNodeInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoRequest* msg, bool tao);
void _uavcan_protocol_GetNodeInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

void _uavcan_protocol_GetNodeInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_GetNodeInfoRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_GetNodeInfoRequest sample_uavcan_protocol_GetNodeInfoRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
