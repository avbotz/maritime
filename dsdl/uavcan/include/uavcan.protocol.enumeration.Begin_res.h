#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_MAX_SIZE 1
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_SIGNATURE (0x196AE06426A3B5D8ULL)
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ID 15

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_OK 0
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_INVALID_MODE 1
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_INVALID_PARAMETER 2
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_UNSUPPORTED 3
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_UNKNOWN 255

struct uavcan_protocol_enumeration_BeginResponse {
    uint8_t error;
};

uint32_t uavcan_protocol_enumeration_BeginResponse_encode(struct uavcan_protocol_enumeration_BeginResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_enumeration_BeginResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_enumeration_BeginResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_enumeration_BeginResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_BeginResponse* msg, bool tao);
static inline void _uavcan_protocol_enumeration_BeginResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_BeginResponse* msg, bool tao);
void _uavcan_protocol_enumeration_BeginResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_BeginResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->error);
    *bit_ofs += 8;
}

void _uavcan_protocol_enumeration_BeginResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_enumeration_BeginResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->error);
    *bit_ofs += 8;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_enumeration_BeginResponse sample_uavcan_protocol_enumeration_BeginResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
