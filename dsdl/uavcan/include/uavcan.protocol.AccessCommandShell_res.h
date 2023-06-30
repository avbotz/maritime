#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_MAX_SIZE 263
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_SIGNATURE (0x59276B5921C9246EULL)
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_ID 6

#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_FLAG_RUNNING 1
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_FLAG_SHELL_ERROR 2
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_FLAG_HAS_PENDING_STDOUT 64
#define UAVCAN_PROTOCOL_ACCESSCOMMANDSHELL_RESPONSE_FLAG_HAS_PENDING_STDERR 128

struct uavcan_protocol_AccessCommandShellResponse {
    int32_t last_exit_status;
    uint8_t flags;
    struct { uint16_t len; uint8_t data[256]; }output;
};

uint32_t uavcan_protocol_AccessCommandShellResponse_encode(struct uavcan_protocol_AccessCommandShellResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_AccessCommandShellResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_AccessCommandShellResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_AccessCommandShellResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellResponse* msg, bool tao);
static inline void _uavcan_protocol_AccessCommandShellResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellResponse* msg, bool tao);
void _uavcan_protocol_AccessCommandShellResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->last_exit_status);
    *bit_ofs += 32;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->flags);
    *bit_ofs += 8;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 9, &msg->output.len);
        *bit_ofs += 9;
    }
    for (size_t i=0; i < msg->output.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->output.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_AccessCommandShellResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_AccessCommandShellResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->last_exit_status);
    *bit_ofs += 32;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->flags);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 9, false, &msg->output.len);
        *bit_ofs += 9;
    } else {
        msg->output.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->output.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->output.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_AccessCommandShellResponse sample_uavcan_protocol_AccessCommandShellResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
