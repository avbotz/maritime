#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.protocol.file.Error.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_MAX_SIZE 260
#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_SIGNATURE (0x8DCDCA939F33F678ULL)
#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_ID 48

struct uavcan_protocol_file_ReadResponse {
    struct uavcan_protocol_file_Error error;
    struct { uint16_t len; uint8_t data[256]; }data;
};

uint32_t uavcan_protocol_file_ReadResponse_encode(struct uavcan_protocol_file_ReadResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_file_ReadResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_file_ReadResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_file_ReadResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao);
static inline void _uavcan_protocol_file_ReadResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao);
void _uavcan_protocol_file_ReadResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_file_Error_encode(buffer, bit_ofs, &msg->error, false);
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 9, &msg->data.len);
        *bit_ofs += 9;
    }
    for (size_t i=0; i < msg->data.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_protocol_file_ReadResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_ReadResponse* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_file_Error_decode(transfer, bit_ofs, &msg->error, false);

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 9, false, &msg->data.len);
        *bit_ofs += 9;
    } else {
        msg->data.len = ((transfer->payload_len*8)-*bit_ofs)/8;
    }

    for (size_t i=0; i < msg->data.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.data[i]);
        *bit_ofs += 8;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_ReadResponse sample_uavcan_protocol_file_ReadResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
