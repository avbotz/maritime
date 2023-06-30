#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.protocol.file.Path.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_FILE_GETINFO_REQUEST_MAX_SIZE 201
#define UAVCAN_PROTOCOL_FILE_GETINFO_REQUEST_SIGNATURE (0x5004891EE8A27531ULL)
#define UAVCAN_PROTOCOL_FILE_GETINFO_REQUEST_ID 45

struct uavcan_protocol_file_GetInfoRequest {
    struct uavcan_protocol_file_Path path;
};

uint32_t uavcan_protocol_file_GetInfoRequest_encode(struct uavcan_protocol_file_GetInfoRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_file_GetInfoRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_file_GetInfoRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_file_GetInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoRequest* msg, bool tao);
static inline void _uavcan_protocol_file_GetInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoRequest* msg, bool tao);
void _uavcan_protocol_file_GetInfoRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_file_Path_encode(buffer, bit_ofs, &msg->path, tao);
}

void _uavcan_protocol_file_GetInfoRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_GetInfoRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_protocol_file_Path_decode(transfer, bit_ofs, &msg->path, tao);

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_GetInfoRequest sample_uavcan_protocol_file_GetInfoRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
