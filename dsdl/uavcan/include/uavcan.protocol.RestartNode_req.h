#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAX_SIZE 5
#define UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_SIGNATURE (0x569E05394A3017F0ULL)
#define UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_ID 5

#define UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER 742196058910

struct uavcan_protocol_RestartNodeRequest {
    uint64_t magic_number;
};

uint32_t uavcan_protocol_RestartNodeRequest_encode(struct uavcan_protocol_RestartNodeRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_RestartNodeRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_RestartNodeRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_RestartNodeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_RestartNodeRequest* msg, bool tao);
static inline void _uavcan_protocol_RestartNodeRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_RestartNodeRequest* msg, bool tao);
void _uavcan_protocol_RestartNodeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_RestartNodeRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 40, &msg->magic_number);
    *bit_ofs += 40;
}

void _uavcan_protocol_RestartNodeRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_RestartNodeRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 40, false, &msg->magic_number);
    *bit_ofs += 40;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_RestartNodeRequest sample_uavcan_protocol_RestartNodeRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
