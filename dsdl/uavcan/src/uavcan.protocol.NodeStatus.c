#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.NodeStatus.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_NodeStatus_encode(struct uavcan_protocol_NodeStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);
    _uavcan_protocol_NodeStatus_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_NodeStatus_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_NodeStatus* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_NodeStatus_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_NodeStatus sample_uavcan_protocol_NodeStatus_msg(void) {
    struct uavcan_protocol_NodeStatus msg;

    msg.uptime_sec = (uint32_t)random_bitlen_unsigned_val(32);
    msg.health = (uint8_t)random_bitlen_unsigned_val(2);
    msg.mode = (uint8_t)random_bitlen_unsigned_val(3);
    msg.sub_mode = (uint8_t)random_bitlen_unsigned_val(3);
    msg.vendor_specific_status_code = (uint16_t)random_bitlen_unsigned_val(16);
    return msg;
}
#endif