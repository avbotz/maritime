#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.debug.LogMessage.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_debug_LogMessage_encode(struct uavcan_protocol_debug_LogMessage* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE);
    _uavcan_protocol_debug_LogMessage_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_debug_LogMessage_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_debug_LogMessage* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_debug_LogMessage_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_debug_LogMessage sample_uavcan_protocol_debug_LogMessage_msg(void) {
    struct uavcan_protocol_debug_LogMessage msg;

    msg.level = sample_uavcan_protocol_debug_LogLevel_msg();
    msg.source.len = (uint8_t)random_range_unsigned_val(0, 31);
    for (size_t i=0; i < msg.source.len; i++) {
        msg.source.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    msg.text.len = (uint8_t)random_range_unsigned_val(0, 90);
    for (size_t i=0; i < msg.text.len; i++) {
        msg.text.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif