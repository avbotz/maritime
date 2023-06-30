#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.GetTransportStats_res.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_GetTransportStatsResponse_encode(struct uavcan_protocol_GetTransportStatsResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_GETTRANSPORTSTATS_RESPONSE_MAX_SIZE);
    _uavcan_protocol_GetTransportStatsResponse_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_GetTransportStatsResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_GetTransportStatsResponse* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_GetTransportStatsResponse_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_GetTransportStatsResponse sample_uavcan_protocol_GetTransportStatsResponse_msg(void) {
    struct uavcan_protocol_GetTransportStatsResponse msg;

    msg.transfers_tx = (uint64_t)random_bitlen_unsigned_val(48);
    msg.transfers_rx = (uint64_t)random_bitlen_unsigned_val(48);
    msg.transfer_errors = (uint64_t)random_bitlen_unsigned_val(48);
    msg.can_iface_stats.len = (uint8_t)random_range_unsigned_val(0, 3);
    for (size_t i=0; i < msg.can_iface_stats.len; i++) {
        msg.can_iface_stats.data[i] = sample_uavcan_protocol_CANIfaceStats_msg();
    }
    return msg;
}
#endif