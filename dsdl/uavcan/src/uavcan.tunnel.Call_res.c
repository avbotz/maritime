#define CANARD_DSDLC_INTERNAL
#include <uavcan.tunnel.Call_res.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_tunnel_CallResponse_encode(struct uavcan_tunnel_CallResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_TUNNEL_CALL_RESPONSE_MAX_SIZE);
    _uavcan_tunnel_CallResponse_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_tunnel_CallResponse_decode(const CanardRxTransfer* transfer, struct uavcan_tunnel_CallResponse* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_tunnel_CallResponse_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_tunnel_CallResponse sample_uavcan_tunnel_CallResponse_msg(void) {
    struct uavcan_tunnel_CallResponse msg;

    msg.buffer.len = (uint8_t)random_range_unsigned_val(0, 60);
    for (size_t i=0; i < msg.buffer.len; i++) {
        msg.buffer.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif