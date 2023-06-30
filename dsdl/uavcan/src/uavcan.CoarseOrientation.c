#define CANARD_DSDLC_INTERNAL
#include <uavcan.CoarseOrientation.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_CoarseOrientation_encode(struct uavcan_CoarseOrientation* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_COARSEORIENTATION_MAX_SIZE);
    _uavcan_CoarseOrientation_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_CoarseOrientation_decode(const CanardRxTransfer* transfer, struct uavcan_CoarseOrientation* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_CoarseOrientation_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_CoarseOrientation sample_uavcan_CoarseOrientation_msg(void) {
    struct uavcan_CoarseOrientation msg;

    for (size_t i=0; i < 3; i++) {
        msg.fixed_axis_roll_pitch_yaw[i] = (int8_t)random_bitlen_signed_val(5);
    }
    msg.orientation_defined = (bool)random_bitlen_unsigned_val(1);
    return msg;
}
#endif