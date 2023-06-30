#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.HardwareVersion.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_HardwareVersion_encode(struct uavcan_protocol_HardwareVersion* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_HARDWAREVERSION_MAX_SIZE);
    _uavcan_protocol_HardwareVersion_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_HardwareVersion_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_HardwareVersion* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_HardwareVersion_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_HardwareVersion sample_uavcan_protocol_HardwareVersion_msg(void) {
    struct uavcan_protocol_HardwareVersion msg;

    msg.major = (uint8_t)random_bitlen_unsigned_val(8);
    msg.minor = (uint8_t)random_bitlen_unsigned_val(8);
    for (size_t i=0; i < 16; i++) {
        msg.unique_id[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    msg.certificate_of_authenticity.len = (uint8_t)random_range_unsigned_val(0, 255);
    for (size_t i=0; i < msg.certificate_of_authenticity.len; i++) {
        msg.certificate_of_authenticity.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif