#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.gnss.RTCMStream.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_gnss_RTCMStream_encode(struct uavcan_equipment_gnss_RTCMStream* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_MAX_SIZE);
    _uavcan_equipment_gnss_RTCMStream_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_gnss_RTCMStream_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_gnss_RTCMStream* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_gnss_RTCMStream_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_gnss_RTCMStream sample_uavcan_equipment_gnss_RTCMStream_msg(void) {
    struct uavcan_equipment_gnss_RTCMStream msg;

    msg.protocol_id = (uint8_t)random_bitlen_unsigned_val(8);
    msg.data.len = (uint8_t)random_range_unsigned_val(0, 128);
    for (size_t i=0; i < msg.data.len; i++) {
        msg.data.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif