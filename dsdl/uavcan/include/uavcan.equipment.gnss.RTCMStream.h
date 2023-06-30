#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_MAX_SIZE 130
#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_SIGNATURE (0x1F56030ECB171501ULL)
#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID 1062

#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_PROTOCOL_ID_UNKNOWN 0
#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_PROTOCOL_ID_RTCM2 2
#define UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_PROTOCOL_ID_RTCM3 3

struct uavcan_equipment_gnss_RTCMStream {
    uint8_t protocol_id;
    struct { uint8_t len; uint8_t data[128]; }data;
};

uint32_t uavcan_equipment_gnss_RTCMStream_encode(struct uavcan_equipment_gnss_RTCMStream* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_gnss_RTCMStream_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_gnss_RTCMStream* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_gnss_RTCMStream_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_RTCMStream* msg, bool tao);
static inline void _uavcan_equipment_gnss_RTCMStream_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_RTCMStream* msg, bool tao);
void _uavcan_equipment_gnss_RTCMStream_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_RTCMStream* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->protocol_id);
    *bit_ofs += 8;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.len);
        *bit_ofs += 8;
    }
    for (size_t i=0; i < msg->data.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 8, &msg->data.data[i]);
        *bit_ofs += 8;
    }
}

void _uavcan_equipment_gnss_RTCMStream_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_RTCMStream* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->protocol_id);
    *bit_ofs += 8;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->data.len);
        *bit_ofs += 8;
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
struct uavcan_equipment_gnss_RTCMStream sample_uavcan_equipment_gnss_RTCMStream_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
