#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.Timestamp.h>
#include <uavcan.equipment.gnss.ECEFPositionVelocity.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE 222
#define UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE (0xCA41E7000F37435FULL)
#define UAVCAN_EQUIPMENT_GNSS_FIX2_ID 1063

#define UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE 0
#define UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_TAI 1
#define UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC 2
#define UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_GPS 3
#define UAVCAN_EQUIPMENT_GNSS_FIX2_NUM_LEAP_SECONDS_UNKNOWN 0
#define UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX 0
#define UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY 1
#define UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX 2
#define UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX 3
#define UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE 0
#define UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS 1
#define UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK 2
#define UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_PPP 3
#define UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER 0
#define UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS 1
#define UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT 0
#define UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED 1

struct uavcan_equipment_gnss_Fix2 {
    struct uavcan_Timestamp timestamp;
    struct uavcan_Timestamp gnss_timestamp;
    uint8_t gnss_time_standard;
    uint8_t num_leap_seconds;
    int64_t longitude_deg_1e8;
    int64_t latitude_deg_1e8;
    int32_t height_ellipsoid_mm;
    int32_t height_msl_mm;
    float ned_velocity[3];
    uint8_t sats_used;
    uint8_t status;
    uint8_t mode;
    uint8_t sub_mode;
    struct { uint8_t len; float data[36]; }covariance;
    float pdop;
    struct { uint8_t len; struct uavcan_equipment_gnss_ECEFPositionVelocity data[1]; }ecef_position_velocity;
};

uint32_t uavcan_equipment_gnss_Fix2_encode(struct uavcan_equipment_gnss_Fix2* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_gnss_Fix2_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_gnss_Fix2* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_gnss_Fix2_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_Fix2* msg, bool tao);
static inline void _uavcan_equipment_gnss_Fix2_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_Fix2* msg, bool tao);
void _uavcan_equipment_gnss_Fix2_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_Fix2* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_encode(buffer, bit_ofs, &msg->timestamp, false);
    _uavcan_Timestamp_encode(buffer, bit_ofs, &msg->gnss_timestamp, false);
    canardEncodeScalar(buffer, *bit_ofs, 3, &msg->gnss_time_standard);
    *bit_ofs += 3;
    *bit_ofs += 13;
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->num_leap_seconds);
    *bit_ofs += 8;
    canardEncodeScalar(buffer, *bit_ofs, 37, &msg->longitude_deg_1e8);
    *bit_ofs += 37;
    canardEncodeScalar(buffer, *bit_ofs, 37, &msg->latitude_deg_1e8);
    *bit_ofs += 37;
    canardEncodeScalar(buffer, *bit_ofs, 27, &msg->height_ellipsoid_mm);
    *bit_ofs += 27;
    canardEncodeScalar(buffer, *bit_ofs, 27, &msg->height_msl_mm);
    *bit_ofs += 27;
    for (size_t i=0; i < 3; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->ned_velocity[i]);
        *bit_ofs += 32;
    }
    canardEncodeScalar(buffer, *bit_ofs, 6, &msg->sats_used);
    *bit_ofs += 6;
    canardEncodeScalar(buffer, *bit_ofs, 2, &msg->status);
    *bit_ofs += 2;
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->mode);
    *bit_ofs += 4;
    canardEncodeScalar(buffer, *bit_ofs, 6, &msg->sub_mode);
    *bit_ofs += 6;
    canardEncodeScalar(buffer, *bit_ofs, 6, &msg->covariance.len);
    *bit_ofs += 6;
    for (size_t i=0; i < msg->covariance.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->covariance.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->pdop);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 1, &msg->ecef_position_velocity.len);
        *bit_ofs += 1;
    }
    for (size_t i=0; i < msg->ecef_position_velocity.len; i++) {
        _uavcan_equipment_gnss_ECEFPositionVelocity_encode(buffer, bit_ofs, &msg->ecef_position_velocity.data[i], false);
    }
}

void _uavcan_equipment_gnss_Fix2_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_gnss_Fix2* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_decode(transfer, bit_ofs, &msg->timestamp, false);

    _uavcan_Timestamp_decode(transfer, bit_ofs, &msg->gnss_timestamp, false);

    canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->gnss_time_standard);
    *bit_ofs += 3;

    *bit_ofs += 13;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->num_leap_seconds);
    *bit_ofs += 8;

    canardDecodeScalar(transfer, *bit_ofs, 37, true, &msg->longitude_deg_1e8);
    *bit_ofs += 37;

    canardDecodeScalar(transfer, *bit_ofs, 37, true, &msg->latitude_deg_1e8);
    *bit_ofs += 37;

    canardDecodeScalar(transfer, *bit_ofs, 27, true, &msg->height_ellipsoid_mm);
    *bit_ofs += 27;

    canardDecodeScalar(transfer, *bit_ofs, 27, true, &msg->height_msl_mm);
    *bit_ofs += 27;

    for (size_t i=0; i < 3; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->ned_velocity[i]);
        *bit_ofs += 32;
    }

    canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->sats_used);
    *bit_ofs += 6;

    canardDecodeScalar(transfer, *bit_ofs, 2, false, &msg->status);
    *bit_ofs += 2;

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->mode);
    *bit_ofs += 4;

    canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->sub_mode);
    *bit_ofs += 6;

    canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->covariance.len);
    *bit_ofs += 6;
    for (size_t i=0; i < msg->covariance.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->covariance.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->pdop = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->ecef_position_velocity.len);
        *bit_ofs += 1;
    }


    if (tao) {
        msg->ecef_position_velocity.len = 0;
        while ((transfer->payload_len*8) > *bit_ofs) {
            _uavcan_equipment_gnss_ECEFPositionVelocity_decode(transfer, bit_ofs, &msg->ecef_position_velocity.data[msg->ecef_position_velocity.len], false);
            msg->ecef_position_velocity.len++;
        }
    } else {
        for (size_t i=0; i < msg->ecef_position_velocity.len; i++) {
            _uavcan_equipment_gnss_ECEFPositionVelocity_decode(transfer, bit_ofs, &msg->ecef_position_velocity.data[i], false);
        }
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_gnss_Fix2 sample_uavcan_equipment_gnss_Fix2_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
