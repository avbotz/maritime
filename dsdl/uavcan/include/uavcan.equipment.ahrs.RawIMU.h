#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.Timestamp.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE 120
#define UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE (0x8280632C40E574B5ULL)
#define UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID 1003

struct uavcan_equipment_ahrs_RawIMU {
    struct uavcan_Timestamp timestamp;
    float integration_interval;
    float rate_gyro_latest[3];
    float rate_gyro_integral[3];
    float accelerometer_latest[3];
    float accelerometer_integral[3];
    struct { uint8_t len; float data[36]; }covariance;
};

uint32_t uavcan_equipment_ahrs_RawIMU_encode(struct uavcan_equipment_ahrs_RawIMU* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_ahrs_RawIMU_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_ahrs_RawIMU* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_ahrs_RawIMU_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_RawIMU* msg, bool tao);
static inline void _uavcan_equipment_ahrs_RawIMU_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_RawIMU* msg, bool tao);
void _uavcan_equipment_ahrs_RawIMU_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_RawIMU* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_encode(buffer, bit_ofs, &msg->timestamp, false);
    canardEncodeScalar(buffer, *bit_ofs, 32, &msg->integration_interval);
    *bit_ofs += 32;
    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->rate_gyro_latest[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    for (size_t i=0; i < 3; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->rate_gyro_integral[i]);
        *bit_ofs += 32;
    }
    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->accelerometer_latest[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    for (size_t i=0; i < 3; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 32, &msg->accelerometer_integral[i]);
        *bit_ofs += 32;
    }
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 6, &msg->covariance.len);
        *bit_ofs += 6;
    }
    for (size_t i=0; i < msg->covariance.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->covariance.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
}

void _uavcan_equipment_ahrs_RawIMU_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_RawIMU* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_decode(transfer, bit_ofs, &msg->timestamp, false);

    canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->integration_interval);
    *bit_ofs += 32;

    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->rate_gyro_latest[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    for (size_t i=0; i < 3; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->rate_gyro_integral[i]);
        *bit_ofs += 32;
    }

    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->accelerometer_latest[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    for (size_t i=0; i < 3; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 32, true, &msg->accelerometer_integral[i]);
        *bit_ofs += 32;
    }

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 6, false, &msg->covariance.len);
        *bit_ofs += 6;
    } else {
        msg->covariance.len = ((transfer->payload_len*8)-*bit_ofs)/16;
    }

    for (size_t i=0; i < msg->covariance.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->covariance.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_ahrs_RawIMU sample_uavcan_equipment_ahrs_RawIMU_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
