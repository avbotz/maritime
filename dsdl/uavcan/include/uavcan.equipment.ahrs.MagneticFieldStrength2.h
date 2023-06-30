#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_MAX_SIZE 26
#define UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_SIGNATURE (0xB6AC0C442430297EULL)
#define UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID 1002

struct uavcan_equipment_ahrs_MagneticFieldStrength2 {
    uint8_t sensor_id;
    float magnetic_field_ga[3];
    struct { uint8_t len; float data[9]; }magnetic_field_covariance;
};

uint32_t uavcan_equipment_ahrs_MagneticFieldStrength2_encode(struct uavcan_equipment_ahrs_MagneticFieldStrength2* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_ahrs_MagneticFieldStrength2_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_ahrs_MagneticFieldStrength2* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_ahrs_MagneticFieldStrength2_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_MagneticFieldStrength2* msg, bool tao);
static inline void _uavcan_equipment_ahrs_MagneticFieldStrength2_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_MagneticFieldStrength2* msg, bool tao);
void _uavcan_equipment_ahrs_MagneticFieldStrength2_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_MagneticFieldStrength2* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sensor_id);
    *bit_ofs += 8;
    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->magnetic_field_ga[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 4, &msg->magnetic_field_covariance.len);
        *bit_ofs += 4;
    }
    for (size_t i=0; i < msg->magnetic_field_covariance.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->magnetic_field_covariance.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
}

void _uavcan_equipment_ahrs_MagneticFieldStrength2_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_MagneticFieldStrength2* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sensor_id);
    *bit_ofs += 8;

    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->magnetic_field_ga[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->magnetic_field_covariance.len);
        *bit_ofs += 4;
    } else {
        msg->magnetic_field_covariance.len = ((transfer->payload_len*8)-*bit_ofs)/16;
    }

    for (size_t i=0; i < msg->magnetic_field_covariance.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->magnetic_field_covariance.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_ahrs_MagneticFieldStrength2 sample_uavcan_equipment_ahrs_MagneticFieldStrength2_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
