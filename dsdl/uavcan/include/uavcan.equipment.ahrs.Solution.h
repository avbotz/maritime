#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.Timestamp.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE 84
#define UAVCAN_EQUIPMENT_AHRS_SOLUTION_SIGNATURE (0x72A63A3C6F41FA9BULL)
#define UAVCAN_EQUIPMENT_AHRS_SOLUTION_ID 1000

struct uavcan_equipment_ahrs_Solution {
    struct uavcan_Timestamp timestamp;
    float orientation_xyzw[4];
    struct { uint8_t len; float data[9]; }orientation_covariance;
    float angular_velocity[3];
    struct { uint8_t len; float data[9]; }angular_velocity_covariance;
    float linear_acceleration[3];
    struct { uint8_t len; float data[9]; }linear_acceleration_covariance;
};

uint32_t uavcan_equipment_ahrs_Solution_encode(struct uavcan_equipment_ahrs_Solution* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_ahrs_Solution_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_ahrs_Solution* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_ahrs_Solution_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_Solution* msg, bool tao);
static inline void _uavcan_equipment_ahrs_Solution_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_Solution* msg, bool tao);
void _uavcan_equipment_ahrs_Solution_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_Solution* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_encode(buffer, bit_ofs, &msg->timestamp, false);
    for (size_t i=0; i < 4; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->orientation_xyzw[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    *bit_ofs += 4;
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->orientation_covariance.len);
    *bit_ofs += 4;
    for (size_t i=0; i < msg->orientation_covariance.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->orientation_covariance.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->angular_velocity[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    *bit_ofs += 4;
    canardEncodeScalar(buffer, *bit_ofs, 4, &msg->angular_velocity_covariance.len);
    *bit_ofs += 4;
    for (size_t i=0; i < msg->angular_velocity_covariance.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->angular_velocity_covariance.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->linear_acceleration[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 4, &msg->linear_acceleration_covariance.len);
        *bit_ofs += 4;
    }
    for (size_t i=0; i < msg->linear_acceleration_covariance.len; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->linear_acceleration_covariance.data[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
}

void _uavcan_equipment_ahrs_Solution_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_ahrs_Solution* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_decode(transfer, bit_ofs, &msg->timestamp, false);

    for (size_t i=0; i < 4; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->orientation_xyzw[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    *bit_ofs += 4;

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->orientation_covariance.len);
    *bit_ofs += 4;
    for (size_t i=0; i < msg->orientation_covariance.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->orientation_covariance.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->angular_velocity[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    *bit_ofs += 4;

    canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->angular_velocity_covariance.len);
    *bit_ofs += 4;
    for (size_t i=0; i < msg->angular_velocity_covariance.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->angular_velocity_covariance.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    for (size_t i=0; i < 3; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->linear_acceleration[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 4, false, &msg->linear_acceleration_covariance.len);
        *bit_ofs += 4;
    } else {
        msg->linear_acceleration_covariance.len = ((transfer->payload_len*8)-*bit_ofs)/16;
    }

    for (size_t i=0; i < msg->linear_acceleration_covariance.len; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->linear_acceleration_covariance.data[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_ahrs_Solution sample_uavcan_equipment_ahrs_Solution_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
