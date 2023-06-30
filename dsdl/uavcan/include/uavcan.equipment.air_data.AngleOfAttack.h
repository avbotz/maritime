#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_AIR_DATA_ANGLEOFATTACK_MAX_SIZE 5
#define UAVCAN_EQUIPMENT_AIR_DATA_ANGLEOFATTACK_SIGNATURE (0xD5513C3F7AFAC74EULL)
#define UAVCAN_EQUIPMENT_AIR_DATA_ANGLEOFATTACK_ID 1025

#define UAVCAN_EQUIPMENT_AIR_DATA_ANGLEOFATTACK_SENSOR_ID_LEFT 254
#define UAVCAN_EQUIPMENT_AIR_DATA_ANGLEOFATTACK_SENSOR_ID_RIGHT 255

struct uavcan_equipment_air_data_AngleOfAttack {
    uint8_t sensor_id;
    float aoa;
    float aoa_variance;
};

uint32_t uavcan_equipment_air_data_AngleOfAttack_encode(struct uavcan_equipment_air_data_AngleOfAttack* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_air_data_AngleOfAttack_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_air_data_AngleOfAttack* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_air_data_AngleOfAttack_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_air_data_AngleOfAttack* msg, bool tao);
static inline void _uavcan_equipment_air_data_AngleOfAttack_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_air_data_AngleOfAttack* msg, bool tao);
void _uavcan_equipment_air_data_AngleOfAttack_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_air_data_AngleOfAttack* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sensor_id);
    *bit_ofs += 8;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->aoa);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->aoa_variance);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

void _uavcan_equipment_air_data_AngleOfAttack_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_air_data_AngleOfAttack* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sensor_id);
    *bit_ofs += 8;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->aoa = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->aoa_variance = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_air_data_AngleOfAttack sample_uavcan_equipment_air_data_AngleOfAttack_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
