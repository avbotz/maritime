#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.CoarseOrientation.h>
#include <uavcan.Timestamp.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE 15
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE (0x68FFFE70FC771952ULL)
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID 1050

#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_UNDEFINED 0
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR 1
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR 2
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR 3
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED 0
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE 1
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE 2
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR 3

struct uavcan_equipment_range_sensor_Measurement {
    struct uavcan_Timestamp timestamp;
    uint8_t sensor_id;
    struct uavcan_CoarseOrientation beam_orientation_in_body_frame;
    float field_of_view;
    uint8_t sensor_type;
    uint8_t reading_type;
    float range;
};

uint32_t uavcan_equipment_range_sensor_Measurement_encode(struct uavcan_equipment_range_sensor_Measurement* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_range_sensor_Measurement_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_range_sensor_Measurement* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_range_sensor_Measurement_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_range_sensor_Measurement* msg, bool tao);
static inline void _uavcan_equipment_range_sensor_Measurement_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_range_sensor_Measurement* msg, bool tao);
void _uavcan_equipment_range_sensor_Measurement_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_range_sensor_Measurement* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_encode(buffer, bit_ofs, &msg->timestamp, false);
    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->sensor_id);
    *bit_ofs += 8;
    _uavcan_CoarseOrientation_encode(buffer, bit_ofs, &msg->beam_orientation_in_body_frame, false);
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->field_of_view);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 5, &msg->sensor_type);
    *bit_ofs += 5;
    canardEncodeScalar(buffer, *bit_ofs, 3, &msg->reading_type);
    *bit_ofs += 3;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->range);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

void _uavcan_equipment_range_sensor_Measurement_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_range_sensor_Measurement* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    _uavcan_Timestamp_decode(transfer, bit_ofs, &msg->timestamp, false);

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->sensor_id);
    *bit_ofs += 8;

    _uavcan_CoarseOrientation_decode(transfer, bit_ofs, &msg->beam_orientation_in_body_frame, false);

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->field_of_view = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->sensor_type);
    *bit_ofs += 5;

    canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->reading_type);
    *bit_ofs += 3;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->range = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_range_sensor_Measurement sample_uavcan_equipment_range_sensor_Measurement_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
