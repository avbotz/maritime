#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_HARDPOINT_STATUS_MAX_SIZE 7
#define UAVCAN_EQUIPMENT_HARDPOINT_STATUS_SIGNATURE (0x624A519D42553D82ULL)
#define UAVCAN_EQUIPMENT_HARDPOINT_STATUS_ID 1071

struct uavcan_equipment_hardpoint_Status {
    uint8_t hardpoint_id;
    float payload_weight;
    float payload_weight_variance;
    uint16_t status;
};

uint32_t uavcan_equipment_hardpoint_Status_encode(struct uavcan_equipment_hardpoint_Status* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_hardpoint_Status_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_hardpoint_Status* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_hardpoint_Status_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_hardpoint_Status* msg, bool tao);
static inline void _uavcan_equipment_hardpoint_Status_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_hardpoint_Status* msg, bool tao);
void _uavcan_equipment_hardpoint_Status_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_hardpoint_Status* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->hardpoint_id);
    *bit_ofs += 8;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->payload_weight);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->payload_weight_variance);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->status);
    *bit_ofs += 16;
}

void _uavcan_equipment_hardpoint_Status_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_hardpoint_Status* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->hardpoint_id);
    *bit_ofs += 8;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->payload_weight = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->payload_weight_variance = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->status);
    *bit_ofs += 16;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_hardpoint_Status sample_uavcan_equipment_hardpoint_Status_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
