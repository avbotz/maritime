#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_MAX_SIZE 1
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE (0x8700F375556A8003ULL)
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID 1100

#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_DISARMED 0
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED 255

struct uavcan_equipment_safety_ArmingStatus {
    uint8_t status;
};

uint32_t uavcan_equipment_safety_ArmingStatus_encode(struct uavcan_equipment_safety_ArmingStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_safety_ArmingStatus* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_safety_ArmingStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao);
static inline void _uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao);
void _uavcan_equipment_safety_ArmingStatus_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->status);
    *bit_ofs += 8;
}

void _uavcan_equipment_safety_ArmingStatus_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_safety_ArmingStatus* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->status);
    *bit_ofs += 8;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_safety_ArmingStatus sample_uavcan_equipment_safety_ArmingStatus_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
