#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <uavcan.equipment.camera_gimbal.Mode.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_CAMERA_GIMBAL_ANGULARCOMMAND_MAX_SIZE 10
#define UAVCAN_EQUIPMENT_CAMERA_GIMBAL_ANGULARCOMMAND_SIGNATURE (0x4AF6E57B2B2BE29CULL)
#define UAVCAN_EQUIPMENT_CAMERA_GIMBAL_ANGULARCOMMAND_ID 1040

struct uavcan_equipment_camera_gimbal_AngularCommand {
    uint8_t gimbal_id;
    struct uavcan_equipment_camera_gimbal_Mode mode;
    float quaternion_xyzw[4];
};

uint32_t uavcan_equipment_camera_gimbal_AngularCommand_encode(struct uavcan_equipment_camera_gimbal_AngularCommand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_camera_gimbal_AngularCommand_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_camera_gimbal_AngularCommand* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_camera_gimbal_AngularCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_camera_gimbal_AngularCommand* msg, bool tao);
static inline void _uavcan_equipment_camera_gimbal_AngularCommand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_camera_gimbal_AngularCommand* msg, bool tao);
void _uavcan_equipment_camera_gimbal_AngularCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_camera_gimbal_AngularCommand* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->gimbal_id);
    *bit_ofs += 8;
    _uavcan_equipment_camera_gimbal_Mode_encode(buffer, bit_ofs, &msg->mode, false);
    for (size_t i=0; i < 4; i++) {
        {
            uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->quaternion_xyzw[i]);
            canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
        }
        *bit_ofs += 16;
    }
}

void _uavcan_equipment_camera_gimbal_AngularCommand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_camera_gimbal_AngularCommand* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->gimbal_id);
    *bit_ofs += 8;

    _uavcan_equipment_camera_gimbal_Mode_decode(transfer, bit_ofs, &msg->mode, false);

    for (size_t i=0; i < 4; i++) {
        {
            uint16_t float16_val;
            canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
            msg->quaternion_xyzw[i] = canardConvertFloat16ToNativeFloat(float16_val);
        }
        *bit_ofs += 16;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_camera_gimbal_AngularCommand sample_uavcan_equipment_camera_gimbal_AngularCommand_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
