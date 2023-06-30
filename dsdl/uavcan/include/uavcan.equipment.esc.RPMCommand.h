#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_MAX_SIZE 46
#define UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_SIGNATURE (0xCE0F9F621CF7E70BULL)
#define UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID 1031

struct uavcan_equipment_esc_RPMCommand {
    struct { uint8_t len; int32_t data[20]; }rpm;
};

uint32_t uavcan_equipment_esc_RPMCommand_encode(struct uavcan_equipment_esc_RPMCommand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_esc_RPMCommand_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_esc_RPMCommand* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_esc_RPMCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_RPMCommand* msg, bool tao);
static inline void _uavcan_equipment_esc_RPMCommand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_RPMCommand* msg, bool tao);
void _uavcan_equipment_esc_RPMCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_RPMCommand* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 5, &msg->rpm.len);
        *bit_ofs += 5;
    }
    for (size_t i=0; i < msg->rpm.len; i++) {
        canardEncodeScalar(buffer, *bit_ofs, 18, &msg->rpm.data[i]);
        *bit_ofs += 18;
    }
}

void _uavcan_equipment_esc_RPMCommand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_RPMCommand* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->rpm.len);
        *bit_ofs += 5;
    } else {
        msg->rpm.len = ((transfer->payload_len*8)-*bit_ofs)/18;
    }

    for (size_t i=0; i < msg->rpm.len; i++) {
        canardDecodeScalar(transfer, *bit_ofs, 18, true, &msg->rpm.data[i]);
        *bit_ofs += 18;
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_esc_RPMCommand sample_uavcan_equipment_esc_RPMCommand_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
