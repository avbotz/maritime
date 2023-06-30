#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_MAX_SIZE 4
#define UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_SIGNATURE (0xBE9EA9FEC2B15D52ULL)
#define UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND_ID 1080

struct uavcan_equipment_indication_BeepCommand {
    float frequency;
    float duration;
};

uint32_t uavcan_equipment_indication_BeepCommand_encode(struct uavcan_equipment_indication_BeepCommand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_indication_BeepCommand_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_indication_BeepCommand* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_indication_BeepCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_indication_BeepCommand* msg, bool tao);
static inline void _uavcan_equipment_indication_BeepCommand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_indication_BeepCommand* msg, bool tao);
void _uavcan_equipment_indication_BeepCommand_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_indication_BeepCommand* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->frequency);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->duration);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

void _uavcan_equipment_indication_BeepCommand_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_indication_BeepCommand* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->frequency = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->duration = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_indication_BeepCommand sample_uavcan_equipment_indication_BeepCommand_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
