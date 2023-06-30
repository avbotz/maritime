#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.actuator.Command.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_actuator_Command_encode(struct uavcan_equipment_actuator_Command* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_MAX_SIZE);
    _uavcan_equipment_actuator_Command_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_actuator_Command_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_actuator_Command* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_actuator_Command_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_actuator_Command sample_uavcan_equipment_actuator_Command_msg(void) {
    struct uavcan_equipment_actuator_Command msg;

    msg.actuator_id = (uint8_t)random_bitlen_unsigned_val(8);
    msg.command_type = (uint8_t)random_bitlen_unsigned_val(8);
    msg.command_value = random_float16_val();
    return msg;
}
#endif