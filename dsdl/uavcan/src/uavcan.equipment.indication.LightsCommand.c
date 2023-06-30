#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.indication.LightsCommand.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_indication_LightsCommand_encode(struct uavcan_equipment_indication_LightsCommand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_MAX_SIZE);
    _uavcan_equipment_indication_LightsCommand_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_indication_LightsCommand_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_indication_LightsCommand* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_indication_LightsCommand_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_indication_LightsCommand sample_uavcan_equipment_indication_LightsCommand_msg(void) {
    struct uavcan_equipment_indication_LightsCommand msg;

    msg.commands.len = (uint8_t)random_range_unsigned_val(0, 20);
    for (size_t i=0; i < msg.commands.len; i++) {
        msg.commands.data[i] = sample_uavcan_equipment_indication_SingleLightCommand_msg();
    }
    return msg;
}
#endif