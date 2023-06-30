#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.ice.FuelTankStatus.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_ice_FuelTankStatus_encode(struct uavcan_equipment_ice_FuelTankStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_MAX_SIZE);
    _uavcan_equipment_ice_FuelTankStatus_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_ice_FuelTankStatus_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_ice_FuelTankStatus* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_ice_FuelTankStatus_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_ice_FuelTankStatus sample_uavcan_equipment_ice_FuelTankStatus_msg(void) {
    struct uavcan_equipment_ice_FuelTankStatus msg;

    msg.available_fuel_volume_percent = (uint8_t)random_bitlen_unsigned_val(7);
    msg.available_fuel_volume_cm3 = random_float_val();
    msg.fuel_consumption_rate_cm3pm = random_float_val();
    msg.fuel_temperature = random_float16_val();
    msg.fuel_tank_id = (uint8_t)random_bitlen_unsigned_val(8);
    return msg;
}
#endif