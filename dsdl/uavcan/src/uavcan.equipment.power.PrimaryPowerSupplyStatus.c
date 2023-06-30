#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.power.PrimaryPowerSupplyStatus.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_power_PrimaryPowerSupplyStatus_encode(struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_POWER_PRIMARYPOWERSUPPLYSTATUS_MAX_SIZE);
    _uavcan_equipment_power_PrimaryPowerSupplyStatus_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_power_PrimaryPowerSupplyStatus_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_power_PrimaryPowerSupplyStatus* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_power_PrimaryPowerSupplyStatus_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_power_PrimaryPowerSupplyStatus sample_uavcan_equipment_power_PrimaryPowerSupplyStatus_msg(void) {
    struct uavcan_equipment_power_PrimaryPowerSupplyStatus msg;

    msg.hours_to_empty_at_10sec_avg_power = random_float16_val();
    msg.hours_to_empty_at_10sec_avg_power_variance = random_float16_val();
    msg.external_power_available = (bool)random_bitlen_unsigned_val(1);
    msg.remaining_energy_pct = (uint8_t)random_bitlen_unsigned_val(7);
    msg.remaining_energy_pct_stdev = (uint8_t)random_bitlen_unsigned_val(7);
    return msg;
}
#endif