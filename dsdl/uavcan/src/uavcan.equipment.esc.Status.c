#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.esc.Status.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_esc_Status_encode(struct uavcan_equipment_esc_Status* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE);
    _uavcan_equipment_esc_Status_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_esc_Status_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_esc_Status* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_esc_Status_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_esc_Status sample_uavcan_equipment_esc_Status_msg(void) {
    struct uavcan_equipment_esc_Status msg;

    msg.error_count = (uint32_t)random_bitlen_unsigned_val(32);
    msg.voltage = random_float16_val();
    msg.current = random_float16_val();
    msg.temperature = random_float16_val();
    msg.rpm = (int32_t)random_bitlen_signed_val(18);
    msg.power_rating_pct = (uint8_t)random_bitlen_unsigned_val(7);
    msg.esc_index = (uint8_t)random_bitlen_unsigned_val(5);
    return msg;
}
#endif