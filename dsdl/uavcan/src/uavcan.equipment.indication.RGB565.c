#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.indication.RGB565.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_indication_RGB565_encode(struct uavcan_equipment_indication_RGB565* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_INDICATION_RGB565_MAX_SIZE);
    _uavcan_equipment_indication_RGB565_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_equipment_indication_RGB565_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_indication_RGB565* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_indication_RGB565_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_indication_RGB565 sample_uavcan_equipment_indication_RGB565_msg(void) {
    struct uavcan_equipment_indication_RGB565 msg;

    msg.red = (uint8_t)random_bitlen_unsigned_val(5);
    msg.green = (uint8_t)random_bitlen_unsigned_val(6);
    msg.blue = (uint8_t)random_bitlen_unsigned_val(5);
    return msg;
}
#endif