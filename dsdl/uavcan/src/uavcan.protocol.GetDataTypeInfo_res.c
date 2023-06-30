#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.GetDataTypeInfo_res.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_GetDataTypeInfoResponse_encode(struct uavcan_protocol_GetDataTypeInfoResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_GETDATATYPEINFO_RESPONSE_MAX_SIZE);
    _uavcan_protocol_GetDataTypeInfoResponse_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_GetDataTypeInfoResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_GetDataTypeInfoResponse* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_GetDataTypeInfoResponse_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_GetDataTypeInfoResponse sample_uavcan_protocol_GetDataTypeInfoResponse_msg(void) {
    struct uavcan_protocol_GetDataTypeInfoResponse msg;

    msg.signature = (uint64_t)random_bitlen_unsigned_val(64);
    msg.id = (uint16_t)random_bitlen_unsigned_val(16);
    msg.kind = sample_uavcan_protocol_DataTypeKind_msg();
    msg.flags = (uint8_t)random_bitlen_unsigned_val(8);
    msg.name.len = (uint8_t)random_range_unsigned_val(0, 80);
    for (size_t i=0; i < msg.name.len; i++) {
        msg.name.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif