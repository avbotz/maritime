#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.file.BeginFirmwareUpdate_res.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(struct uavcan_protocol_file_BeginFirmwareUpdateResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE);
    _uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_file_BeginFirmwareUpdateResponse_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_file_BeginFirmwareUpdateResponse* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_file_BeginFirmwareUpdateResponse_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_BeginFirmwareUpdateResponse sample_uavcan_protocol_file_BeginFirmwareUpdateResponse_msg(void) {
    struct uavcan_protocol_file_BeginFirmwareUpdateResponse msg;

    msg.error = (uint8_t)random_bitlen_unsigned_val(8);
    msg.optional_error_message.len = (uint8_t)random_range_unsigned_val(0, 127);
    for (size_t i=0; i < msg.optional_error_message.len; i++) {
        msg.optional_error_message.data[i] = (uint8_t)random_bitlen_unsigned_val(8);
    }
    return msg;
}
#endif