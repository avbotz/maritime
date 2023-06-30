#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_MAX_SIZE 1
#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_SIGNATURE (0x6924572FBB2086E5ULL)

#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE 1
#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY 2
#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_SYMLINK 4
#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE 8
#define UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_WRITEABLE 16

struct uavcan_protocol_file_EntryType {
    uint8_t flags;
};

uint32_t uavcan_protocol_file_EntryType_encode(struct uavcan_protocol_file_EntryType* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_protocol_file_EntryType_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_file_EntryType* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_protocol_file_EntryType_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_EntryType* msg, bool tao);
static inline void _uavcan_protocol_file_EntryType_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_EntryType* msg, bool tao);
void _uavcan_protocol_file_EntryType_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_protocol_file_EntryType* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->flags);
    *bit_ofs += 8;
}

void _uavcan_protocol_file_EntryType_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_protocol_file_EntryType* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->flags);
    *bit_ofs += 8;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_EntryType sample_uavcan_protocol_file_EntryType_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
