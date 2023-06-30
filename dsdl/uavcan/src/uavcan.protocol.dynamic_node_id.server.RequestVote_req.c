#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.dynamic_node_id.server.RequestVote_req.h>
#include <uavcan.protocol.dynamic_node_id.server.RequestVote_res.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_REQUESTVOTE_REQUEST_MAX_SIZE);
    _uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

bool uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    return (((bit_ofs+7)/8) != transfer->payload_len);
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest sample_uavcan_protocol_dynamic_node_id_server_RequestVoteRequest_msg(void) {
    struct uavcan_protocol_dynamic_node_id_server_RequestVoteRequest msg;

    msg.term = (uint32_t)random_bitlen_unsigned_val(32);
    msg.last_log_term = (uint32_t)random_bitlen_unsigned_val(32);
    msg.last_log_index = (uint8_t)random_bitlen_unsigned_val(8);
    return msg;
}
#endif