#ifndef _MARITIME_THRUSTER_H
#define _MARITIME_THRUSTER_H

#include <stdbool.h>
#include <stdint.h>

enum packet_start_type {
	PACKET_START_TYPE_REQUEST = 0,
	PACKET_START_TYPE_RESPONSE = 1,
	PACKET_START_TYPE_UNKNOWN = 2,
};

/* per CSR doc */
/* request packet synchronization header */
#define PACKET_START_TYPE_REQUEST_BYTE1 0xF5
#define PACKET_START_TYPE_REQUEST_BYTE2 0x5F

/* per CSR doc */
/* response packet synchronization header */
#define PACKET_START_TYPE_RESPONSE_BYTE1 0xF0
#define PACKET_START_TYPE_RESPONSE_BYTE2 0x0F

/* per CSR doc */
#define NODE_ID_BROADCAST 0xFF /* send to all nodes */

/* per M5 manual */
#define FLAG_RESPONSE_NONE 0x00 /* no thruster response */
#define FLAG_RESPONSE_STANDARD 0x02 /* standard thruster response */

/* Default per M5 manual. Thrusters came with this set to 0XFF,
 * but they should be set to 0x81 */
#define THRUSTER_GROUP_ID 0x81

/* per CSR doc */
#define HEADER_START_BYTE1_INDEX 0
#define HEADER_START_BYTE2_INDEX 1
#define HEADER_NETWORK_ID_INDEX 2
#define HEADER_FLAGS_INDEX 3
#define HEADER_ADDRESS_INDEX 4
#define HEADER_PAYLOAD_LENGTH_INDEX 5
#define HEADER_CHECKSUM_INDEX 6

#define CHECKSUM_LEN sizeof(uint32_t)

#define PAYLOAD_OFFSET (6 + CHECKSUM_LEN) /* data payload starts after 6 byte header + 4 byte checksum */

/* per M5 manual */
#define CSR_ADDRESS_CUSTOM_COMMAND 0xF0


/* Expected propulsion command size:
 * 06 byte header +
 * 04 byte header checksum +
 * 01 byte command ID +
 * 01 byte response node ID (generally unused) +
 * 32 byte propulsion command x8 (4 byte float each)
 * 04 byte payload checksum
 * ------------------------------------
 * 48 byte total size
 */
#define COMMAND_PROPULSION_PAYLOAD_DATA_LEN 34
#define COMMAND_PROPULSION_PAYLOAD_LEN (COMMAND_PROPULSION_PAYLOAD_DATA_LEN + CHECKSUM_LEN)
#define COMMAND_PROPULSION_LEN (PAYLOAD_OFFSET + COMMAND_PROPULSION_PAYLOAD_LEN)

#define COMMAND_PROPULSION_COMMAND_INDEX 0
#define COMMAND_PROPULSION_NODE_ID_INDEX 1
#define COMMAND_PROPULSION_THRUST_OFFSET 2

/* per M5 manual */
#define COMMAND_PROPULSION 0xAA

/* per M5 manual */
#define RESPONSE_THRUSTER_STANDARD 0x02

#define NUM_THRUSTERS 8

struct csr_header_s {
	enum packet_start_type start_type;
	uint8_t network_id;
	uint8_t flags;
	uint8_t csr_address;
	uint8_t payload_len;
};

struct csr_command_propulsion_s {
	bool enable_response;
	uint8_t response_node_id; /* only used if enable_response is true */
	float power[NUM_THRUSTERS]; /* propulsion power setpoint for each thruster, normalized [-1, 1] */
};

void csr_header_pack(uint8_t *buffer, struct csr_header_s *header);
bool csr_header_unpack(uint8_t *buffer, struct csr_header_s *header);
void csr_payload_pack(uint8_t *buffer, uint8_t *payload, uint8_t payload_len);
bool csr_payload_unpack(uint8_t *buffer, uint8_t *payload, uint8_t payload_len);

void csr_command_propulsion_pack(uint8_t *buffer, struct csr_command_propulsion_s *command);

#endif /* _MARITIME_THRUSTER_H */
