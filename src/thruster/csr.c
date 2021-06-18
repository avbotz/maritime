/*
 * csr.c
 *
 * Implements serialization/deserialization of the VideoRay CSR
 * communication protocol used on the VideoRay M5 thruster
 *
 * More information about the protocol spec can be found here:
 * https://github.com/videoray/VRCommsProtocol_doc/raw/master/VR_CSR_Communication_Protocol.doc
 * Documentation about the VideoRay M5 memory map can be found here:
 * https://download.videoray.com/documentation/m5_thruster/html/csr_memory_map.html
 *
 * @author Kalyan Sriram <kalyan@coderkalyan.com>
 */

#include <string.h>
#include <stdbool.h>

#include <sys/crc.h>

#include "thruster.h"

/*
 * Packs the thruster command header per CSR protocol
 *
 * The header consists of 6 bytes (not 7 as listed in the doc):
 * 0 Start of packet byte 1
 * 1 Start of packet byte 2
 * 2 Network ID
 * 3 Flags
 * 4 CSR Address
 * 5 Length
 *
 * 4 byte Header checksum
 */
void csr_header_pack(uint8_t *buffer, struct csr_header_s *header)
{
	memset(buffer, 0, PAYLOAD_OFFSET);

	if (header->start_type == PACKET_START_TYPE_REQUEST) {
		buffer[HEADER_START_BYTE1_INDEX] = PACKET_START_TYPE_REQUEST_BYTE1;
		buffer[HEADER_START_BYTE2_INDEX] = PACKET_START_TYPE_REQUEST_BYTE2;
	} else {
		buffer[HEADER_START_BYTE1_INDEX] = PACKET_START_TYPE_RESPONSE_BYTE1;
		buffer[HEADER_START_BYTE2_INDEX] = PACKET_START_TYPE_RESPONSE_BYTE2;
	}

	buffer[HEADER_NETWORK_ID_INDEX] = header->network_id;
	buffer[HEADER_FLAGS_INDEX] = header->flags;
	buffer[HEADER_ADDRESS_INDEX] = header->csr_address;
	buffer[HEADER_PAYLOAD_LENGTH_INDEX] = header->payload_len;

	/* compute header crc32 checksum */
	/* ARM generally runs little-endian, and the thrusters want little-endian,
	 * so we *should* be good to just memcpy the power value into the buffer
	 *
	 * This has been tested to work, but something to keep in mind in the unlikely
	 * case that this code is run on a big-endian system
	 */
	uint32_t checksum = crc32_ieee(buffer, HEADER_PAYLOAD_LENGTH_INDEX + 1);
	memcpy(buffer + HEADER_CHECKSUM_INDEX, &checksum, 4);
}

/*
 * Unpacks the thruster command header per CSR protocol
 *
 * @return true if checksum passed
 */
bool csr_header_unpack(uint8_t *buffer, struct csr_header_s *header)
{
	if ((buffer[HEADER_START_BYTE1_INDEX] == PACKET_START_TYPE_REQUEST_BYTE1) &&
		(buffer[HEADER_START_BYTE2_INDEX] == PACKET_START_TYPE_REQUEST_BYTE2)) {
		header->start_type = PACKET_START_TYPE_REQUEST;
	} else if ((buffer[HEADER_START_BYTE1_INDEX] == PACKET_START_TYPE_RESPONSE_BYTE1) &&
		(buffer[HEADER_START_BYTE2_INDEX] == PACKET_START_TYPE_RESPONSE_BYTE2)) {
		header->start_type = PACKET_START_TYPE_RESPONSE;
	} else {
		header->start_type = PACKET_START_TYPE_UNKNOWN;
	}

	header->network_id = buffer[HEADER_NETWORK_ID_INDEX];
	header->flags = buffer[HEADER_FLAGS_INDEX];
	header->csr_address = buffer[HEADER_ADDRESS_INDEX];
	header->payload_len = buffer[HEADER_PAYLOAD_LENGTH_INDEX];

	/* verify header crc32 checksum */
	uint32_t checksum = crc32_ieee(buffer, HEADER_PAYLOAD_LENGTH_INDEX + 1);
	if (memcmp(&checksum, buffer + HEADER_CHECKSUM_INDEX, sizeof(checksum)) == 0) {
		return true;
	} else {
		return false;
	}
}

void csr_payload_pack(uint8_t *buffer, uint8_t *payload, uint8_t payload_len)
{
	memset(buffer + PAYLOAD_OFFSET, 0, payload_len + CHECKSUM_LEN);
	memcpy(buffer + PAYLOAD_OFFSET, payload, payload_len);

	/* compute payload crc32 checksum */
	/* ARM generally runs little-endian, and the thrusters want little-endian,
	 * so we *should* be good to just memcpy the power value into the buffer
	 *
	 * This has been tested to work, but something to keep in mind in the unlikely
	 * case that this code is run on a big-endian system
	 */
	uint32_t checksum = crc32_ieee(payload, payload_len);
	memcpy(buffer + PAYLOAD_OFFSET + payload_len, &checksum, sizeof(checksum));
}

bool csr_payload_unpack(uint8_t *buffer, uint8_t *payload, uint8_t payload_len)
{
	memcpy(payload, buffer + PAYLOAD_OFFSET, payload_len);

	/* verify payload crc32 checksum */
	uint32_t checksum = crc32_ieee(payload, payload_len);
	if (memcmp(&checksum, buffer + PAYLOAD_OFFSET + payload_len, sizeof(checksum)) == 0) {
		return true;
	} else {
		return false;
	}
}

/*
 * Packs the propulsion command for driving 8 thrusters (for simplicity)
 *
 * To send data to specific thrusters, omit thrust values as needed
 * in csr_command_propulsion_s
 */
void csr_command_propulsion_pack(uint8_t *buffer, struct csr_command_propulsion_s *command)
{
	/* note: we don't want data back from the thruster. Instead, this is used
	 * on a publish-only basis, and telemetry from each thruster is queried separately
	 * at a fixed interval */

	/* first create the payload */
	uint8_t payload[COMMAND_PROPULSION_PAYLOAD_LEN];

	memset(payload, 0, COMMAND_PROPULSION_PAYLOAD_LEN);

	payload[COMMAND_PROPULSION_COMMAND_INDEX] = COMMAND_PROPULSION;
	payload[COMMAND_PROPULSION_NODE_ID_INDEX] = 
		command->enable_response ? command->response_node_id : 0;

	int offset = COMMAND_PROPULSION_THRUST_OFFSET;
	for (int i = 0;i < NUM_THRUSTERS;i++) {
		/* ARM generally runs little-endian, and the thrusters want little-endian,
		 * so we *should* be good to just memcpy the power value into the buffer
		 *
		 * This has been tested to work, but something to keep in mind in the unlikely
		 * case that this code is run on a big-endian system
		 */
		memcpy(payload + offset, &(command->power[i]), sizeof(float));
		offset += sizeof(float);
	}

	/* now pack the entire message */
	struct csr_header_s header = {
		.start_type = PACKET_START_TYPE_REQUEST,
		.network_id = THRUSTER_GROUP_ID,
		.flags = command->enable_response ? FLAG_RESPONSE_STANDARD : FLAG_RESPONSE_NONE,
		.csr_address = CSR_ADDRESS_CUSTOM_COMMAND,
		.payload_len = COMMAND_PROPULSION_PAYLOAD_DATA_LEN,
	};

	csr_header_pack(buffer, &header);
	csr_payload_pack(buffer, payload, COMMAND_PROPULSION_PAYLOAD_DATA_LEN);
}
