#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include <canard.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <uavcan.equipment.esc.Status.h>

#include "thruster.h"

LOG_MODULE_REGISTER(uavcan, LOG_LEVEL_INF);

/*
 * TODO: 
 * Find out why unaligned access occurs when optimizing size in binary (C flag -Os)
 * Seems to be happening in canard.c https://github.com/dronecan/libcanard/blob/e060d18396d5fdd5e72aeaf723df5ea4219d6953/canard.c#L385
 * Fix for now is just to disable binary size optimization 
 */

//ZZ LOG_MODULE_REGISTER(uavcan, LOG_LEVEL_DBG);

const static struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

int ret;

// uint8_t buffer[256];


#define CANARD_SPIN_PERIOD_MS 1000
#define CANARD_MEM_POOL_SIZE 4096

// holds maximum 5 can frames; 17 bytes / frame * 10 frames = 170 bytes
#define CAN_RX_FRAME_BUF_SIZE (sizeof(struct can_frame) * 10)

static CanardInstance canard_ins;
static uint8_t canard_mem_pool[CANARD_MEM_POOL_SIZE];

K_MUTEX_DEFINE(canard_tx_queue_mutex);

K_SEM_DEFINE(can_rx_frame_buf_sem, 0, 1);
RING_BUF_DECLARE(can_rx_frame_buf, CAN_RX_FRAME_BUF_SIZE);


uint32_t get_uptime_sec_32() {
    return k_uptime_get_32() / 1000;
}

/*
void rx_irq_callback(const struct device *dev, struct can_frame *frame, void *user_data) {
    uint64_t data = 0;
    for (int i = 0; i < frame->dlc; ++i) {
        data |= frame->data[i];
        printk("data[i]: 0x%x, data: 0x%llx\n", frame->data[i], data);
        if (i != frame->dlc - 1) data <<= 8;
    }
    // for (int i = 0; i < frame->dlc; ++i) printk("%x ", frame->data[i]);
    LOG_DBG("Rx IRQ id: 0x%x dlc: %d, data: 0x%llx", frame->id, frame->dlc, data);
}
*/

uint32_t get_canard_id(const struct can_frame *can_frame) {
    uint32_t canard_id = can_frame->id;

    if (can_frame->flags & CAN_FRAME_IDE) {
        canard_id |= CANARD_CAN_FRAME_EFF;
    } 

    if (can_frame->flags & CAN_FRAME_RTR) {
        canard_id |= CANARD_CAN_FRAME_RTR;
    }

    return canard_id;
}

/*
 * Note on CAN RX interrupt:
 * This could have been alternatively been achieved by Zephyr's CAN_MSG_Q wrapper,
 * however k_msg_q_get() employs spin-locks that would waste significant CPU cycles for 
 * NodeStatus and other infrequent RX messages happening at the magnitude of around 
 * 1 Hz. Therefore, interrupts with semaphore signaling is a much more efficient use 
 * of CPU usage. 
 */
void can_rx_irq_callback(const struct device *dev, struct can_frame *can_frame, void *user_data) {
    // config rx irq timestamp
    // const uint64_t timestamp_usec = k_tick_to_us_floor64(k_uptime_ticks());
    uint8_t *dst;
    ring_buf_put_claim(&can_rx_frame_buf, &dst, sizeof(struct can_frame));
    memcpy(dst, can_frame, sizeof(struct can_frame));
    ring_buf_put_finish(&can_rx_frame_buf, sizeof(struct can_frame)); // assume all bytes written

    k_sem_give(&can_rx_frame_buf_sem);
}

void can_rx_frame_handle_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
        /*
        k_sem_take(&can_rx_frame_buf_sem, K_FOREVER);
        */
        if (k_sem_take(&can_rx_frame_buf_sem, K_MSEC(1100)) != 0) {
            LOG_DBG("CAN frame rx buffer sem exceeded 1 sec timeout");
            continue;
        } 

        struct can_frame can_rx_frame; 
        uint8_t *src;

        ring_buf_get_claim(&can_rx_frame_buf, &src, sizeof(struct can_frame));
        memcpy(&can_rx_frame, src, sizeof(struct can_frame));
        ring_buf_get_finish(&can_rx_frame_buf, sizeof(struct can_frame)); // assume all bytes read

        CanardCANFrame canard_frame = {
            .id       = get_canard_id(&can_rx_frame),
            .data_len = can_rx_frame.dlc,
        };
        memcpy(canard_frame.data, can_rx_frame.data, sizeof(can_rx_frame.data));
        
        // TODO: - what the hell is the stm32 can timestamp 16-bit timer outputted in
        //       - also checkout how to enable stm32 can rx timestamp without enabling tx timestamp (which changes last two bytes of 8 byte can tx message to timestamp) 
        /*
        LOG_DBG("CAN Frame received with timestamp %llu(sys uptime) %u(can ticks) %u(can ms)", 
                k_uptime_get(),
                can_rx_frame.timestamp,
                k_cyc_to_ms_floor32(can_rx_frame.timestamp));
        */
        uint64_t data = 0;
        memcpy(&data, can_rx_frame.data, sizeof(can_rx_frame.data));
        /*
        LOG_DBG("CAN RX Frame id: %X, data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
                can_rx_frame.id,
                can_rx_frame.data[0],
                can_rx_frame.data[1],
                can_rx_frame.data[2],
                can_rx_frame.data[3],
                can_rx_frame.data[4],
                can_rx_frame.data[5],
                can_rx_frame.data[6],
                can_rx_frame.data[7]);
        */
        // const uint64_t timestamp_usec = k_cyc_to_us_floor64(canard_frame->timestamp);
        int ret = canardHandleRxFrame(&canard_ins, &canard_frame, k_uptime_get());
        // int ret = canardHandleRxFrame(&canard_ins, &canard_frame, 1000);
        if (ret < 0 && ret != -12 && ret != -13 && ret != -11) {
            LOG_DBG("Error handling CANARD frame (id %X, dlc %d): %d", canard_frame.id, canard_frame.data_len, ret);
        }
        k_yield();
    }
}


void can_send_canard_frame(const CanardCANFrame *canard_frame) {
    // LOG_DBG("Sending canard frame with id: %u", canard_frame->id);
    struct can_frame tx_frame = {
        .id    = canard_frame->id,
        .dlc   = canard_frame->data_len,
        .flags = CAN_FRAME_IDE
    };
    memcpy(tx_frame.data, canard_frame->data, sizeof(canard_frame->data));

    int ret = can_send(can_dev, &tx_frame, K_MSEC(100), NULL, NULL);

    if (ret != 0) {
        LOG_DBG("Failed to send can message %d", ret);
    } 
}

void loopback() {
    can_mode_t cap;
    ret = can_get_capabilities(can_dev, &cap);
    if (ret != 0) {
        LOG_DBG("Error getting capabilities: %d", ret);
    }

    LOG_DBG("cap: %d", cap);
    if (!(cap & CAN_MODE_LOOPBACK)) {
        LOG_DBG("no loopback :(");
        return;
    }

    ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
    if (ret != 0) {
        LOG_DBG("error setting can mode %d", ret);
        return;
    } 
}


bool should_accept_transfer(const CanardInstance *ins,
                            uint64_t *out_data_type_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id) {

    if (data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID || data_type_id == UAVCAN_EQUIPMENT_ESC_STATUS_ID) {
        return false;
    } else {
        LOG_DBG("Should accept transfer with id: 0x%X ?", data_type_id);
        // LOG_DBG("Received NODE status");
    }

    if ((transfer_type == CanardTransferTypeRequest) &&
        (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)) {
        // LOG_DBG("YOOO GETTING NODE INFO SHOULD ACCEPT");
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        // LOG_DBG("Made it here");
        return true;
    }
   
    // LOG_DBG("NO. Good.");
    return false;
}

void canard_get_node_info_handle(CanardRxTransfer *transfer) {
    LOG_DBG("YOOO HANDLING NODE INFO");
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uint8_t name[] = "kush_test";
    struct uavcan_protocol_GetNodeInfoResponse res = {
        .status = {
            .uptime_sec = get_uptime_sec_32(),
            .health     = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
            .mode       = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
        },
        .software_version = {
            .major = 9,
            .minor = 6,
            .image_crc = 0xFFFFFFFF,

        },
    };
    memcpy(res.name.data, name, strlen(name));
    res.name.len = strlen(name);

    uint32_t payload_size = uavcan_protocol_GetNodeInfoResponse_encode(&res, buffer);

    int ret = k_mutex_lock(&canard_tx_queue_mutex, K_MSEC(100));
    if (ret < 0) { 
        LOG_DBG("could not lock %d", ret);
        return;
    }
    ret = canardRequestOrRespond(&canard_ins,
                                 transfer->source_node_id,
                                 UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_SIGNATURE,
                                 UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID,
                                 &transfer->transfer_id,
                                 transfer->priority,
                                 CanardResponse,
                                 buffer,
                                 (uint16_t)payload_size);     
    k_mutex_unlock(&canard_tx_queue_mutex);

    if (ret < 0) {
        LOG_DBG("canard responsd failed: %d", ret);
    } else {
        LOG_DBG("Broadcasted canard get node info response");
    }
}

void canard_raw_esc_cmd_broadcast(int16_t raw_cmd[8]) {
    static uint8_t transfer_id = 0;
    LOG_DBG("Sending raw esc command: %d %d %d %d %d %d %d %d", 
            raw_cmd[0],
            raw_cmd[1],
            raw_cmd[2],
            raw_cmd[3],
            raw_cmd[4],
            raw_cmd[5],
            raw_cmd[6],
            raw_cmd[7]);
    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];

    struct uavcan_equipment_esc_RawCommand esc_cmd;
    esc_cmd.cmd.len = 8;
    memcpy(esc_cmd.cmd.data, raw_cmd, sizeof(int16_t) * 8);

    uint32_t payload_size = uavcan_equipment_esc_RawCommand_encode(&esc_cmd, buffer);
    // LOG_DBG("Payload size: %d", payload_size);

    int ret = k_mutex_lock(&canard_tx_queue_mutex, K_MSEC(100));
    if (ret < 0) { 
        LOG_DBG("could not lock %d", ret);
        return;
    }
    int r = canardBroadcast(&canard_ins,
                            UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE,
                            UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID,
                            &transfer_id,
                            CANARD_TRANSFER_PRIORITY_HIGH,
                            buffer,
                            (uint16_t)payload_size);
    if (r < 0) {
        LOG_DBG("CANARD BROADCAST FAILED: %d", r);
    } else {
        // LOG_DBG("Enqueued %d frames", r);
    }

    k_mutex_unlock(&canard_tx_queue_mutex);

    if (ret < 0) {
        LOG_DBG("canard esc command broadcast failed: %d", ret);
    }
}

void on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer) {
    LOG_DBG("here tho part 1");
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)) {
        LOG_DBG("here tho");
        canard_get_node_info_handle(transfer);
    } 
}



void canard_spin() {
    static uint8_t transfer_id = 0;

    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    struct uavcan_protocol_NodeStatus status = {
        .uptime_sec = get_uptime_sec_32(),
        .health     = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
        .mode       = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
    };
    uint32_t payload_size = uavcan_protocol_NodeStatus_encode(&status, buffer);

    int ret = k_mutex_lock(&canard_tx_queue_mutex, K_MSEC(100));
    if (ret < 0) { 
        LOG_DBG("could not lock %d", ret);
        return;
    }
    int r = canardBroadcast(&canard_ins,
                            UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                            UAVCAN_PROTOCOL_NODESTATUS_ID,
                            &transfer_id,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            buffer,
                            (uint16_t)payload_size);
    if (r < 0) {
        LOG_DBG("CANARD BROADCAST FAILED: %d", r);
    }
    k_mutex_unlock(&canard_tx_queue_mutex);
}

void canard_broadcast_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    static uint64_t prev_uptime_ms = 0;

    // int16_t raw[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    while (1) {
        const uint64_t cur_uptime_ms = k_uptime_get();
        // LOG_DBG("Prev: %llu cur: %llu", prev_uptime_ms, cur_uptime_ms);
        if (cur_uptime_ms - prev_uptime_ms > CANARD_SPIN_PERIOD_MS) {
            canardCleanupStaleTransfers(&canard_ins, cur_uptime_ms / 1000);
            canard_spin();
            // LOG_DBG("broadcast %d", raw[0]);
            // raw[0] += 1000;

            prev_uptime_ms = cur_uptime_ms;
        }

        int ret = k_mutex_lock(&canard_tx_queue_mutex, K_MSEC(100));
        if (ret < 0) { 
            LOG_DBG("could not lock %d", ret);
            return;
        }
        
        const CanardCANFrame *tx_frame = canardPeekTxQueue(&canard_ins);

        // The counter basically acts as a tiemout so that this while loop
        // doesn't lock the mutex too long
        int counter = 0;
        while (tx_frame && counter < 5) {
            uint8_t id = tx_frame->id >> 16;
            if (id == UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_ID) {
                // LOG_DBG("Sending GET NODE INFO RESPONSE %u", id);
		// Hela sus fix: for some reason this message bricks the code
		// sometimes, so I'm discarding this message
		canardPopTxQueue(&canard_ins);
		tx_frame = canardPeekTxQueue(&canard_ins);
		continue;
            } else if (id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
                // LOG_DBG("Sending node status RESPONSE %u", id);

            }
            can_send_canard_frame(tx_frame);
            canardPopTxQueue(&canard_ins);
            tx_frame = canardPeekTxQueue(&canard_ins);
            counter++;
        }
        k_mutex_unlock(&canard_tx_queue_mutex);
        k_yield();
    }
    return;
}

/*
CanardCANFrame get_node_info_req = {
    .id = 0x1E01E4FF | CANARD_CAN_FRAME_EFF,
    .data_len = 1,
    .data = {0xC0},
};
*/

struct can_frame get_node_info_req = {
    .id = 0x1E01E4FF,
    .dlc = 1,
    .data = {0xC0},
    .flags = CAN_FRAME_IDE
};


int can_hw_init() {
    if (!device_is_ready(can_dev)) {
        LOG_DBG("CAN device not found");
        return -1;
    } 

    // loopback();

    ret = can_start(can_dev);
    if (ret != 0) {
        LOG_DBG("Could not start CAN controller");
        return -1;
    }

    // accept all messages, enable extended 29-bit ID 
    struct can_filter filter = {
        .id    = 0,
        .mask  = 0,
        .flags = CAN_FILTER_DATA | CAN_FILTER_IDE
    };

    int filter_id;
    filter_id = can_add_rx_filter(can_dev, can_rx_irq_callback, NULL, &filter);
    if (filter_id < 0) {
        LOG_DBG("Unable to add rx filter [%d]", filter_id);
        return -1;
    }

    // let driver meditate
    k_sleep(K_MSEC(2000)); 

    return 0;
}

void canard_init() {
    canardInit(&canard_ins,
               canard_mem_pool,
               sizeof(canard_mem_pool),
               on_transfer_received,
               should_accept_transfer,
               NULL);
 
    canardSetLocalNodeID(&canard_ins, 100); 
}
int setup_thrusters(void) {
    LOG_DBG("Setting up thrusters");
    canard_init();
    ret = can_hw_init();
    if (ret < 0) {
        return - 1;
    }
    LOG_DBG("Initialized CAN driver");
    return 0;
}

void can_send_msg(uint8_t cnt, uint8_t id) {
    struct can_frame frame = {
        .id = id,
        .dlc = 8,
        .data = {cnt, cnt, cnt, cnt, cnt, cnt, cnt, cnt},
        .flags = CAN_FRAME_IDE
    };
    LOG_DBG("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
            frame.data[0],
            frame.data[1],
            frame.data[2],
            frame.data[3],
            frame.data[4],
            frame.data[5],
            frame.data[6],
            frame.data[7]);

    ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);

    if (ret != 0) {
        LOG_DBG("Failed to send can message %d", ret);
    } 
}

void send_thrusts(float thrusts[8]) {
    int16_t raw_cmd[8];
    for (int i = 0; i < 8; ++i) {
        raw_cmd[i] = 8191 * thrusts[i];
    }
    canard_raw_esc_cmd_broadcast(raw_cmd);
}


K_THREAD_DEFINE(canard_broadcast_thread_id, 4096,
                canard_broadcast_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
K_THREAD_DEFINE(can_rx_frame_handle_thread_id, 4096,
                can_rx_frame_handle_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

