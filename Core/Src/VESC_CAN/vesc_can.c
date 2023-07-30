/*
 * vesc_can.c
 *
 *  Created on: Dec 23, 2020
 *      Author: root
 */
#include "vesc_can.h"

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_last_id;
static Vescmsg rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read = 0;
static int rx_frame_write = 0;

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * 0: Packet goes to commands_process_packet of receiver
 * 1: Packet goes to commands_send_packet of receiver --> can use to send to other board?
 * 2: Packet goes to commands_process and send function is set to null
 *    so that no reply is sent back.
 */
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = (uint8_t)RNS_TO_VESC;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
#if defined USED_CAN1
			CAN_TxMsgEID(&hcan1, controller_id |
					((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
#elif defined USED_CAN2
			CAN_TxMsgEID(&hcan2, controller_id |
								((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
#endif
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}
#if defined USED_CAN1
			CAN_TxMsgEID(&hcan1, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
#elif defined USED_CAN2
			CAN_TxMsgEID(&hcan2, controller_id |
								((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
#endif

		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

#if defined USED_CAN1
			CAN_TxMsgEID(&hcan1, controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
#elif defined USED_CAN2
			CAN_TxMsgEID(&hcan2, controller_id |
								((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
#endif
		}

		uint32_t ind = 0;
		send_buffer[ind++] = (uint8_t)RNS_TO_VESC;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

#if defined USED_CAN1
			CAN_TxMsgEID(&hcan1, controller_id |
					((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
#elif defined USED_CAN2
			CAN_TxMsgEID(&hcan2, controller_id |
								((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
#endif
	}
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
#endif
}

void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
#endif
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
#endif

}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
#endif
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
#endif
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0 1.0]
 */
void comm_can_set_current_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
#endif
}

/**
 * Set brake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0 1.0]
 */
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
#endif
}

/**
 * Set handbrake current.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void comm_can_set_handbrake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
#endif
}

/**
 * Set handbrake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0 1.0]
 */
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
#if defined USED_CAN1
	CAN_TxMsgEID(&hcan1, controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
#elif defined USED_CAN2
	CAN_TxMsgEID(&hcan2, controller_id |
				((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL<< 8), buffer, send_index);
#endif
}

void decode_VESC(void){
	int32_t ind = 0;
//	unsigned int rxbuf_len;
//	unsigned int rxbuf_ind;
//	uint8_t crc_low;
//	uint8_t crc_high;
//	uint8_t commands_send;

//
//	Vescmsg *rxmsg_tmp;
//	while ((rxmsg_tmp = get_rx_frame()) != 0) {
//		Vescmsg rxmsg = *rxmsg_tmp;

		if (vescmsg.Rxmsg.IDE == CAN_ID_EXT) {
			uint8_t id = vescmsg.Rxmsg.ExtId & 0xFF;
			CAN_PACKET_ID cmd = vescmsg.Rxmsg.ExtId >> 8;

//			if (id == 255 || id == RNS_TO_VESC) {
//				switch (cmd) {
//				case CAN_PACKET_FILL_RX_BUFFER:
//					memcpy(rx_buffer + vescmsg.Data[0], vescmsg.Data + 1, vescmsg.Rxmsg.DLC - 1);
//					break;
//
//				case CAN_PACKET_FILL_RX_BUFFER_LONG:
//					rxbuf_ind = (unsigned int)vescmsg.Data[0] << 8;
//					rxbuf_ind |= vescmsg.Data[1];
//					if (rxbuf_ind < RX_BUFFER_SIZE) {
//						memcpy(rx_buffer + rxbuf_ind, vescmsg.Data + 2, vescmsg.Rxmsg.DLC - 2);
//					}
//					break;
//
//				case CAN_PACKET_PROCESS_RX_BUFFER:
//					ind = 0;
//					rx_buffer_last_id = vescmsg.Data[ind++];
//					commands_send = vescmsg.Data[ind++];
//					rxbuf_len = (unsigned int)vescmsg.Data[ind++] << 8;
//					rxbuf_len |= (unsigned int)vescmsg.Data[ind++];
//
//					if (rxbuf_len > RX_BUFFER_SIZE) {
//						break;
//					}
//
//					crc_high = vescmsg.Data[ind++];
//					crc_low = vescmsg.Data[ind++];
//
//					if (crc16(rx_buffer, rxbuf_len)
//							== ((unsigned short) crc_high << 8
//									| (unsigned short) crc_low)) {
//						if(commands_send==1)
//							bldc_interface_process_packet(rx_buffer, rxbuf_len);
//					}
//					break;
//
//				case CAN_PACKET_PROCESS_SHORT_BUFFER:
//					ind = 0;
//					rx_buffer_last_id = vescmsg.Data[ind++];
//					commands_send = vescmsg.Data[ind++];
//
//					if(commands_send==1)
//						bldc_interface_process_packet(rx_buffer, rxbuf_len);
//					break;
//
//				default:
//					break;
//				}
//			}
			if(cmd == CAN_PACKET_STATUS)
			{
				if(id == 111)
				{
					vesc1.Info.rpm = buffer_get_float32((uint8_t*)&vescmsg.Data, 1.0, &ind);
					vesc1.Info.current = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &ind);
					vesc1.Info.duty = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &ind);
				}
				else if(id == 112)
				{
					vesc2.Info.rpm = buffer_get_float32((uint8_t*)&vescmsg.Data, 1.0, &ind);
					vesc2.Info.current = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &ind);
					vesc2.Info.duty = buffer_get_float16((uint8_t*)&vescmsg.Data, 1e1, &ind);
				}
			}
		}
	}

Vescmsg *get_rx_frame(void) {
	if (rx_frame_read != rx_frame_write){
		Vescmsg *res = &rx_frames[rx_frame_read++];

		if (rx_frame_read == RX_FRAMES_SIZE){
			rx_frame_read = 0;
		}

		return res;
	} else
		return 0;
}

uint8_t set_rx_frames(Vescmsg* CANRxFrame) {
	uint32_t cmd;

	rx_frames[rx_frame_write++] = *CANRxFrame;
	if (rx_frame_write == RX_FRAMES_SIZE) {
		rx_frame_write = 0;
	}

	cmd = CANRxFrame->Rxmsg.ExtId >> 8;
	if(cmd == (uint32_t)CAN_PACKET_PROCESS_RX_BUFFER || cmd == (uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER)
		return 112;
	else
		return 0;
}



