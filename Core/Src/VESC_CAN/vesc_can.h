/*
 * vesc_can.h
 *
 *  Created on: Dec 23, 2020
 *      Author: root
 */

#ifndef VESC_CAN_VESC_CAN_H_
#define VESC_CAN_VESC_CAN_H_

//#include "stm32f4xx_conf.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "bldc_interface.h"
#include "buffer.h"
#include "crc.h"
#include "datatypes.h"
#include "../CAN/can.h"
#include "../adapter.h"


// Settings
#define USED_CAN2

#define RX_FRAMES_SIZE	100
#define RX_BUFFER_SIZE	512


enum{
	VESC1 = 111,
	VESC2,
	VESC3,
	VESC4,
};

typedef struct {
	CAN_RxHeaderTypeDef Rxmsg;
	uint8_t Data[8];
} Vescmsg;


enum{
	mainboard_TO_VESC = 29,
		RNS_TO_VESC
};

void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

void decode_VESC(void);
Vescmsg *get_rx_frame(void);
uint8_t set_rx_frames(Vescmsg* CANRxFrame);

Vescmsg vescmsg;
#endif /* VESC_CAN_VESC_CAN_H_ */
