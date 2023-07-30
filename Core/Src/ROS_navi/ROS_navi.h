#ifndef SRC_ROS_NAVI_H_
#define SRC_ROS_NAVI_H_

#include "../common.h"
#include "../adapter.h"
#include "../RNS_interface/RNS_interface.h"
#include "../PSx_Interface/PSx_Interface.h"

struct
{
	UART_HandleTypeDef* Recv_Vel;
	UART_HandleTypeDef* Send_Pos;

	float x_vel, y_vel, w_vel; // Received from ROS
	float vel1, vel2, vel3, vel4; // After MODN
	float x_pos, y_pos, yaw_pos; // Enq from RNS

}ROS_navi;

uint8_t pos_ack, vel_ack, vel[12], pos[12];



void ROS_Navi_Init(UART_HandleTypeDef* send_pos, UART_HandleTypeDef* recv_vel);
void ROS_Navi_Enq();
void ROS_Navi_Pos_Handler();
void ROS_Navi_Vel_Handler();

#endif
