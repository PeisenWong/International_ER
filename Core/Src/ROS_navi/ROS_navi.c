#include "ROS_navi.h"

void ROS_Navi_Init(UART_HandleTypeDef* send_pos, UART_HandleTypeDef* recv_vel)
{
	ROS_navi.Recv_Vel = recv_vel;
	ROS_navi.Send_Pos = send_pos;

	sys.stop = 0;
	sys.vel_ready = 0;
	HAL_UART_Receive_IT(send_pos, &pos_ack, 1); // Got ack to send pos
	HAL_UART_Receive_IT(recv_vel, vel, 12); // Receive vel command from ROS
}

void ROS_Navi_Enq()
{
	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
	ROS_navi.x_pos = rns.enq.enq_buffer[0].data;
	ROS_navi.y_pos = rns.enq.enq_buffer[1].data;
	ROS_navi.yaw_pos = rns.enq.enq_buffer[3].data; // In radian

	memcpy(&pos[0], &ROS_navi.x_pos, 4);
	memcpy(&pos[4], &ROS_navi.y_pos, 4);
	memcpy(&pos[8], &ROS_navi.yaw_pos, 4);
}

void ROS_Navi_Pos_Handler()
{
	if(pos_ack == 0x01)
	{
		HAL_UART_Transmit(ROS_navi.Send_Pos, pos, 12, HAL_MAX_DELAY);
	}
	HAL_UART_Receive_IT(ROS_navi.Send_Pos, &pos_ack, 1);
}

void ROS_Navi_Vel_Handler()
{
	memcpy(&ROS_navi.x_vel, &vel[0], 4);
	memcpy(&ROS_navi.y_vel, &vel[4], 4);
	memcpy(&ROS_navi.w_vel, &vel[8], 4);

	sys.vel_ready = 1;

	vel_ack = 0x10;

	if(!sys.stop)
	{
		HAL_UART_Transmit(ROS_navi.Recv_Vel, &vel_ack, 1, HAL_MAX_DELAY);
		HAL_UART_Receive_IT(ROS_navi.Recv_Vel, vel, 12);
	}
}
