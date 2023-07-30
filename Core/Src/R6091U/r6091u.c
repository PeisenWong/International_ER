/*
 * r6091u.c
 *
 *  Created on: Oct 10, 2021
 *      Author: amera
 */

#include "r6091u.h"
#include "../adapter.h"

void IMU_InitI2C(R6091U_t* IMU, I2C_HandleTypeDef *hi2c){
//	I2CxInit(hi2c, main_board_2, CLOCK_SPEED_100KHz, ENABLE);
	IMU->hi2cimu = hi2c;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	HAL_I2C_Master_Receive_IT(IMU->hi2cimu, 0x35 << 1, (uint8_t*)IMU->Buffer, 20);
}

void IMU_InitI2C_DMA(R6091U_t* IMU, I2C_HandleTypeDef *hi2c){
	IMU->hi2cimu = hi2c;
	IMU->checksum = 0;
	IMU->offset = 0;
	IMU->yaw_constant = 0;
	IMU->prev_yaw = 0;
	HAL_I2C_Master_Receive_DMA(IMU->hi2cimu, 0x35 << 1, (uint8_t*)IMU->Buffer, 20);
}

void IMU_I2CHandle(R6091U_t* IMU){
	IMU->checksum = 0;
	for(int i = 0; i < 19; i++)
		IMU->checksum += IMU->Buffer[i];

	if(IMU->checksum == IMU->Buffer[19]){
		IMU->roll = *((int16_t *)&IMU->Buffer[0]) / 100.0;
		IMU->pitch = *((int16_t *)&IMU->Buffer[2]) / 100.0; 	 // -90 to 90
		IMU->yaw = *((int16_t *)&IMU->Buffer[4]) / 100.0;	 //-180 to 180
		IMU->roll_rate = *((int16_t *)&IMU->Buffer[6]) / 100.0;
		IMU->pitch_rate = *((int16_t *)&IMU->Buffer[8]) / 100.0;
		IMU->yaw_rate = *((int16_t *)&IMU->Buffer[10]) / 100.0;
		IMU->x_acc = *((int16_t *)&IMU->Buffer[12]) / 1000 * 9.8067;
		IMU->y_acc = *((int16_t *)&IMU->Buffer[14]) / 1000 * 9.8067;
		IMU->z_acc = *((int16_t *)&IMU->Buffer[16]) / 1000 * 9.8067;
		IMU->index = IMU->Buffer[18];
	}

	if(IMU->yaw < -150.0){
		if(IMU->prev_yaw > 150.0){
			IMU->yaw_constant++;
		}
	}else if(IMU->yaw > 150.0){
		if(IMU->prev_yaw < -150.0){
			IMU->yaw_constant--;
		}
	}

	IMU->prev_yaw = IMU->yaw;
	IMU->real_z = IMU->yaw + IMU->yaw_constant * 360.0 + IMU->offset;
	IMU->real_zrad = (IMU->real_z / 180.0) * 3.141593;
	memset(IMU->Buffer, 0, 20);
	HAL_I2C_Master_Receive_IT(IMU->hi2cimu, 0x35 << 1, (uint8_t*)IMU->Buffer, 20);
}

void IMU_DMAHandle(R6091U_t* IMU){
	IMU->checksum = 0;
	for(int i = 0; i < 19; i++)
		IMU->checksum += IMU->Buffer[i];

	if(IMU->checksum == IMU->Buffer[19]){
		IMU->roll = *((int16_t *)&IMU->Buffer[0]) / 100.0;
		IMU->pitch = *((int16_t *)&IMU->Buffer[2]) / 100.0; 	 // -90 to 90
		IMU->yaw = *((int16_t *)&IMU->Buffer[4]) / 100.0;	 //-180 to 180
		IMU->roll_rate = *((int16_t *)&IMU->Buffer[6]) / 100.0;
		IMU->pitch_rate = *((int16_t *)&IMU->Buffer[8]) / 100.0;
		IMU->yaw_rate = *((int16_t *)&IMU->Buffer[10]) / 100.0;
		IMU->x_acc = *((int16_t *)&IMU->Buffer[12]) / 1000 * 9.8067;
		IMU->y_acc = *((int16_t *)&IMU->Buffer[14]) / 1000 * 9.8067;
		IMU->z_acc = *((int16_t *)&IMU->Buffer[16]) / 1000 * 9.8067;
		IMU->index = IMU->Buffer[18];
	}

	if(IMU->yaw < -150.0){
		if(IMU->prev_yaw > 150.0){
			IMU->yaw_constant++;
		}
	}else if(IMU->yaw > 150.0){
		if(IMU->prev_yaw < -150.0){
			IMU->yaw_constant--;
		}
	}

	IMU->prev_yaw = IMU->yaw;
	IMU->real_z = IMU->yaw + IMU->yaw_constant * 360.0 + IMU->offset;
	IMU->real_zrad = (IMU->real_z / 180.0) * 3.141593;
	memset(IMU->Buffer, 0, 20);
	HAL_I2C_Master_Receive_DMA(IMU->hi2cimu, 0x35<<1, (uint8_t*)&IMU->Buffer, 20);//RECEIVE FROM R6091U
}


void R6091U_Init(R6091U_t* IMU,UART_HandleTypeDef* huartx){

	IMU->huartx = huartx;
	IMU->State = PENDING_SYNC;
	HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);

}
void R6091U_Handler(R6091U_t* IMU){

//	uint8_t checksum;

	switch(IMU->State){


	case PENDING_SYNC:

		if(IMU->Buffer[0] == 0xAA){
			IMU->State = CONFIRMING_SYNC;
		}

		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		break;

	case CONFIRMING_SYNC:

		if(IMU->Buffer[0] == 0x00){
			IMU->State = IN_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 13);

		}else{

			IMU->State = PENDING_SYNC;
			HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);
		}

		break;

	case IN_SYNC:
		IMU->checksum = 0;

		IMU->checksum = IMU->Buffer[0] + IMU->Buffer[1] + IMU->Buffer[2] + IMU->Buffer[3] + IMU->Buffer[4] + IMU->Buffer[5]
						+ IMU->Buffer[6] + IMU->Buffer[7] + IMU->Buffer[8] + IMU->Buffer[9] + IMU->Buffer[10] + IMU->Buffer[11];


		if( IMU->checksum == IMU->Buffer[12]){
//		int16_t yaw = ((IMU->Buffer[1] & 0xFF)) | ((IMU->Buffer[2] << 8) & 0xFF00) ;
			int16_t yaw = *((uint16_t*)&IMU->Buffer[1]);
//			int16_t x_accel = *((uint16_t*)&IMU->Buffer[5]);
//			int16_t y_accel = *((uint16_t*)&IMU->Buffer[7]);
//			int16_t z_accel = *((uint16_t*)&IMU->Buffer[9]);
//			fx_accel = (float)x_accel / 1000.0 * 9.81;
//			fy_accel = (float)y_accel / 1000.0 * 9.81;
//			fz_accel = (float)z_accel / 1000.0 * 9.81;
//			pitch = asin(fx_accel / sqrt(pow(fx_accel, 2) + pow(fy_accel, 2) + pow(fz_accel, 2)));
//			pitch = 180.0 / M_PI * atan2(-fx_accel, sqrt(pow(fy_accel, 2) + pow(fz_accel, 2)));
//			roll = atan(fy_accel / fz_accel);
			fyaw = ((float)(yaw) / (float)100.0) + 180.0 ;
			if(testCounter2 >= 20){
//				LED3 = !LED3;
				testCounter2 = 0;
			}else
				testCounter2 ++;
		}
		memset(IMU->Buffer, 0, 13);
		IMU->State = PENDING_SYNC;
		HAL_UART_Receive_IT(IMU->huartx, IMU->Buffer, 1);

		break;
	}
}
