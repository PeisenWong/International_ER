/*
 *  Manual Specifications:
 *  - worm gear for ring picking (Left and Right)
 *  - Servo  to close the ring rack after pick
 *  - Cylinder to load ring to shooting mechanism
 *  - Rack compinion to push the ring
 *  - Linear actuator for adjusting pitch
 *
 *
 *  Demo 4 Ideas
 *  1. Robot will be operated under 2 modes: Manual and Auto
 *  2. Use lidar to adjust after reached any position
 *  3. Able to disable the lidar adjustment afters reached destination
 *  4. Angle for each position must be set initially including pitch and speed of VESC
 *  5. Robot able to load/change pitch & speed automatically during changing position
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {
	set();
	const osThreadAttr_t MainTask_attributes =
			{ .name = "MainTask", .stack_size = 512 * 4, .priority =
					(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t CheckingTask_attributes =
			{ .name = "CheckingTask", .stack_size =512 * 4, .priority =
					(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t NaviTask_attributes =
			{ .name = "NaviTask", .stack_size = 512 * 4, .priority =
					(osPriority_t) osPriorityNormal, };

	const osThreadAttr_t EmergencyTask_attributes =
			{ .name = "EmergencyTask", .stack_size = 256 * 4, .priority =
					(osPriority_t) osPriorityNormal, };

	osKernelInitialize();

	MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
	NaviTaskHandle = osThreadNew(NaviTask, NULL, &NaviTask_attributes);
	CheckingTaskHandle = osThreadNew(CheckingTask, NULL,
			&CheckingTask_attributes);
	EmergencyTaskHandle = osThreadNew(EmergencyTask, NULL,
			&EmergencyTask_attributes);
	led2 = 1;
	osKernelStart();

	while (1) {

	}
}

void TIM6_DAC_IRQHandler(void) {

	static int counter = 0;
	counter++;

	if(counter % 4 == 0)
	{
		led1 = !led1;
		counter = 0;
//		if(mode == TUNE)
//		{
//			switch(wheel)
//			{
//				case 1:
//					sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f W: %d\n", AP, AI, AD, wheel);
//					break;
//				case 2:
//					sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f W: %d\n", BP, BI, BD, wheel);
//					break;
//				case 3:
//					sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f W: %d\n", CP, CI, CD, wheel);
//					break;
//				case 4:
//					sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f W: %d\n", DP, DI, DD, wheel);
//					break;
//			}
//		}
//		else
//		{
//	//		sprintf((char*) debug, "vX: %.2f vY: %.2f vrX: %.2f vrY: %.2f rz: %.2f\n", pp.vx, pp.vy, pp.rvx, pp.rvy, pp.real_z_rad);
//			sprintf((char*) debug, "v1: %.2f v2: %.2f v3: %.2f v4: %.2f ErrX: %.2f ErrY: %.2f ErrZ: %.2f L: %.2f R: %.2f start: %d\n", pp.v1, pp.v2, pp.v3, pp.v4,
//					pp.error_x, pp.error_y, pp.error_z, lsrL.dist, lsrR.dist, pp.lsr_start);
////			sprintf((char*) debug, "X: %.2f Y: %.2f Yaw: %.2f LX: %.2lf LY: %.2lf LAx: %.2f LAy: %.2f VESC: %.2f\n",
////					pp.real_x, pp.real_y, pp.real_z, lidar.pole.x, lidar.pole.y, lidar.adj_x, lidar.adj_y, vesc_duty);
////			sprintf((char*)debug, "Servo: %ld\n", servo_ring.currentPulse);
//	//		sprintf((char*)debug, "pos: %d errX: %.2f errY: %.2f X: %.2f Y: %.2f Z: %.2f Pick: %ld\n",
//	//				lidar.pos_counter, pp.error_x, pp.error_y, pp.real_x, pp.real_y, pp.real_z, pick_enc);
//	//		sprintf((char*)debug, "VESC: %f\n", vesc_duty);
//		}

		sprintf((char*) debug, "X: %.2f Y: %.2f Yaw: %.2f VESC: %.2f Extra: %d P: %ld LSRL: %.2f LSRR: %.2f\n",
				pp.real_x, pp.real_y, pp.real_z, vesc_duty, extra_rpm, pick_enc, lsrL.dist, lsrR.dist);
//		sprintf((char*) debug, "Left: %.2f Right: %.2f VESC: %.2f\n",
//				vesc1.vel, vesc2.vel, vesc_speed);
		HAL_UART_Transmit(&huart5, debug, strlen((char*) debug), HAL_MAX_DELAY);
	}

//	fXEncData = (0.05 / 4000.0 * 3.142) * QEIRead(QEI1);
//	fYEncData = (0.05 / 4000.0 * 3.142) * -QEIRead(QEI4);
//
//	ABT(&x_data);
//	ABT(&y_data);
//
//	fyaw = (fyaw >= 0.0) ? fyaw : 360.0 + fyaw;
//	enq();

	// Speed PID
//	vesc1.vel = VESC_calVel(vesc1.Info.rpm);
//	vesc2.vel = VESC_calVel(vesc2.Info.rpm);
//
//	if(fabs(vesc_speed - vesc1.vel) < 0.1 && fabs(vesc_speed - vesc2.vel) < 0.1)
//	{
//		led5_on;
//	}
//	else
//	{
//		led5_off;
//	}
//
//	flywheelPID(vesc_speed);
//	flywheelAct();

//	enq();
	PathPlan(&pp);
	CheckPickEnc();

//	sprintf((char*) debug, "X: %.2f Y: %.2f Yaw: %.2f \n", fXPos, fYPos, fyaw);
//	HAL_UART_Transmit(&huart4, debug, strlen((char*) debug), HAL_MAX_DELAY);
	HAL_TIM_IRQHandler(&htim6);

}

// Can be polling
void MainTask(void *argument) {
	RobotStart();
	while (1) {
		if (ps4.button == SELECT)
		{
			while (ps4.button == SELECT);
			mode++;

			if (mode > 1)
				mode = 0;
		}

		if(ps4.button == L1)
		{
			while(ps4.button == L1);
			vesc_speed += 0.05;
			vesc_duty += 5;
			manual_adjust++;
//			ServoMovePulse(&servo_ring, 50);
//			ServoMovePulse(&servo_ring_new, -50);
		}

		if(ps4.button == R1)
		{
			while(ps4.button == R1);
			vesc_speed -= 0.05;
			vesc_duty -= 5;
			manual_adjust--;
//			ServoMovePulse(&servo_ring, -50);
//			ServoMovePulse(&servo_ring_new, 50);
		}

		switch (mode)
		{
			case NORMAL:
				NormalControl();
			break;

			case AUTO:
				Auto();
				LidarControl(&lidar);
			break;

			case TUNE:
				tune();
			break;

		}
	}
}

// Cannot Polling
void NaviTask(void *argument) {
	while (1) {

//		if(!set_pitch)
//		{
//			if (ps4.button == UP)
//			{
//				pitch_up;
//			}
//			else if (ps4.button == DOWN)
//			{
//				pitch_down;
//			}
//			else
//			{
//				pitch_stop;
//			}
//		}

		switch(go_type_3)
		{
			case 0:
				led5_off;
				break;

			case 1:
				led5_on;
				break;

			case 2:
				led5_off;
				break;

			case 3:
				led5_on;
				break;

			default:
				break;
		}

		if(!load_start && !set_pick_enc && !reload)
			pick_manual(ps4.joyR_y * 14000);

		if(vesc_duty > 0.0)
			vesc_start = 1;
		else
			vesc_start = 0;

		if(vesc_start)
		{
			if(vesc_duty == type1Duty)
			{
				VESCRPM(vesc_duty, &vesc1);
				VESCRPM(vesc_duty, &vesc2);
			}
			else
			{
				VESCRPM(vesc_duty + extra_rpm, &vesc1);
				VESCRPM(vesc_duty + extra_rpm, &vesc2);
			}
		}

//		CheckPick0();

//		if(stick_fence && MODN.x_vel == 0.0 && MODN.y_vel == 0.0 && !In_LS_Shot_1 && !In_LS_Shot_2)
//			RNSVelocity(0.25, 0.25, 0.25, 0.25, &rns);
//		else if(stick_fence && In_LS_Shot_1 && In_LS_Shot_2)
//			RNSStop(&rns);

		enq();
		pick_enc = QEIRead(QEI1);

		if(!pp.pp_start && !vel_adjust && !pp.lsr_start)
		{
//			if (ps4.button == L1) {
//				while (ps4.button == L1);
//				moveSpeed++;
//				setSpeedMODN(moveSpeed);
//			}
//
//			if (ps4.button == R1) {
//				while (ps4.button == R1);
//				moveSpeed--;
//				setSpeedMODN(moveSpeed);
//			}
			realMODN(&ps4, &rns);
		}
		else if(pp.pp_start || pp.lsr_start)
			RNSVelocity(pp.v1, pp.v2, pp.v3, pp.v4, &rns);

	}
}

void CheckingTask(void *argument)
{
	while (1)
	{
		Checking();

//		if(In_LS_Shot_1 && In_LS_Shot_2)
//		{
//			led4_on;
//		}
//		else
//		{
//			led4_off;
//		}

		if(blue)
		{
			led4_on;
		}
		else
		{
			led4_off;
		}

//		if(In_LS_Left_1 && In_LS_Left_2)
//		{
//			led5_on;
//		}
//		else
//		{
//			led5_off;
//		}

		if(In_Load)
		{
			led6_on;
		}
		else
		{
			led6_off;
		}

		if(In_Pick)
		{
			led7_on;
		}
		else
		{
			led7_off;
		}

		if(led_enb)
		{
			if (HAL_GetTick() - before_shot >= shot_prd)
			{
				led8 = !led8;
				before_shot = HAL_GetTick();
			}
		}
	}
}

void EmergencyTask(void *argument) {

	while (1) {

		if(servo_enb)
		{
			for(int i = 0; i < 4; i++)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					break;
				}

				load_adjust_servo;
				osDelay(100);
				close_servo;
				osDelay(100);

//				if(stop_adjust)
//				{
//					stop_adjust = 0;
//					break;
//				}

	//			if(In_ShotDone)
	//			{
	//				adjust_servo;
	//				break;
	//			}
			}
			adjust_servo;
			servo_enb = 0;
		}

		float angle_shoot_point[1][7] = {{1.5, 0, -0.2, -90, 0, 0, 0}};

		if (ps4.button == TOUCH) {
//			while(ps4.button == TOUCH);
//			RNSStop(&rns);
//			PP_stop(&pp);
//			pitch_stop;
//			pick_stop;
//			push_stop;
//			vesc_duty = 0.0;
//			osThreadTerminate(MainTaskHandle);
//			osDelay(5);
//
//			const osThreadAttr_t MainTask_attributes = { .name = "MainTask",
//					.stack_size = 256 * 4, .priority =
//							(osPriority_t) osPriorityNormal, };
//
//			MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
			while(ps4.button == TOUCH);
			static int counter = 0;
			if(counter == 0)
			{
				angle_shoot = 1;
				led5_on;
				ResetCoordinate();
				PP_start(angle_shoot_point, 1, &pp);
				while(pp.pp_start)
				{
					if(ps4.button == SQUARE)
					{
						while(ps4.button == SQUARE);
						PP_stop(&pp);
					}
				}
				lidar.angle_counter = BR;
				counter++;
			}
			else
			{
				angle_shoot = 0;
				led5_off;
				counter = 0;
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == IMU.huartx) {
		R6091U_Handler(&IMU);
	}

	if(huart == lidar.lidar_UART)
	{
		ObstacleHandler(&lidar);
	}
}

/**
 * @brief  This function is executed in case of errorYaw occurrence.
 */
void Error_Handler(void) {

}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
