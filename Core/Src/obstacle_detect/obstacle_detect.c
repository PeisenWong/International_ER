#include "obstacle_detect.h"
#include "../adapter.h"

/*
 *  Current strategy is to obtain the nearest pole using lidar twice only, which is picking from left and right, or maybe self positioning using nearest pole
 *  Second strategy: Use Lidar positioning every time moved to a new position
 *  I will set 3 shooting position (Center, Upper left and Upper right)
 *  Robot will be able to change its position by LEFT and RIGHT
 *  After picking from left, robot will navigate to Upper Left to aim for 4 poles(excluding type 3)
 *  Robot will be able to change its angle by button UP and DOWN
 *  If the rings still sufficient, navigate to Center to aim for 2 poles(Center type 1 and type 3)
 *  Then navigate to Upper Right to aim for 4 poles, just like Upper Left but turning anti-clockwise direction
 *
 *  Risks:
 *  May accidently scan judges or operator leg (Need to process outputs from PC)
 *  May received corrupted message, but this can be notified through the response_t
 */

/*
 * center 4 X: -0.25 Y: 0.93
 * center 3 type 2 X: -1.5 Y: 0.92
 * center type 1: X: -0.21 Y: 0.92 type 3: -0.1
 * center 2 type 2: X: 1.19 Y: 0.97
 * center 1 type 1: X: -0.25 Y: 0.99
 *
 * LASER
 * center_1: 2.56 Err: 2.15
 * center_2: 4.42 Err: 3.99, 5.04
 * center: 5.79 Err: 6.09
 * center_3: 4.51 Err: 5.05
 * center_4: 2.61  Err: 2.04
 */

// The sequence of array: Left -> Right -> Center
float Lidar_Offsets[6] = {0, 0, 0, 0, -0.21, 0.92};
float Lidar_Shoot_Angles[10] = {0, -90.0, -135, -45, 0, 0, 0, 0, 0, 0}; // The angles should be absolute, try not to reset the angle every path plan
float Lidar_Center_Offsets[8] = {-0.25, 0.99, 1.19, 0.96, -1.5, 0.99, -0.25, 0.91};  // Offsets for center positions
float Lidar_Adjust_Lim[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void LidarInit(UART_HandleTypeDef* lidarUART, MODE_t mode, Lidar_t* lidar)
{
	lidar->lidar_UART = lidarUART;
	lidar->mode = mode;
	lidar->start = 0;

	if(lidar->mode == CONTINUOUS)
	{
		HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 5);
		lidar->new = 0;
	}
	else
	{
		LidarAnglesInit(Lidar_Shoot_Angles[0], Lidar_Shoot_Angles[1], Lidar_Shoot_Angles[2], Lidar_Shoot_Angles[3],
				Lidar_Shoot_Angles[4], Lidar_Shoot_Angles[5], Lidar_Shoot_Angles[6], Lidar_Shoot_Angles[7], Lidar_Shoot_Angles[8], Lidar_Shoot_Angles[9], lidar);
		LidarOffsetInit(Lidar_Offsets[0], Lidar_Offsets[1], Lidar_Offsets[2], Lidar_Offsets[3], Lidar_Offsets[4], Lidar_Offsets[5], lidar);
		LidarCenterOffsetInit(Lidar_Center_Offsets[0], Lidar_Center_Offsets[1], Lidar_Center_Offsets[2], Lidar_Center_Offsets[3], Lidar_Center_Offsets[4],
				Lidar_Center_Offsets[5], Lidar_Center_Offsets[6], Lidar_Center_Offsets[7], lidar);
		lidar->response = NO;
		lidar->fail = 0;
		lidar->pos = PICK_LEFT;
		lidar->pos_counter = 1;
		lidar->angle = S;
		lidar->angle_counter = 2;
		lidar->AdjEnb = 0;
		lidar->laser = 1;
		lidar->autoshot = 1;
		HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 2);
	}
}

void LidarOffsetInit(float left_offset_x, float left_offset_y, float right_offset_x, float right_offset_y, float center_offset_x,
		float center_offset_y, Lidar_t* lidar)
{
	lidar->left_offset_x = left_offset_x;
	lidar->left_offset_y = left_offset_y;
	lidar->right_offset_x = right_offset_x;
	lidar->right_offset_y = right_offset_y;
	lidar->center_offset_x = center_offset_x;
	lidar->center_offset_y = center_offset_y;
}

void LidarCenterOffsetInit(float center_1_offset_x, float center_1_offset_y, float center_2_offset_x, float center_2_offset_y,
		float center_3_offset_x, float center_3_offset_y, float center_4_offset_x, float center_4_offset_y, Lidar_t* lidar)
{
	lidar->center_1_offset_x = center_1_offset_x;
	lidar->center_1_offset_y = center_1_offset_y;
	lidar->center_2_offset_x = center_2_offset_x;
	lidar->center_2_offset_y = center_2_offset_y;
	lidar->center_3_offset_x = center_3_offset_x;
	lidar->center_3_offset_y = center_3_offset_y;
	lidar->center_4_offset_x = center_4_offset_x;
	lidar->center_4_offset_y = center_4_offset_y;

}

// Will be shooting based on sequence A-> B -> C -> D
void LidarAnglesInit(float left_angle_A, float left_angle_B, float left_angle_C, float left_angle_D,
					 float right_angle_A, float right_angle_B, float right_angle_C, float right_angle_D,
					 float center_angle_A, float center_angle_B, Lidar_t* lidar)
{
	lidar->left_angle_A = left_angle_A;
	lidar->left_angle_B = left_angle_B;
	lidar->left_angle_C = left_angle_C;
	lidar->left_angle_D = left_angle_D;

	lidar->right_angle_A = right_angle_A;
	lidar->right_angle_B = right_angle_B;
	lidar->right_angle_C = right_angle_C;
	lidar->right_angle_D = right_angle_D;

	lidar->center_angle_A = center_angle_A;
	lidar->center_angle_B = center_angle_B;
}

void LidarSetPos(Pos_t pose, Lidar_t* lidar)
{
	uint32_t before, after;

//	*******************************************************************************LASER*************************************************************************************
	float lsr_center_1[1][7] = {{0, 2.56, pp.real_y + 0.3, -90, PoleSpeed, 0.06, 1}};
	float lsr_center_2[1][7] = {{0, 4.37, pp.real_y + 0.3, -90, PoleSpeed, 0.06, 1}};
	float lsr_center[1][7] = {{0, 5.74, pp.real_y + 0.3, -90, PoleSpeed, 0.06, 1}};
	float lsr_center_right[1][7] = {{0, 5.86, pp.real_y + 0.3, -90, PoleSpeed, 0.06, 1}};
	float lsr_center_3[1][7] = {{0, 4.60, pp.real_y + 0.3, -90, PoleSpeed, 0.06, 1}};
	float lsr_center_4[1][7] = {{0, 2.67, pp.real_y + 0.3, -90, PoleSpeed, 0.06, 1}};
//	*******************************************************************************LASER*************************************************************************************

	// Pick Left
	float pick_left_adjust[1][7] = {{3.5, 1.8, 0.6, -90.0, 0, 0, 0}};
	float laser_adjust_center_4[1][7] = {{3.5, 5, 0.1, -90, 0, 0, 0}};
//	float bang_fence[1][7] = {{0.75, 0, 10, -90, 0, 0, 0}};

	// Only offset 1 pole
	float pick_left_to_center_1[1][7] = {{0.75, 0, 10, -90, 0, 0, 0}};
	float center_1_to_center_2[1][7] = {{PoleSpeed, 1.35, 0.2, -90, 0, 0, 0}};
	float center_2_to_center_1[1][7] = {{PoleSpeed, -1.4, 0.2, -90, 0, 0, 0}};
	float center_2_to_center[1][7] = {{PoleSpeed, 0.95, 0.2, -90, 0, 0, 0}};
	float center_to_center_2[1][7] = {{PoleSpeed, -1.0, 0.2, -90, 0, 0, 0}};
	float center_to_center_3[1][7] = {{PoleSpeed, 0.9, 0.2, -90, 0, 0, 0}};
	float center_3_to_center[1][7] = {{PoleSpeed, -0.85, 0.2, -90, 0, 0, 0}};
	float center_3_to_center_4[1][7] = {{PoleSpeed, 1.5, 0.2, -90, 0, 0, 0}};
	float center_4_to_center_3[1][7] = {{PoleSpeed, -1.4, 0.1, -90, 0, 0, 0}};
	float center_4_to_upper_right[1][7] = {{PoleSpeed, 1.55, 0.0, -90, 0, 0, 0}};
	float center_4_to_upper_right_2[1][7] = {{PoleSpeed, 0.3, 6.0, -178.0, 0, 0, 0}};
	float upper_right_to_center_4[1][7] = {{PoleSpeed, 0.5, -4.1, -90, 0, 0, 0}};
	float upper_right_to_center_4_2[1][7] = {{PoleSpeed, -1.3, 0.0, -90.0, 0, 0, 0}};
	float upper_right_to_pick_right[1][7] = {{PoleSpeed, 0.5, -4.35, -180.0, 0, 0, 0}};
	float pick_right_to_upper_right[1][7] = {{PoleSpeed, -0.2, 0.5, -90, 0, 0, 0}};
	float pick_right_to_upper_right_2[1][7] = {{PoleSpeed, 0.2, 6, -178.0, 0, 0, 0}};

	// Offset 2 poles
	float center_1_to_center[1][7] = {{PoleSpeed, 2.6, 0.2, -90, 0, 0, 0}};
	float center_to_center_1[1][7] = {{PoleSpeed, -2.7, 0.2, -90, 0, 0, 0}};
	float center_to_center_4[1][7] = {{PoleSpeed, 2.55, 0.2, -90, 0, 0, 0}};
	float center_4_to_center[1][7] = {{PoleSpeed, -2.7, 0.2, -90, 0, 0, 0}};
	float center_2_to_center_3[1][7] = {{PoleSpeed, 2.0, 0.2, -90, 0, 0, 0}};
	float center_3_to_center_2[1][7] = {{PoleSpeed, -1.85, 0.2, -90, 0, 0, 0}};
	float center_3_to_upper_right[1][7] = {{PoleSpeed, 3.4, 0.0, -90, 0, 0, 0}};
	float center_3_to_upper_right_2[1][7] = {{PoleSpeed, 0.3, 6.0, -178.0, 0, 0, 0}};
	float upper_right_to_center_3[1][7] = {{PoleSpeed, 0.5, -4.2, -90, 0, 0, 0}};
	float upper_right_to_center_3_2[1][7] = {{PoleSpeed, -3.3, 0.0, -90, 0, 0, 0}};
	float center_4_to_pick_right[1][7] = {{PoleSpeed, 2.5, -0.35, -90, 0, 0, 0}};

	float pick_right_to_center_4_servo[1][7] = {{4, 0, 0.4, -90, 0, 0, 0}};
	float pick_right_to_center_4[1][7] = {{5, -2.1, 0.8, -90, 0, 0, 0}};
	float pick_right_to_center_3[1][7] = {{5, -3.75, 0.8, -90, 0, 0, 0}};
	float pick_right_to_center[1][7] = {{5, -4.76, 0.8, -90, 0, 0, 0}};
	float pick_right_to_center_2[1][7] = {{5, -5.76, 0.8, -90, 0, 0, 0}};


	float center_3_to_pick_right[1][7] = {{PoleSpeed, 6, -0.25, -90, 0, 0, 0}};
	float center_to_pick_right[1][7] = {{PoleSpeed, 7, -0.25, -90, 0, 0, 0}};
	float center_2_to_pick_right[1][7] = {{PoleSpeed, 8.5, -0.25, -90, 0, 0, 0}};
	float center_1_to_pick_right[1][7] = {{PoleSpeed, 10, -0.25, -90, 0, 0, 0}};

	float robot_center[1][7] = {{2.0, pp.real_x, pp.real_y + 0.1, -90, 0, 0, 0}};

	switch(pose) // Target Pose
	{
		case UPPER_LEFT:
			switch(lidar->pos) // Self Pose
			{
				case PICK_LEFT:
					lidar->pos = UPPER_LEFT;
					break;

				case PICK_RIGHT:
					lidar->pos = UPPER_LEFT;
					break;

				case CENTER:
					lidar->pos = UPPER_LEFT;
					break;

				case UPPER_RIGHT:
					lidar->pos = UPPER_LEFT;
					break;

				default:
					break;

			}
			break;

		case CENTER_1:

			switch(lidar->pos) // Self Pose
			{
				case PICK_LEFT:
					lidar->fail = 0;
					stick_fence = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;
					ResetCoordinate();
					lidar->pos = CENTER_1;


//					PP_start(pick_left_adjust_servo, 1, &pp);
//					while(pp.pp_start)
//					{
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							PP_stop(&pp);
//						}
//					}
					PP_start(pick_left_adjust, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					// Stick to fence
//					PP_start(pick_left_to_center_1, 1, &pp);
//					while(pp.pp_start)
//					{
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							PP_stop(&pp);
//							lidar->fail = 1;
////							lidar->pos = PICK_LEFT;
////							lidar->pos_counter = PICK_LEFT;
//						}
//
//						if(In_LS_Shot_1 || In_LS_Shot_2)
//							PP_stop(&pp);
//					}

//					Shot();
					stick_fence = 1;

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case PICK_RIGHT:
					lidar->pos = CENTER_1;
					break;

				case UPPER_LEFT:
					lidar->pos = CENTER_1;
					break;

				case UPPER_RIGHT:
					lidar->pos = CENTER_1;
					break;

				case CENTER_2:
					lidar->fail = 0;
					stick_fence = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_1;
						PP_start(center_2_to_center_1, 1, &pp);
						while(pp.pp_start)
						{
							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_2_to_center_1[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_2; // PP Failed
	//							lidar->pos_counter = CENTER_2;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_1;
						LSR_start(lsr_center_1, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist <= lsr_center_1[0][1] + Offset)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;

					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER:
					stick_fence = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;
					lidar->fail = 0;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_1;
						PP_start(center_to_center_1, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER; // PP Failed
	//							lidar->pos_counter = CENTER;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_to_center_1[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_1;
						LSR_start(lsr_center_1, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist <= lsr_center_1[0][1] + Offset_2)
								PP_stop(&pp);
						}

					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER_3:
					vesc_duty = type1Duty;
					vesc_speed = type1;
					lidar->pos = CENTER_1;
					LSR_start(lsr_center_1, 1, &pp, 0, 0);
					while(pp.lsr_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(lsrL.dist <= lsr_center_1[0][1] + Offset_3 - 0.1)
							PP_stop(&pp);
					}


					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;

					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}
					break;

				case CENTER_4:
					vesc_duty = type1Duty;
					vesc_speed = type1;
					lidar->pos = CENTER_1;
					LSR_start(lsr_center_1, 1, &pp, 0, 0);
					while(pp.lsr_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(lsrL.dist <= lsr_center_1[0][1] + Offset_4)
							PP_stop(&pp);
					}


					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;

					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}
					break;

				default:
					break;
			}
			break;

		case CENTER_2:
			switch(lidar->pos) // Self Pose
			{
				case PICK_LEFT:
					lidar->pos = CENTER_2;
					break;

				case PICK_RIGHT:
					stick_fence = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}
					ResetCoordinate();
					lidar->pos = CENTER_2;

					PP_start(pick_right_to_center_4_servo, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					LoadRing();
//					close_servo;

					PP_start(pick_right_to_center_2, 1, &pp);

					load_adjust = 1;
					AdjustRings();
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
//							lidar->pos = PICK_RIGHT;
//							lidar->pos_counter = PICK_RIGHT;
						}

						if(In_LS_Shot_1 || In_LS_Shot_2)
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}

						if(pp.real_x <= pick_right_to_center_2[0][1])
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}

						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							if(blue)
							{
								vesc_speed = BlueOppoType2;
								vesc_duty = BlueOppoType2Duty;
							}
							else
							{
								vesc_speed = RedOppoType2;
								vesc_duty = RedOppoType2Duty;
							}
						}
					}

					adjust_servo;

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 1000)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					// Only adjust after reached destination
//					if(lidar->pos == pose)
					LidarAdjust(lidar);

					break;

				case UPPER_LEFT:
					lidar->pos = CENTER_2;
					break;

				case UPPER_RIGHT:
					lidar->pos = CENTER_2;
					break;

				case CENTER_1:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}
					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_2;
						PP_start(center_1_to_center_2, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_1;
	//							lidar->pos_counter = CENTER_1;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_1_to_center_2[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_2;
						LSR_start(lsr_center_2, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist >= lsr_center_2[0][1] - Offset + 0.05)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_2;
						PP_start(center_to_center_2, 1, &pp);
						while(pp.pp_start)
						{
							if(In_LS_Shot_1 || In_LS_Shot_2)
								pp.error_y = 0;

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER;
	//							lidar->pos_counter = CENTER;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_to_center_2[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_2;
						LSR_start(lsr_center_2, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist <= lsr_center_2[0][1] + Offset - 0.05)
								PP_stop(&pp);
						}

					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER_3:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_2;
						PP_start(center_3_to_center_2, 1, &pp);
						while(pp.pp_start)
						{
							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_3_to_center_2[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_3; // PP Failed
	//							lidar->pos_counter = CENTER_3;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_2;
						LSR_start(lsr_center_2, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist <= lsr_center_2[0][1] + Offset_2 - 0.4)
								PP_stop(&pp);
						}
					}
					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 750)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);

					break;

				case CENTER_4:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}

					lidar->pos = CENTER_2;
					LSR_start(lsr_center_2, 1, &pp, 0, 0);
					while(pp.lsr_start)
					{
						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							if(blue)
							{
								vesc_speed = BlueOppoType2;
								vesc_duty = BlueOppoType2Duty;
							}
							else
							{
								vesc_speed = RedOppoType2;
								vesc_duty = RedOppoType2Duty;
							}
						}

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(lsrL.dist <= lsr_center_2[0][1] + Offset_3 - 0.5)
							PP_stop(&pp);
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 750)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}
					break;

				default:
					break;
			}
			break;

		case CENTER:
			switch(lidar->pos) // Self Pose
			{
				case PICK_LEFT:
					lidar->pos = CENTER;
					break;

				case PICK_RIGHT:
					stick_fence = 0;
					if(blue)
					{
						vesc_speed = BlueType3;
						vesc_duty = BlueType3Duty;
					}
					else
					{
						vesc_speed = RedType3;
						vesc_duty = RedType3Duty;
					}
					ResetCoordinate();
					lidar->pos = CENTER;

					PP_start(pick_right_to_center_4_servo, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					LoadRing();
//					close_servo;

					PP_start(pick_right_to_center, 1, &pp);

					load_adjust = 1;
					AdjustRings();
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
//							lidar->pos = PICK_RIGHT;
//							lidar->pos_counter = PICK_RIGHT;
						}

						if(In_LS_Shot_1 || In_LS_Shot_2)
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}

						if(pp.real_x <= -4.6)
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}
					}

					adjust_servo;

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 1000)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					// Only adjust after reached destination
//					if(lidar->pos == pose)
					LidarAdjust(lidar);

					break;

				case UPPER_LEFT:
					lidar->pos = CENTER;
					break;

				case UPPER_RIGHT:
					lidar->pos = CENTER;
					break;

				case CENTER_1:
					stick_fence = 0;
					vesc_duty = type1Duty;
					lidar->fail = 0;
					vesc_speed = type1;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER;
						PP_start(center_1_to_center, 1, &pp);
						while(pp.pp_start)
						{
							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_1_to_center[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_1; // PP Failed
	//							lidar->pos_counter = CENTER_1;
							}
						}
					}
					else
					{
						lidar->pos = CENTER;
						LSR_start(lsr_center, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueType3;
									vesc_duty = BlueType3Duty;
								}
								else
								{
									vesc_speed = RedType3;
									vesc_duty = RedType3Duty;
								}
							}
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist >= lsr_center[0][1] - Offset_2 - 0.1)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;


				case CENTER_2:
					stick_fence = 0;
					vesc_duty = type1Duty;
					lidar->fail = 0;
					vesc_speed = type1;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER;
						PP_start(center_2_to_center, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_2;
	//							lidar->pos_counter = CENTER_2;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_2_to_center[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER;
						LSR_start(lsr_center, 1, &pp, 0, 0);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrL.dist >= lsr_center[0][1] - Offset + 0.15)
								PP_stop(&pp);

							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueType3;
									vesc_duty = BlueType3Duty;
								}
								else
								{
									vesc_speed = RedType3;
									vesc_duty = RedType3Duty;
								}
							}
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER_3:
					stick_fence = 0;
					vesc_duty = type1Duty;
					lidar->fail = 0;
					vesc_speed = type1;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER;
						PP_start(center_3_to_center, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_3;
	//							lidar->pos_counter = CENTER_3;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_3_to_center[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueType3;
									vesc_duty = BlueType3Duty;
								}
								else
								{
									vesc_speed = RedType3;
									vesc_duty = RedType3Duty;
								}
							}
						}
					}
					else
					{
						lidar->pos = CENTER;
						LSR_start(lsr_center_right, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist >= lsr_center_right[0][1] - Offset + 0.25)
								PP_stop(&pp);

							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueType3;
									vesc_duty = BlueType3Duty;
								}
								else
								{
									vesc_speed = RedType3;
									vesc_duty = RedType3Duty;
								}
							}
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER_4:
					stick_fence = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;
					lidar->fail = 0;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER;
						PP_start(center_4_to_center, 1, &pp);
						while(pp.pp_start)
						{
							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_4_to_center[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_4; // PP Failed
	//							lidar->pos_counter = CENTER_4;
							}
						}
					}
					else
					{
						lidar->pos = CENTER;
						LSR_start(lsr_center_right, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist >= lsr_center_right[0][1] - Offset_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
								PP_stop(&pp);
							}

							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueType3;
									vesc_duty = BlueType3Duty;
								}
								else
								{
									vesc_speed = RedType3;
									vesc_duty = RedType3Duty;
								}
							}
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				default:
					break;
			}
			break;

		case CENTER_3:
			switch(lidar->pos) // Self Pose
			{
				case PICK_LEFT:
					lidar->pos = CENTER_3;
					break;

				case PICK_RIGHT:
					stick_fence = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}
					ResetCoordinate();
					lidar->pos = CENTER_3;

					PP_start(pick_right_to_center_4_servo, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					LoadRing();
//					close_servo;

					PP_start(pick_right_to_center_3, 1, &pp);

					load_adjust = 1;
					AdjustRings();
					while(pp.pp_start)
					{
						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							if(blue)
							{
								vesc_speed = BlueOppoType2;
								vesc_duty = BlueOppoType2Duty;
							}
							else
							{
								vesc_speed = RedOppoType2;
								vesc_duty = RedOppoType2Duty;
							}
						}

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
//							lidar->pos = PICK_RIGHT;
//							lidar->pos_counter = PICK_RIGHT;
						}

						if(In_LS_Shot_1 || In_LS_Shot_2)
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}

						if(pp.real_x <= pick_right_to_center_3[0][1])
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}
					}

					adjust_servo;

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 1000)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					// Only adjust after reached destination
//					if(lidar->pos == pose)
					LidarAdjust(lidar);

					break;

				case UPPER_LEFT:
					lidar->pos = CENTER_3;
					break;

				case UPPER_RIGHT:
					lidar->pos = CENTER_3;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2Duty;
					}
					ResetCoordinate();
					setOrientationMODN(OPERATOR_TURNED_0_DEGREE);
					PP_start(upper_right_to_center_3, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(pp.real_y <= -2.7)
							pp.target_vel[0] = 2.0;
					}

					PP_start(upper_right_to_center_3_2, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(In_LS_Shot_1 || In_LS_Shot_2)
							pp.error_y = 0;
					}

//					PP_start(bang_fence, 1, &pp);
//					while(pp.pp_start)
//					{
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							PP_stop(&pp);
//						}
//
//						if(In_LS_Shot_1 || In_LS_Shot_2)
//							PP_stop(&pp);
//					}

					AdjustRings();
					adjust_servo;

					LidarAdjust(lidar);

					break;

				case CENTER_1:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}

					lidar->pos = CENTER_3;
					LSR_start(lsr_center_3, 1, &pp, 0, 1);
					while(pp.lsr_start)
					{
						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							if(blue)
							{
								vesc_speed = BlueOppoType2;
								vesc_duty = BlueOppoType2Duty;
							}
							else
							{
								vesc_speed = RedOppoType2;
								vesc_duty = RedOppoType2Duty;
							}
						}

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(lsrR.dist <= lsr_center_3[0][1] + Offset_3 - 0.45)
							PP_stop(&pp);
					}


					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					break;

				case CENTER:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_3;
						PP_start(center_to_center_3, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								StopAutoPP();
								lidar->fail = 1;
	//							lidar->pos = CENTER;
	//							lidar->pos_counter = CENTER;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_to_center_3[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_3;
						LSR_start(lsr_center_3, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist <= lsr_center_3[0][1] + Offset - 0.2)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case CENTER_2:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_3;
						PP_start(center_2_to_center_3, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_2; // PP Failed
	//							lidar->pos_counter = CENTER_2;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_2_to_center_3[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_3;
						LSR_start(lsr_center_3, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist <= lsr_center_3[0][1] + Offset_2 - 0.4)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(after - before >= 1000)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}
					stick_fence = 1;

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;


				case CENTER_4:
					stick_fence = 0;
					lidar->fail = 0;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2Duty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2Duty;
					}
					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_3;
						PP_start(center_4_to_center_3, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_4;
	//							lidar->pos_counter = CENTER_4;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x <= center_4_to_center_3[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_3;
						LSR_start(lsr_center_3, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == UP)
							{
								while(ps4.button == UP);
								if(blue)
								{
									vesc_speed = BlueOppoType2;
									vesc_duty = BlueOppoType2Duty;
								}
								else
								{
									vesc_speed = RedOppoType2;
									vesc_duty = RedOppoType2Duty;
								}
							}
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist >= lsr_center_3[0][1] - Offset - 0.05)
								PP_stop(&pp);
						}
					}

					// Robot will slanted to right after path plan, need to fix the slanted right problem
					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z <= -89 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;
				default:
					break;
			}
			break;

		case CENTER_4:
			switch(lidar->pos) // Self Pose
			{
				case PICK_LEFT:
					lidar->pos = CENTER_4;
					break;

				case CENTER_1:
					stick_fence = 0;
					lidar->fail = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;

					lidar->pos = CENTER_4;
					LSR_start(lsr_center_4, 1, &pp, 0, 1);
					while(pp.lsr_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(lsrR.dist <= lsr_center_4[0][1] + Offset_4 + 0.08)
							PP_stop(&pp);
					}


					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}
					break;

				case CENTER_2:
					stick_fence = 0;
					lidar->fail = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;

					lidar->pos = CENTER_4;
					LSR_start(lsr_center_4, 1, &pp, 0, 1);
					while(pp.lsr_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(lsrR.dist <= lsr_center_4[0][1] + Offset_3 + 0.1)
							PP_stop(&pp);
					}


					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					break;

				case CENTER:
					stick_fence = 0;
					lidar->fail = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_4;
						PP_start(center_to_center_4, 1, &pp);
						while(pp.pp_start)
						{
							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_to_center_4[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER; // PP Failed
	//							lidar->pos_counter = CENTER;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_4;
						LSR_start(lsr_center_4, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist <= lsr_center_4[0][1] + Offset_2 + 0.1)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				case PICK_RIGHT:
					stick_fence = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;
					ResetCoordinate();
					lidar->pos = CENTER_4;

					PP_start(pick_right_to_center_4_servo, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
//							lidar->pos = PICK_RIGHT;
//							lidar->pos_counter = PICK_RIGHT;
						}
					}
					LoadRing();
//					close_servo;

					PP_start(pick_right_to_center_4, 1, &pp);
//
					load_adjust = 1;
					AdjustRings();
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
//							lidar->pos = PICK_RIGHT;
//							lidar->pos_counter = PICK_RIGHT;
						}

						if(In_LS_Shot_1 || In_LS_Shot_2)
							pp.error_y = 0;

						if(pp.real_x <= -1.9)
						{
							pp.target_y[0] = pp.real_y;
							pp.error_y = 0;
						}
					}

//					PP_start(bang_fence, 1, &pp);
//					while(pp.pp_start)
//					{
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							PP_stop(&pp);
//						}
//
//						if(In_LS_Shot_1 || In_LS_Shot_2)
//							PP_stop(&pp);
//					}
//
//					for(int i = 0; i < 4; i++)
//					{
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							break;
//						}
//
//						load_adjust_servo;
//						osDelay(100);
//						close_servo;
//						osDelay(100);
//					}
					adjust_servo;

					//					osDelay(100);
//					PP_start(robot_center, 1, &pp);
//					while(pp.pp_start)
//					{
//						pp.target_y[0] = pp.real_y;
//						pp.error_y = 0;
//						pp.target_x[0] = pp.real_x;
//						pp.error_x = 0;
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							PP_stop(&pp);
//						}
//					}
					stick_fence = 1;
					// Only adjust after reached destination
//					if(lidar->pos == pose)
					LidarAdjust(lidar);
					break;

				case UPPER_LEFT:
					lidar->pos = CENTER_4;
					break;

				case UPPER_RIGHT:
					lidar->pos = CENTER_4;
					setOrientationMODN(OPERATOR_TURNED_0_DEGREE);
					vesc_duty = type1Duty;
					vesc_speed = type1;
					ResetCoordinate();
					PP_start(upper_right_to_center_4, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(pp.real_y <= -3.1)
						{
							pp.target_vel[0] = 2.0;
						}
					}

					PP_start(upper_right_to_center_4_2, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(In_LS_Shot_1 || In_LS_Shot_2)
							pp.error_y = 0;
					}

//					PP_start(bang_fence, 1, &pp);
//					while(pp.pp_start)
//					{
//						if(ps4.button == SQUARE)
//						{
//							while(ps4.button == SQUARE);
//							PP_stop(&pp);
//						}
//
//						if(In_LS_Shot_1 || In_LS_Shot_2)
//							PP_stop(&pp);
//					}

					LidarAdjust(lidar);

					AdjustRings();
					adjust_servo;

					break;

				case CENTER_3:
					stick_fence = 0;
					lidar->fail = 0;
					vesc_duty = type1Duty;
					vesc_speed = type1;

					if(!lidar->laser)
					{
						ResetCoordinate();
						lidar->pos = CENTER_4;
						PP_start(center_3_to_center_4, 1, &pp);
						while(pp.pp_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
								lidar->fail = 1;
	//							lidar->pos = CENTER_3;
	//							lidar->pos_counter = CENTER_3;
							}

							if(In_LS_Shot_1 || In_LS_Shot_2)
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}

							if(pp.real_x >= center_3_to_center_4[0][1])
							{
								pp.target_y[0] = pp.real_y;
								pp.error_y = 0;
							}
						}
					}
					else
					{
						lidar->pos = CENTER_4;
						LSR_start(lsr_center_4, 1, &pp, 0, 1);
						while(pp.lsr_start)
						{
							if(ps4.button == SQUARE)
							{
								while(ps4.button == SQUARE);
								PP_stop(&pp);
							}

							if(lsrR.dist <= lsr_center_4[0][1] + Offset + 0.05)
								PP_stop(&pp);
						}
					}

					PP_start(robot_center, 1, &pp);
					before = HAL_GetTick();
					while(pp.pp_start)
					{
						after = HAL_GetTick();

						if(pp.real_z >= -91 && (after - before) >= 700)
							PP_stop(&pp);

						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}

					stick_fence = 1;
					if(lidar->autoshot)
					{
						osDelay(300);
						Shot();
					}

					// Only adjust after reached destination
					if(!lidar->fail)
						LidarAdjust(lidar);
					break;

				default:
					break;
			}
			break;


		case UPPER_RIGHT:
			switch(lidar->pos) // Self Pose
			{
				default:
					break;
			}
			break;

		case PICK_RIGHT:
			switch(lidar->pos)
			{
				case UPPER_RIGHT:
					lidar->pos = PICK_RIGHT;
					ResetCoordinate();
					setOrientationMODN(OPERATOR_TURNED_0_DEGREE);
					loaded = 0;
					vesc_duty = 0.0;
					vesc_speed = 0;
					setPick(1000);
					open_servo;
					cylinder_retract;
					PP_start(upper_right_to_pick_right, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if(pp.real_y <= -2.5)
							pp.target_vel[0] = 1.2;
					}
					pick_right = 1;
					break;

				case CENTER_1:
					lidar->pos = PICK_RIGHT;
					ResetCoordinate();
					loaded = 0;
					vesc_duty = 0.0;
					vesc_speed = 0;
					setPick(1000);
					open_servo;
					cylinder_retract;
					PP_start(center_1_to_pick_right, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if((In_LS_Left_1 || In_LS_Left_2) && lsrR.dist <= 1.5)
							PP_stop(&pp);

						if(lsrR.dist <= 2.7)
							pp.target_vel[0] = 0.5;

						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							go_type_3++;
						}
					}
					pick_right = 1;
					break;

				case CENTER_2:
					lidar->pos = PICK_RIGHT;
					ResetCoordinate();
					loaded = 0;
					vesc_duty = 0.0;
					vesc_speed = 0;
					setPick(1000);
					open_servo;
					cylinder_retract;
					PP_start(center_2_to_pick_right, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if((In_LS_Left_1 || In_LS_Left_2) && lsrR.dist <= 1.5)
							PP_stop(&pp);

						if(lsrR.dist <= 2.6)
							pp.target_vel[0] = 0.5;

						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							go_type_3++;
						}
					}
					pick_right = 1;
					break;

				case CENTER:
					lidar->pos = PICK_RIGHT;
					ResetCoordinate();
					loaded = 0;
					vesc_duty = 0.0;
					vesc_speed = 0;
					setPick(1000);
					open_servo;
					cylinder_retract;
					PP_start(center_to_pick_right, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if((In_LS_Left_1 || In_LS_Left_2) && lsrR.dist <= 1.5)
							PP_stop(&pp);

						if(lsrR.dist <= 2.4)
							pp.target_vel[0] = 0.5;

						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							go_type_3++;
						}
					}
					pick_right = 1;
					break;

				case CENTER_3:
					lidar->pos =  PICK_RIGHT;
					ResetCoordinate();
					loaded = 0;
					vesc_duty = 0.0;
					vesc_speed = 0;
					setPick(1000);
					open_servo;
					cylinder_retract;
					PP_start(center_3_to_pick_right, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if((In_LS_Left_1 || In_LS_Left_2) && lsrR.dist <= 1.5)
							PP_stop(&pp);

						if(lsrR.dist <= 2.15)
							pp.target_vel[0] = 0.5;

						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							go_type_3++;
						}
					}
					pick_right = 1;

					break;

				case CENTER_4:
					lidar->pos = PICK_RIGHT;
					ResetCoordinate();
					loaded = 0;
					vesc_duty = 0.0;
					vesc_speed = 0;
					setPick(1000);
					open_servo;
					cylinder_retract;
					PP_start(center_4_to_pick_right, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}

						if((In_LS_Left_1 || In_LS_Left_2) && lsrR.dist <= 1.0)
							PP_stop(&pp);

						if(lsrR.dist <= 1.7)
							pp.target_vel[0] = 0.5;

						if(ps4.button == UP)
						{
							while(ps4.button == UP);
							go_type_3++;
						}
					}

					pick_right = 1;
					break;

				default:
					break;
			}

		default:
			break;
		}
}

/*
 * Most Ideal Case
 * In current code, A->B->C->D turn in anti-clockwise, aiming from self_Type_1 -> self_Type_2 -> Opponent_Type_2 -> Opponent_Type_1
 * The available range for poles may differ based on the position
 *
 * Current situation is not able to shoot opponent Type 1, 2
 * Initial angle will be B (-90 degree)
 * After pick rings from left, directly go to aim self Type-2(B), Type-1-Left(A), Type-1-Right(C)
 * Then go to right self Type-2 repeat again
 */
void LidarSetAngle(Angle_t angle, Lidar_t* lidar)
{
	float angle_BL[1][7] = {{1.0, 0, 0, -113.41, 0, 0, 0}};
	float angle_FL[1][7] = {{1.0, 0, 0, -103.26, 0, 0, 0}};
	float angle_S[1][7] = {{1.0, 0, 0, -91, 0, 0, 0}};
	float angle_FR[1][7] = {{1.0, 0, 0, -80.28, 0, 0, 0}};
	float angle_BR[1][7] = {{1.0, 0, 0, -70.85, 0, 0, 0}};

	switch(angle) // Target Angle
	{
		case BL:
			switch(lidar->angle) // Current Angle
			{
				case FL:
					lidar->angle = BL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();

					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case S:
					lidar->angle = BL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case FR:
					lidar->angle = BL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case BR:
					lidar->angle = BL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				default:
					break;
			}
			break;

		case FL:
			switch(lidar->angle) // Current Angle
			{
				case BL:
					lidar->angle = FL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();

					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case S:
					lidar->angle = FL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case FR:
					lidar->angle = FL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case BR:
					lidar->angle = FL;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FL, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				default:
					break;
			}
			break;

		case S:
			switch(lidar->angle)
			{
				case FL:
					lidar->angle = S;
					if(blue)
					{
						vesc_speed = BlueType3;
						vesc_duty = BlueType3RotateDuty;
					}
					else
					{
						vesc_speed = RedType3;
						vesc_duty = RedType3RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_S, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case BL:
					lidar->angle = S;
					if(blue)
					{
						vesc_speed = BlueType3;
						vesc_duty = BlueType3RotateDuty;
					}
					else
					{
						vesc_speed = RedType3;
						vesc_duty = RedType3RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_S, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case FR:
					lidar->angle = S;
					if(blue)
					{
						vesc_speed = BlueType3;
						vesc_duty = BlueType3RotateDuty;
					}
					else
					{
						vesc_speed = RedType3;
						vesc_duty = RedType3RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_S, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case BR:
					lidar->angle = S;
					if(blue)
					{
						vesc_speed = BlueType3;
						vesc_duty = BlueType3RotateDuty;
					}
					else
					{
						vesc_speed = RedType3;
						vesc_duty = RedType3RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_S, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				default:
					break;
			}
			break;

		case FR:
			switch(lidar->angle)
			{
				case BL:
					lidar->angle = FR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case FL:
					lidar->angle = FR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case S:
					lidar->angle = FR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case BR:
					lidar->angle = FR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueOppoType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedOppoType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_FR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				default:
					break;
			}
			break;

		case BR:
			switch(lidar->angle)
			{
				case FL:
					lidar->angle = BR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case BL:
					lidar->angle = BR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case S:
					lidar->angle = BR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				case FR:
					lidar->angle = BR;
					if(blue)
					{
						vesc_speed = BlueType2;
						vesc_duty = BlueType2RotateDuty;
					}
					else
					{
						vesc_speed = RedType2;
						vesc_duty = RedType2RotateDuty;
					}
					ResetCoordinate();
					PP_PIDZSet(0.75, 0.05, 0.2, 6.5, &pp);
					PP_start(angle_BR, 1, &pp);
					while(pp.pp_start)
					{
						if(ps4.button == SQUARE)
						{
							while(ps4.button == SQUARE);
							PP_stop(&pp);
						}
					}
					PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
					break;

				default:
					break;
			}
			break;

		default:
			break;
	}
}

void LidarAdjust(Lidar_t* lidar)
{
	if(lidar->AdjEnb)
	{
		// Ensure robot is straight
//		float robot_straight[1][7] = {{1.0, 0, 0, -90, 0, 0, 0}};
//		PP_start(robot_straight, 1, &pp);
//		while(pp.pp_start)
//		{
//			if(ps4.button == SQUARE)
//			{
//				while(ps4.button == SQUARE);
//				PP_stop(&pp);
//			}
//		}

		// Ensure robot sticking fence
		float stick_fence_point[1][7] = {{2.0, 0, 5, -90, 0, 0, 0}};
		PP_start(stick_fence_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
			}

			if(In_LS_Shot_1 || In_LS_Shot_2)
				PP_stop(&pp);
		}

		// Only adjust after reached destination
		switch(lidar->pos)
		{
			case UPPER_LEFT:
				/* Send command to lidar to obtain nearest pole
				 * Compare with preset values
				 * Path plan to desired position
				 */
				break;

			case CENTER_1:
				LidarSendIns(NEAR, lidar);
				lidar->adj_x = lidar->pole.x - lidar->center_1_offset_x;
				lidar->adj_y = lidar->pole.y - lidar->center_1_offset_y;
				LidarAdjustPP(lidar->adj_x, lidar->adj_y, -90.0);
				break;

			case CENTER_2:
				LidarSendIns(NEAR, lidar);
				lidar->adj_x = lidar->pole.x - lidar->center_2_offset_x;
				lidar->adj_y = lidar->pole.y - lidar->center_2_offset_y;
				LidarAdjustPP(lidar->adj_x, lidar->adj_y, -90.0);
				break;

			case CENTER:
				LidarSendIns(NEAR, lidar);
				lidar->adj_x = lidar->pole.x - lidar->center_offset_x;
				lidar->adj_y = lidar->pole.y - lidar->center_offset_y;
				LidarAdjustPP(lidar->adj_x, lidar->adj_y, -90.0);
				break;

			case CENTER_3:
				LidarSendIns(NEAR, lidar);
				lidar->adj_x = lidar->pole.x - lidar->center_3_offset_x;
				lidar->adj_y = lidar->pole.y - lidar->center_3_offset_y;
				LidarAdjustPP(lidar->adj_x, lidar->adj_y, -90.0);
				break;

			case CENTER_4:
				LidarSendIns(NEAR, lidar);
				lidar->adj_x = lidar->pole.x - lidar->center_4_offset_x;
				lidar->adj_y = lidar->pole.y - lidar->center_4_offset_y;
				LidarAdjustPP(lidar->adj_x, lidar->adj_y, -90.0);
				break;

			case UPPER_RIGHT:
				break;

			default:
				break;
		}
	}
}

void LidarAdjustPP(float adj_x, float adj_y, float z)
{
	ResetCoordinate();

	float adj_point[1][7] = {{1.5, adj_x, adj_y, z, 0, 1, 0}};
	PP_start(adj_point, 1, &pp);

	while(pp.pp_start)
	{
		if(ps4.button == SQUARE)
		{
			while(ps4.button == SQUARE);
			PP_stop(&pp);
		}

		if(In_LS_Shot_1 || In_LS_Shot_2)
		{
			pp.error_y = 0;
			pp.target_y[0] = pp.real_y;
		}
	}
}

/* Call this inside a task
 *   <- -> Will be used to change position
 *   UP DOWN will be used to change angle
 */
void LidarControl(Lidar_t* lidar)
{
	uint32_t before, after;

	if(angle_shoot)
	{
		if(ps4.button == LEFT)
		{
			before = HAL_GetTick();
			while(ps4.button == LEFT)
			{
				after = HAL_GetTick();
			}
			if(after - before > 500)
				lidar->angle_counter -= 3;
			else
				lidar->angle_counter--;
		}

		if(ps4.button == RIGHT)
		{
			before = HAL_GetTick();
			while(ps4.button == RIGHT)
			{
				after = HAL_GetTick();
			}
			if(after - before > 500)
				lidar->angle_counter += 3;
			else
				lidar->angle_counter++;
		}

		if(ps4.button == UP)
		{
			before = HAL_GetTick();
			while(ps4.button == UP)
			{
				after = HAL_GetTick();
			}
			if(after - before > 500)
				lidar->angle_counter += 4;
			else
				lidar->angle_counter += 2;
		}

		if(ps4.button == DOWN)
		{
			before = HAL_GetTick();
			while(ps4.button == DOWN)
			{
				after = HAL_GetTick();
			}
			if(after - before > 500)
				lidar->angle_counter -= 4;
			else
				lidar->angle_counter -= 2;
		}
	}
	else
	{
		if(ps4.button == LEFT)
		{
			before = HAL_GetTick();
			while(ps4.button == LEFT)
			{
				after = HAL_GetTick();
			}
			shot_count = 0;

			if(after - before > 500)
				lidar->pos_counter -= 3;
			else
				lidar->pos_counter--;
		}

		if(ps4.button == RIGHT)
		{
			before = HAL_GetTick();
			while(ps4.button == RIGHT)
			{
				after = HAL_GetTick();
			}

			if(after - before > 500)
				lidar->pos_counter += 3;
			else
				lidar->pos_counter++;
			shot_count = 0;
		}

		if(ps4.button == UP)
		{
			before = HAL_GetTick();
			while(ps4.button == UP)
			{
				after = HAL_GetTick();
			}

			if(after - before > 500)
				lidar->pos_counter += 4;
			else
				lidar->pos_counter += 2;
			shot_count = 0;
		}

		if(ps4.button == DOWN)
		{
			before = HAL_GetTick();
			while(ps4.button == DOWN)
			{
				after = HAL_GetTick();
			}

			if(after - before > 500)
				lidar->pos_counter -= 4;
			else
				lidar->pos_counter -= 2;
			shot_count = 0;
		}
	}
	LidarCheckPos(lidar);
	LidarCheckAngle(lidar);
}

void LidarCheckAngle(Lidar_t* lidar)
{
	switch(lidar->angle_counter)
	{
		case BL:
			LidarSetAngle(BL, lidar);
			break;

		case FL:
			LidarSetAngle(FL, lidar);
			break;

		case S:
			LidarSetAngle(S, lidar);
			break;

		case FR:
			LidarSetAngle(FR, lidar);
			break;

		case BR:
			LidarSetAngle(BR, lidar);
			break;

		default:
			break;
	}
}

void LidarCheckPos(Lidar_t* lidar)
{
	switch(lidar->pos_counter)
	{
		case PICK_RIGHT:
			LidarSetPos(PICK_RIGHT, lidar);
			break;

		case UPPER_LEFT:
			LidarSetPos(UPPER_LEFT, lidar);
			break;

		case CENTER_1:
			LidarSetPos(CENTER_1, lidar);
			break;

		case CENTER_2:
			LidarSetPos(CENTER_2, lidar);
			break;

		case CENTER:
			LidarSetPos(CENTER, lidar);
			break;

		case CENTER_3:
			LidarSetPos(CENTER_3, lidar);
			break;

		case CENTER_4:
			LidarSetPos(CENTER_4, lidar);
			break;

		case UPPER_RIGHT:
			LidarSetPos(UPPER_RIGHT, lidar);
			break;

		default:
			break;
	}
}

// Send Instruction to pi/pc
void LidarSendIns(Instruction_t ins, Lidar_t* lidar)
{
	lidar->response = NO; // After user got the data, response will be OK
	lidar->inst = ins;
	lidar->obstacle_send[0] = 0x01;
	memcpy(&lidar->obstacle_send[1], &lidar->inst, 1);

	HAL_UART_Transmit(lidar->lidar_UART, lidar->obstacle_send, 2, HAL_MAX_DELAY);
	HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 25);

	while(lidar->response == NO) // Poll the user after got new data
	{
		if(ps4.button == SQUARE)
			break;
	}
}

// Call this in UART Callback
void ObstacleHandler(Lidar_t* lidar)
{
	if(lidar->mode == CONTINUOUS)
	{
		static int mode = 0;
		switch(mode)
		{
			case 0:
				if(lidar->obstacle_receive[0] == 0x01)
				{
					memcpy(&lidar->obstacle_count, &lidar->obstacle_receive[1], 4);
					if(lidar->new == 0)
					{
							lidar->Polelist = (Pole*)malloc(lidar->obstacle_count * 3 * sizeof(float));
					}
					else
					{
							lidar->Polelist = (Pole*)realloc(lidar->Polelist, lidar->obstacle_count * 3 * sizeof(float));
					}

					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->obstacle_count, 4, HAL_MAX_DELAY);
					HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, lidar->obstacle_count * 3 * 4);
					mode = 1;
					lidar->new = 1;
				}
				break;

			case 1:

				for(int i = 0; i < lidar->obstacle_count; i++)
				{
					memcpy(&lidar->pole.x, &lidar->obstacle_receive[i * 12], 4);
					memcpy(&lidar->pole.y, &lidar->obstacle_receive[i * 12 + 4], 4);
					memcpy(&lidar->pole.distance, &lidar->obstacle_receive[i * 12 + 8], 4);

					lidar->pole.angle = atanf(lidar->pole.y / lidar->pole.x);
					lidar->Polelist[i] = lidar->pole;
				}

				// For checking purpose
				for(int i = 0; i < lidar->obstacle_count; i++)
				{
					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->Polelist[i].x, 4, HAL_MAX_DELAY);
					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->Polelist[i].y, 4, HAL_MAX_DELAY);
					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->Polelist[i].distance, 4, HAL_MAX_DELAY);
				}

				mode = 0;
				HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 5);
				break;
		}
	}
	else // Discontinuous mode
	{
		if(lidar->start == 0) // Start booting
		{
			if(lidar->obstacle_receive[0] == 0x01)
			{
				memcpy(&lidar->res, &lidar->obstacle_receive[1], 1);
				lidar->response = lidar->res;

				if(lidar->response == OK)
				{
					HAL_UART_Transmit(lidar->lidar_UART, &lidar->res, 1, HAL_MAX_DELAY);
					lidar->start = 1;
					lidar->response = NO;
					HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 25);
				}
			}
		}
		else
		{
			memcpy(&lidar->res, &lidar->obstacle_receive[0], 1);
			lidar->response = lidar->res;

			if(lidar->response == OK)
			{
				// Ready to use this data
				memcpy(&lidar->pole.x, &lidar->obstacle_receive[1], 8);
				memcpy(&lidar->pole.y, &lidar->obstacle_receive[9], 8);
				memcpy(&lidar->pole.distance, &lidar->obstacle_receive[17], 8);

				// Convert from ROS coordinate to RBC coordinate
				float temp = lidar->pole.x;
				lidar->pole.x = -lidar->pole.y;
				lidar->pole.y = temp;

				lidar->pole.angle = atanf(lidar->pole.y / lidar->pole.x);
			}
			else
			{
				lidar->reject = 1;
				// Error Handler here
				LidarSendIns(lidar->inst, lidar);
			}

			// Checking Purpose on pc side
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->res, 1, HAL_MAX_DELAY);
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->pole.x, 8, HAL_MAX_DELAY);
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->pole.y, 8, HAL_MAX_DELAY);
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->pole.distance, 8, HAL_MAX_DELAY);
		}
	}
}
