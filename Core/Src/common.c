/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"
#include "math.h"

float pickVG[7] = {1.0, 1.0, 19999.0, 1.0, 1.0, 0.001, 60.0};
float flywheelVG[7] = {1.0, 1.0 / 25.0, 19999, 1.0, 1.5, 0.001, 60};
float flywheelLVG[7] = {1.0, 1.0 / 25.0, 19999, 1.0, 1.5, 0.001, 60};

void set(void) {
	Initialize();
	PSxInitDMA(&ps4, &hi2c1);
	TIMxInit(&htim6, 5000, 84);
	MODNInit(MODN_FWD_OMNI, 5.5, 0.5, 2.0, 0.1);
	LidarInit(&huart4, DISCONTINUOUS, &lidar);
	RNS_config(&hcan1);

	ServoxInit(&servo_ring, &htim5, IP15_PIN, TIM_CHANNEL_2); // BDC7
	ServoInitPulseLimit(&servo_ring, 500, 2500);
	ServoSetPulse(&servo_ring, 2220);

	ServoxInit(&servo_ring_new, &htim5, IP15_PIN, TIM_CHANNEL_1); // BDC8
	ServoInitPulseLimit(&servo_ring_new, 500, 2500);
	ServoSetPulse(&servo_ring_new, 800);

	PPInit(fwd_omni, &fXPos, &fYPos, &fyaw, &pp, &lsrL, &lsry, &lsrR);
//	PPInit(fwd_omni, &fXPos, &fYPos, &fyaw, &pp);
	PP_SetZ(-180, &pp);
	PP_PIDPathSet(1.5, 1.5, 0.5, &pp);
	PP_PIDEndSet(1.0, 1.5, 1.0, &pp);
	PP_PIDZSet(0.75, 0.05, 0.2, 5.5, &pp);
	PP_SetCrv_Points(10, &pp);

	R6091U_Init(&IMU, &huart3);

	PIDSourceInit(&pickErr, &pickU, &pick_PID);
	PIDDelayInit(&pick_PID);
	PIDGainInit(0.005, pickVG[0], pickVG[1], pickVG[2], pickVG[3], pickVG[4], pickVG[5], pickVG[6], &pick_PID);

	PIDSourceInit(&leftErr, &leftU, &left_PID);
	PIDDelayInit(&left_PID);
	PIDGainInit(0.005, flywheelLVG[0], flywheelLVG[1], flywheelLVG[2], flywheelLVG[3], flywheelLVG[4], flywheelLVG[5], flywheelLVG[6], &left_PID);

	PIDSourceInit(&rightErr, &rightU, &right_PID);
	PIDDelayInit(&right_PID);
	PIDGainInit(0.005, flywheelVG[0], flywheelVG[1], flywheelVG[2], flywheelVG[3], flywheelVG[4], flywheelVG[5], flywheelVG[6], &right_PID);

	float fXPosGain[3] = {0.8, 0.4, 0.2};
	float fYPosGain[3] = {0.8, 0.4, 0.2};

	/* X & Y position ABT */
	ABTInit(SAMPLE_TIME, fXPosGain[0], fXPosGain[1], fXPosGain[2], &fXEncData, &fXPos, &fXVel, &fXAcc, &x_data);
	ABTEstimateInit(&x_data);

	ABTInit(SAMPLE_TIME, fYPosGain[0], fYPosGain[1], fYPosGain[2], &fYEncData, &fYPos, &fYVel, &fYAcc, &y_data);
	ABTEstimateInit(&y_data);

	VESCInit(35000, 7, 0.15, 111, &vesc1);
	VESCInit(35000, 7, 0.15, 112, &vesc2);
}

void RNS_config(CAN_HandleTypeDef *hcanx) {
	RNSInit(hcanx, &rns);
	//0b11100101
	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b1000000100, (float) fwd_omni,
			(float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000.0 * 3.142, 2.0,
			0.05 / 4000.0 * 3.142, 2.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, (float) 5816.0 / 5181.0, //remember to put floating point 5175/5174 wrong
	(float) (3.142 * 0.126 * 2 / 581.6));
	RNSSet(&rns, RNS_B_KCD_PTD, (float) 5191.0 / 5183.0,
			(float) (3.142 * 0.126 * 2 / 519.1));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 6.88, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 8.66, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 7.91, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 7.79, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID, 1.0, 1.5, 0.001);   //
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1.0, 1.5, 0.001);   //
	RNSSet(&rns, RNS_B_LEFT_VEL_PID, 1.0, 1.5, 0.001);   //
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 1.0, 1.5, 0.001);   //

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
	RNSSet(&rns, RNS_PPZPID, 0.5, 0.05, 0.2, 8.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0); // Change No. of Points in the Curved Path
}

void manual_mode(void) {

	while (sys.manual) {

		if (ps4.button == R1) {
			setSpeedMODN(4.0);
		} else if (ps4.button == L1) {
			setSpeedMODN(2.0);
		} else {
			setSpeedMODN(3.0);
		}
		realMODN(&ps4, &rns);
	}

}

void RobotStart()
{
	before = 0;
	mode = 1;
	servo_close_once = 0;
	shoot_done = 0;
	shoot_start = 0;
	pick_start = 0;
	Robot_Pitch = PITCH45;
	blue = 1;
	set_pitch = 0;
	picked_left = 0;
	picked_right = 0;
	cylinder_retract;
	set_pick_enc = 0;
	stop_adjust = 0;
	up_done = 0;
	servo_enb = 0;
	pick_tol = 100;
	vel_adjust = 0;
	reload = 0;
	load_stop_once = 0;
	load_start = 0;
	wait_load = 0;
	must_load = 0;
	wheel = 1;
	loaded = 0;
	led_enb = 0;
	pick_enc_buf = 0;
	vesc_speed = 0;
	stick_fence = 0;
	vesc_start = 0;
	pick_left_manual = 0;
	load_adjust = 1;
	shooted = 0;
	adjust_count = 0;
	type_3_done = 0;
	go_type_3 = 0;
	pick_0 = 0;
	cylinder_load_once = 0;
	pick_right_enb = 1;
	shot_count = 0;
	manual_adjust = 0;
	extra_rpm = 0;
	go_A = 0;

	// Parameters
	RedPickLess = 60;
	BluePickLess = 60;
	RedPickMore = 0;
	vesc_extra = 0.0;
	BluePickMore = 0;
}

void NormalControl()
{
	// Cylinder
	if (ps4.button == OPTION)
	{
		while (ps4.button == OPTION);
		ResetPickEnc();
		mode = AUTO;
//		pick_0 = 1;
//		pick_down;
	}

	// Cylinder
	if (ps4.button == TRIANGLE)
	{
		while (ps4.button == TRIANGLE);
//		static int counter = 0;
//		if (counter == 0)
//		{
//			counter++;
//			cylinder_retract; // BDC1
//		}
//		else
//		{
//			cylinder_load;
//			counter = 0;
//		}
//		LidarSendIns(NEAR, &lidar);
//		ResetCoordinate();
		ResetCoordinate();
		lidar.pos = POS_PENDING;
		lidar.pos_counter = POS_PENDING;
		pick_right = 1;
		vesc_speed = 4;
		cylinder_retract;
		mode = AUTO;
	}

	// mode
	if(ps4.button == SQUARE)
	{
		while(ps4.button == SQUARE);
		if(!pp.pp_start)
		{
			static int counter = 0;

			if(counter == 0)
			{
				counter++;
				blue = 0;
			}
			else
			{
				blue = 1;
				counter = 0;
			}
		}
		mode = AUTO;
	}

	if(ps4.button == UP)
	{
		while(ps4.button == UP);

		if(!lidar.autoshot)
			lidar.autoshot = 1;
		else
			lidar.autoshot = 0;

		mode = AUTO;
	}

	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);
		static int counter = 0;

		if(counter == 0)
		{
			lidar.pos = CENTER_4;
			lidar.pos_counter = CENTER_4;
			vesc_duty = type1Duty;
			vesc_speed = type1;
			led7_on;
			mode = AUTO;
			counter++;
		}
		else
		{
			lidar.pos = CENTER_1;
			lidar.pos_counter = CENTER_1;
			led7_off;
			vesc_speed = type1;
			vesc_duty = type1Duty;
			mode = AUTO;
			counter = 0;
		}
	}

	// Apply extra rpm
	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);
		extra_rpm = manual_adjust * 5;

		mode = AUTO;
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		static int counter = 0;
		counter++;

		if(counter == 1)
		{
			cylinder_load;
		}
		else
		{
			cylinder_retract;
			counter = 0;
		}
		mode = AUTO;
	}

	// Adjust for A and B
	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);

		float center_2_to_A[1][7] = {{0, 4.22, pp.real_y + 0.3, -90, 3.5, 0.06, 1}};
		float A_to_center_2[1][7] = {{0, 4.32, pp.real_y + 0.3, -90, 3.5, 0.06, 1}};

		float center_3_to_B[1][7] = {{0, 4.76, pp.real_y + 0.3, -90, 3.5, 0.06, 1}};
		float B_to_center_3[1][7] = {{0, 4.66, pp.real_y + 0.3, -90, 3.5, 0.06, 1}};

		float center_to_three[1][7] = {{0, 5.6, pp.real_y + 0.3, -90, 3.5, 0.06, 1}};
		float three_to_center[1][7] = {{0, 5.82, pp.real_y + 0.3, -90, 3.5, 0.06, 1}};

		if(lidar.pos == CENTER_2)
		{
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

			lidar.pos = A;
			LSR_start(center_2_to_A, 1, &pp, 0, 0);
			while(pp.lsr_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}

				if(lsrL.dist <= center_2_to_A[0][1])
					PP_stop(&pp);
			}

			if(lidar.autoshot)
			{
				osDelay(300);
				Shot();
			}
		}

		else if(lidar.pos == A)
		{
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

			lidar.pos = CENTER_2;
			LSR_start(A_to_center_2, 1, &pp, 0, 0);
			while(pp.lsr_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}

				if(lsrL.dist >= A_to_center_2[0][1])
					PP_stop(&pp);
			}

			if(lidar.autoshot)
			{
				osDelay(300);
				Shot();
			}
		}

		else if(lidar.pos == CENTER_3)
		{
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

			lidar.pos = B;
			LSR_start(center_3_to_B, 1, &pp, 0, 1);
			while(pp.lsr_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}

				if(lsrR.dist >= center_3_to_B[0][1])
					PP_stop(&pp);
			}

			if(lidar.autoshot)
			{
				osDelay(300);
				Shot();
			}
		}

		else if(lidar.pos == B)
		{
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

			lidar.pos = CENTER_3;
			LSR_start(B_to_center_3, 1, &pp, 0, 1);
			while(pp.lsr_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}

				if(lsrR.dist <= B_to_center_3[0][1])
					PP_stop(&pp);
			}

			if(lidar.autoshot)
			{
				osDelay(300);
				Shot();
			}
		}

		else if(lidar.pos == CENTER)
		{
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

			lidar.pos = TYPE_THREE;
			LSR_start(center_to_three, 1, &pp, 0, 0);
			while(pp.lsr_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}

				if(lsrL.dist <= center_to_three[0][1])
					PP_stop(&pp);
			}

			if(lidar.autoshot)
			{
				osDelay(300);
				Shot();
			}
		}

		else if(lidar.pos == TYPE_THREE)
		{
			vesc_speed = type1;
			vesc_duty = type1Duty;

			lidar.pos = CENTER;
			LSR_start(three_to_center, 1, &pp, 0, 1);
			while(pp.lsr_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}

				if(lsrL.dist >= three_to_center[0][1])
					PP_stop(&pp);
			}

			if(lidar.autoshot)
			{
				osDelay(300);
				Shot();
			}
		}

		mode = AUTO;
	}

	if(ps4.button == DOWN)
	{
		while(ps4.button == DOWN);
//		stick_fence = 0;
		if(lidar.pos == A)
			lidar.pos = CENTER_2;
		else if(lidar.pos == B)
			lidar.pos = CENTER_3;
		else if(lidar.pos == TYPE_THREE)
			lidar.pos = CENTER;

		lidar.pos_counter = PICK_RIGHT;
		mode = AUTO;

	}

	if (HAL_GetTick() - before >= NormalMode) {
		led9= !led9;
		before = HAL_GetTick();
	}
}

void Auto() {
	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);

		shot_count++;

		if(shot_count == 1)
		{
			led_enb = 0;
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
			led8 = 1;
		}
		else if(shot_count == 2)
		{
			led_enb = 1;
			shot_prd = 125;
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
//			vesc_duty = 0.275;
//			vesc_speed = 8.0;
		}
		else if(shot_count == 3)
		{
			led_enb = 1;
			shot_prd = 500;
//			vesc_duty = 0.39;
//			vesc_speed = 12.2;
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

		else if(shot_count == 4)
		{
			led_enb = 0;
			vesc_duty = type1Duty;
			vesc_speed = type1;
			led8 = 0;
			shot_count = 0;
		}
	}

	// Shoot
	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);
		push_shoot;
		shoot_start = 1;
		stop_adjust = 1;
		wait_load = 1;
		shooted++;
//		cylinder_load_once = 1; // Only allow cylinder to load once every shot
//		static int counter = 0;
//		counter++;
//
//		if(counter == 1)
//		{
//			push_shoot;
//			shoot_start = 1;
//			wait_load = 1;
//		}
//		else
//		{
//			adjust_servo;
//			reload = 1;
//			pick_manual(5000);
//			counter = 0;
//		}
	}

	if(ps4.button == SQUARE)
	{
		while(ps4.button == SQUARE);

		if(pp.pp_start)
		{
			PP_stop(&pp);
		}
		else
		{
			loaded = 0;
			vesc_duty = 0.0;
			shot_count = 0;
			flywheelStop();
			pick_stop;
			setPick(500);
			cylinder_load;
			open_servo;
			reload = 0;
			load_start = 0;
//			load_adjust = 0;
			stick_fence = 0;
			picked_manual = 0;
//			pick_right_enb = 0;
		}
	}

	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);

		lidar.pos = PICK_LEFT;
		lidar.pos_counter = PICK_LEFT;

		setPick(2200);

		float pick_left_point[1][7] = {{12.0, -3.6, 0.0, 0, 0, 0, 0}};
		PP_start(pick_left_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
			}

			if(pp.real_x <= -3.2)
				PP_stop(&pp);

			if(ps4.button == UP)
			{
				while(ps4.button == UP);
				go_A = 1;
			}
		}
		pick_left = 1;
		counter++;
		cylinder_retract;


	}

	if(ps4.button == TRIANGLE)
	{
		while(ps4.button == TRIANGLE);
		LoadRing();
		load_adjust = 1;
		picked_manual = 1;
		adjust_servo;
		cylinder_retract;
		AdjustRings();
		adjust_servo;
	}

	if (HAL_GetTick() - before >= AutoMode)
	{
		led9= !led9;
		before = HAL_GetTick();
	}
}

void setPitch(PITCH_t target_pitch)
{
	switch(target_pitch)
	{
		case PITCH45:
			switch(Robot_Pitch)
			{
				case PITCH70:
					pitch_up;
					set_pitch = 1;
					break;

				default:
					break;
			}
			break;

		case PITCH70:
			switch(Robot_Pitch)
			{
				case PITCH45:
					pitch_down;
					set_pitch = 1;
					break;
				default:
					break;
			}
			break;

		default:
			break;
	}
}

void setPick(int32_t target)
{
	pick_target_enc = target;
	set_pick_enc = 1;
}

void CheckPickEnc(void)
{
	if(set_pick_enc)
	{
		pickErr = pick_target_enc - pick_enc;
		PID(&pick_PID);
		pick_manual((int32_t)pickU);

		if(abs(pickErr) < pick_tol)
		{
			set_pick_enc = 0;
			pick_stop;
		}
	}
}

void CheckPitch()
{
	if(set_pitch)
	{
		switch(Robot_Pitch)
		{
			// Setting to pitch 70 deg
			case PITCH45:
				if(In_Pitch70)
				{
					pitch_stop;
					Robot_Pitch = PITCH70;
					set_pitch = 0;
				}
				break;

			// Setting to pitch 45 deg
			case PITCH70:
				if(In_Pitch45)
				{
					pitch_stop;
					Robot_Pitch = PITCH45;
					set_pitch = 0;
				}
				break;
		}
	}
}

void CheckPick()
{
	if(pick_left)
	{
		lidar.pos = PICK_LEFT;
		lidar.pos_counter = PICK_LEFT;
		// Stick to fence
		float stick_fence_point[1][7] = {{2.0, 0.0, -5, 0, 0, 0, 0}};
		PP_start(stick_fence_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
			}

			if(In_LS_Left_1 || In_LS_Left_2)
				PP_stop(&pp);
		}
		pick_left = 0;
		picked_left = 1;

		setPick(0);

		if(pick_left_manual)
			ResetCoordinate();

		float pick_left_point[1][7] = {{3.0, -10, -1.5, 0, 0, 0, 0}};
		PP_start(pick_left_point, 1, &pp);
		while(pp.pp_start)
		{
			if(!pick_left_manual)
			{
				if(pp.real_x <= -4.2)
					pp.target_vel[0] = 0.7;

				if(In_Pick && pp.real_x <= -5.0)
					PP_stop(&pp);
			}
			else
			{
				if(pp.real_x <= -0.25)
					pp.target_vel[0] = 0.55;

				if(In_Pick && pp.real_x <= -0.3)
					PP_stop(&pp);
			}

			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
				picked_left = 0;
			}
		}
		pick_left_manual = 0;
		if(picked_left)
		{
			ResetCoordinate();
			float pick_left_adjust_servo[1][7] = {{4.0, 0.7, 0.2, 0, 0, 0, 0}};
			PP_start(pick_left_adjust_servo, 1, &pp);
			while(pp.pp_start)
			{
				if(ps4.button == SQUARE)
				{
					while(ps4.button == SQUARE);
					PP_stop(&pp);
				}
			}
			LoadRing();
			if(go_A)
				lidar.pos_counter = CENTER_2;
			else
				lidar.pos_counter = CENTER_1;
			load_adjust = 1;
			AdjustRings();
//			adjust_servo;
//			osDelay(500);
//			setSpeedMODN(5.5);
		}
	}

	if(pick_right)
	{
		load_stop_once = 0;
		loaded = 0;
		load_adjust = 1;

//		pick_0 = 1;
//		pick_manual(-10000);

		lidar.pos = PICK_RIGHT;
		lidar.pos_counter = PICK_RIGHT;
		// Stick to fence
		open_servo;
		pick_right = 0;
		float stick_fence_right[1][7] = {{2.0, 10, 0, -90, 0, 0, 0}};
		PP_start(stick_fence_right, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
			}

			if(In_LS_Left_1 || In_LS_Left_2)
				PP_stop(&pp);
		}

		picked_right = 1;
		ResetCoordinate();
		osDelay(10);
		setPick(0);
		float pick_right_point[1][7] = {{4.5, 1.5, -5, -90, 0, 0, 0}};
		PP_start(pick_right_point, 1, &pp);
		while(pp.pp_start)
		{
			if(ps4.button == SQUARE)
			{
				while(ps4.button == SQUARE);
				PP_stop(&pp);
				picked_right = 0;
			}

			if(pp.real_y <= -0.3)
				pp.target_vel[0] = 0.7;

			if(In_Pick && pp.real_y <= -0.3)
				PP_stop(&pp);

			if(ps4.button == UP)
			{
				while(ps4.button == UP);
				go_type_3++;

				if(go_type_3 > 3)
					go_type_3 = 0;
			}
		}

		if(picked_right)
		{
//			LoadRing()
			switch(go_type_3)
			{
				case 0:
					lidar.pos_counter = CENTER_4;
					break;

				case 1:
					lidar.pos_counter = CENTER_3;
					break;

				case 2:
					lidar.pos_counter = CENTER;
					break;

				case 3:
					lidar.pos_counter = CENTER_2;
					break;

				default:
					break;
			}
//			osDelay(500);
//			close_servo;
			setSpeedMODN(5.5);
		}
	}
}

void CheckShoot()
{
//	if(In_ShotReady)
//		led2 = 1;
//	else
//		led2 = 0;
//
//	if(In_ShotDone)
//		led3 = 1;
//	else
//		led3 = 0;

	if(shoot_start && In_ShotDone)
	{
		shoot_done = 1;
		shoot_start = 0;
		push_return;
	}
	else if(In_ShotDone)
	{
		push_return;
		shoot_done = 1;
	}

	// Done Shooting
	if(shoot_done && In_ShotReady && wait_load)
	{
		wait_load = 0;
		shoot_done = 0;
		push_stop;
		adjust_servo;
		reload = 1;
		pick_manual(4000);
		must_load = 1;
//		AutoLoadRing();
	}
	else if(shoot_done && In_ShotReady)
	{
		shoot_done = 0;
		push_stop;
		if(wait_load)
		{
			reload = 1;
			pick_manual(4000);
			wait_load = 0;
		}
	}

//	if(shooted == 10 && pick_right_enb)
//	{
//		shooted = 0;
//		pick_right_enb = 0;
//		osDelay(800);
//		lidar.pos_counter = PICK_RIGHT;
//		open_servo;
//	}

//	if(In_ShotReady && !shoot_start)
//		push_stop;
}

void CheckLoad()
{
//	if(load_start && In_Load)
//	{
//		load_start = 0;
//		pick_stop;
//		AutoLoadRing();
//	}
//	else if(load_start)
//	{
//		pick_up;
//	}

	if(load_start || reload)
	{
		if(pick_enc >= 7000 && !load_stop_once)
		{
			pick_stop;
			osDelay(200);
			pick_manual(4000);
			adjust_servo;
//			close_servo;
			load_stop_once = 1;
		}

		if(pick_enc >= 13000)
		{
			load_start = 0;
			reload = 0;
			pick_stop;
		}
	}

	if(load_start && In_Load && pick_enc >= 6000)
	{
		load_start = 0;
		pick_enc_buf = pick_enc;
//		pick_tol = 10;
//		adjust_servo;

//		if(blue)
//		{
//			while(abs(pick_enc - pick_enc_buf) < BluePickMore)
//				pick_manual(2500);
//		}
//		else
//		{
//			while(abs(pick_enc - pick_enc_buf) < RedPickMore)
//				pick_manual(2500);
//		}

		if(blue)
		{
			while(abs(pick_enc_buf - pick_enc) < BluePickLess)
				pick_manual(-2500);
		}
		else
		{
			while(abs(pick_enc_buf - pick_enc) < RedPickLess)
				pick_manual(-2500);
		}

		pick_tol = 100;
		pick_stop;

		if(mode == AUTO)
			AutoLoadRing();
	}

	if(reload && In_Load)
	{
		pick_enc_buf = pick_enc;
//		pick_tol = 10;
//		adjust_servo;

//		if(blue)
//		{
//			while(abs(pick_enc - pick_enc_buf) < BluePickMore)
//				pick_manual(2500);
//		}
//		else
//		{
//			while(abs(pick_enc - pick_enc_buf) < RedPickMore)
//				pick_manual(2500);
//		}

		if(blue)
		{
			while(abs(pick_enc_buf - pick_enc) < BluePickLess)
				pick_manual(-2500);
		}
		else
		{
			while(abs(pick_enc_buf - pick_enc) < RedPickLess)
				pick_manual(-2500);
		}
//		pick_tol = 100;
		reload = 0;
		must_load = 0;
		pick_stop;

		if(mode == AUTO)
			AutoLoadRing();
	}

//	if(loaded >= 6)
//	{
//		adjust_servo;
//		load_adjust = 0;
//	}

//	if(loaded >= 8)
//	{
//		load_adjust_servo;
//	}
	if(loaded >= 10)
		loaded = 0;

}

void CheckPick0()
{
	if(In_Pick_0)
		led3 = 1;
	else
		led3 = 0;

	if(pick_0 && In_Pick_0)
	{
		pick_0 = 0;
		pick_stop;
		ResetPickEnc();
	}

}

void Checking()
{
	CheckShoot();
	CheckLoad();
	CheckPick();
	CheckPitch();
}

void LoadRing(void)
{
	load_start = 1;
	load_stop_once = 0;
	pick_up;
}

void StopAutoPP(void)
{
	PP_stop(&pp);
	RNSStop(&rns);
}

void AutoLoadRing(void)
{
	cylinder_load;
	osDelay(500);
	cylinder_retract;
	osDelay(500);
	loaded++;

	AdjustRings();
}

void AdjustRings(void)
{
	if(load_adjust)
	{
		servo_enb = 1;
	}
}

void ResetCoordinate(void)
{
	RNSSet(&rns, RNS_PP_Reset);
	PP_reset(&pp);
	osDelay(50);
}

void ResetPickEnc(void)
{
	QEIReset(QEI1);
	pick_enc = 0;
}

void enq(void) {

//	RNSEnquire(RNS_VEL_BOTH, &rns);
//	a = rns.enq.enq_buffer[0].data;
//	b = rns.enq.enq_buffer[1].data;
//	c = rns.enq.enq_buffer[2].data;
//	d = rns.enq.enq_buffer[3].data;
//	sprintf((char*)debug, "A: %.2f B: %.2f C: %.2f D: %.2f\n", a, b, c, d);

//	RNSEnquire(RNS_POS_BOTH, &rns);
//	a = rns.enq.enq_buffer[0].data;
//	b = rns.enq.enq_buffer[1].data;
//	c = rns.enq.enq_buffer[2].data;
//	d = rns.enq.enq_buffer[3].data;

//	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
//	xpos = rns.enq.enq_buffer[0].data;
//	ypos = rns.enq.enq_buffer[1].data;
//	z = rns.enq.enq_buffer[2].data;
//	sprintf((char*)debug, "X: %.2f Y: %.2f Yaw: %.2f\n", xpos, ypos, z);

	RNSEnquire(RNS_LSR_DIST, &rns);
	lsrL.dist = rns.enq.enq_buffer[0].data;
	lsrR.dist = rns.enq.enq_buffer[1].data;

	RNSEnquire(RNS_XYZ_RAW, &rns);
	fXPos = rns.enq.enq_buffer[0].data;
	fYPos = rns.enq.enq_buffer[1].data;
	fyaw = rns.enq.enq_buffer[2].data;

//	sprintf((char*)debug, "A: %.2f B: %.2f C: %.2f D: %.2f X: %.2f Y: %.2f Z: %.2f Servo: %ld\n", a, b, c, d, fXPos, fYPos, fyaw, servo_ring.currentPulse);
//	sprintf((char*)debug, "X: %.2f Y: %.2f Yaw: %.2f\n", fXPos, fYPos, fyaw);
}

void tune(void)
{
	if(ps4.button == UP)
	{
		while(ps4.button == UP);
//		tune_p = 1;
//		tune_i = 0;
//		tune_d = 0;
		if(blue)
			BluePickLess = pick_enc_buf - pick_enc;
		else
			RedPickLess = pick_enc_buf - pick_enc;
	}

	if(ps4.button == DOWN)
	{
		while(ps4.button == DOWN);
		if(blue)
			BluePickMore = pick_enc - pick_enc_buf;
		else
			RedPickMore = pick_enc - pick_enc_buf;
	}

	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);
		tune_p = 0;
		tune_i = 0;
		tune_d = 1;
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		wheel++;
		if(wheel > 5)
			wheel = 1;
	}

	if(tune_p)
	{
		switch(wheel)
		{
		case 1:
			AP += ps4.joyR_y * 0.0001;
			break;
		case 2:
			BP += ps4.joyR_y * 0.0001;
			break;
		case 3:
			CP += ps4.joyR_y * 0.0001;
			break;
		case 4:
			DP += ps4.joyR_y * 0.0001;
			break;
		}
	}
	else if(tune_i)
	{
		switch(wheel)
		{
		case 1:
			AI += ps4.joyR_y * 0.0001;
			break;
		case 2:
			BI += ps4.joyR_y * 0.0001;
			break;
		case 3:
			CI += ps4.joyR_y * 0.0001;
			break;
		case 4:
			DI += ps4.joyR_y * 0.0001;
			break;
		}
	}
	else
	{
		switch(wheel)
		{
		case 1:
			AD += ps4.joyR_y * 0.00001;
			break;
		case 2:
			BD += ps4.joyR_y * 0.00001;
			break;
		case 3:
			CD += ps4.joyR_y * 0.00001;
			break;
		case 4:
			DD += ps4.joyR_y * 0.00001;
			break;
		}
	}

	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);
//		switch(wheel)
//		{
//		case 1:
//			RNSSet(&rns, RNS_F_LEFT_VEL_PID, AP, AI, AD);
//			break;
//
//		case 2:
//			RNSSet(&rns, RNS_F_RIGHT_VEL_PID, BP, BI, BD);
//			break;
//
//		case 3:
//			RNSSet(&rns, RNS_B_LEFT_VEL_PID, CP, CI, CD);
//			break;
//
//		case 4:
//			RNSSet(&rns, RNS_B_RIGHT_VEL_PID, DP, DI, DD);
//			break;
//		}

//		if(blue)
//			BluePickLess = pick_enc_buf - pick_enc;
//		else
//			RedPickLess = pick_enc_buf - pick_enc;
	}

	if(ps4.button == TRIANGLE)
	{
		while(ps4.button == TRIANGLE);

		LoadRing();
		load_adjust = 1;
		picked_manual = 1;
		adjust_servo;
		cylinder_retract;
		AdjustRings();
		adjust_servo;
	}

	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);
	}

	led9 = 1;
}

void flywheelPID(float speed)
{
	leftErr = speed - vesc1.vel;
	rightErr = speed - vesc2.vel;

	start_flywheel = 1;
}

void flywheelAct()
{
	PID(&left_PID);
	PID(&right_PID);

	VESCPDC(leftU / 19999.00, &vesc1);
	VESCPDC(rightU / 19999.00, &vesc2);
}

void flywheelStop()
{
	vesc_speed = 0;

	PIDDelayInit(&left_PID);
	PIDDelayInit(&right_PID);

	leftU = 0;
	rightU = 0;

	VESCStop(&vesc1);
	VESCStop(&vesc2);
}

void Shot()
{
	push_shoot;
	shoot_start = 1;
	wait_load = 1;
}

void Await(uint32_t ticks) {
	counter = 0;
	while (counter < ticks);
}
