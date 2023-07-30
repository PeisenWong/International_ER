
#ifndef INSTRUCTION_H_
#define INSTRUCTION_H_
/*********************************************/
/*          Enumarator                       */
/*********************************************/

enum {
	x_reach=1,
	y_reach,
	z_reach
};

enum {
	coordinate_and_Velocity_triomni ,
	coordinate_and_Velocity_fwdomni,
	LSA_front,
	LSA_back,
	LSA_left,
	LSA_right
};

typedef enum{
	x_ax = 8,
	y_ax
}movedir_t;

typedef enum{
	AT_FRONT = 10,
	AT_BACK,
	AT_LEFT,
	AT_RIGHT
}lsa_pos_t;

typedef enum{
	DIR_FRONT = 14,
	DIR_BACK,
	DIR_LEFT,
	DIR_RIGHT
}dir_t;

enum {
	tri_omni=1,
	fwd_omni,
	mecanum
};

enum {
	fuzzyPID=1,
	roboconPID,
	s_fuzzyPID
};

// maximum 255
enum {
	/* Pending */
	RNS_PENDING,
	RNS_BUSY,
	RNS_WAITING,
	/**************************************************************************/

	/* Reset */
	RNS_STOP,			/* Stop everything */

	RNS_INS_RESET,
	/**************************************************************************/

	/* Velocity */
	RNS_VELOCITY,
	RNS_VELOCITY1,
	RNS_VELOCITY2,
	RNS_VELOCITY3,
	RNS_VELOCITY4,
	RNS_PDC,
	RNS_CONTROLLER,

	RNS_INS_VELOCITY,
	/**************************************************************************/

	/*Line Follow*/
	RNS_LF_DIST,
	RNS_LF_JUNC,

	RNS_INS_LINE_FOLLOW,
	/**************************************************************************/

	/*Path Planning*/

	RNS_PPStart,
	RNS_LSRStart,
	RNS_LSRPointStart,
	RNS_ROTATE,
	RNS_PICK_LEFT,
	RNS_PICK_LEFT_FENCE,
	RNS_PICK_LEFT_RING,

	RNS_INS_PATH_PLAN,
	/**************************************************************************/

	/* Setting */
	RNS_PARAM_0,

	RNS_RESET_POS,		/* 0 */
	RNS_PPReset, 		/* 0 */
	RNS_PPInit,  		/* 0 */
	RNS_PP_Reset,

	RNS_PARAM_1,

	RNS_PPSend_num_Point, 	/* 1 */
	RNS_PPSetX,				/* 1 */
	RNS_PPSetY,				/* 1 */
	RNS_PPSetZ,				/* 1 */
	RNS_PPSetCRV_PTS,       /* 1 */

	RNS_PARAM_2,

	RNS_F_KCD_PTD,		/* 2 */
	RNS_B_KCD_PTD,		/* 2 */
	RNS_Printing,  		/* 2 */
	RNS_SET_PP_XY,

	RNS_PARAM_3,

	RNS_DEVICE_CONFIG,	 	/* 3 */

	RNS_F_LEFT_ABT,			/* 3 */
	RNS_F_RIGHT_ABT,		/* 3 */
	RNS_B_LEFT_ABT,			/* 3 */
	RNS_B_RIGHT_ABT,		/* 3 */

	RNS_X_ABT, 				/* 3 */
	RNS_Y_ABT, 				/* 3 */

	RNS_F_LEFT_VEL_SATEU,	/* 3 */
	RNS_F_RIGHT_VEL_SATEU,	/* 3 */
	RNS_B_LEFT_VEL_SATEU,	/* 3 */
	RNS_B_RIGHT_VEL_SATEU,	/* 3 */

	RNS_F_LEFT_VEL_PID,		/* 3 */
	RNS_F_RIGHT_VEL_PID,	/* 3 */
	RNS_B_LEFT_VEL_PID,		/* 3 */
	RNS_B_RIGHT_VEL_PID,	/* 3 */

	RNS_ROTATE_SATEU, 		/* 3 */
	RNS_ROTATE_PID, 		/* 3 */

	RNS_LF_DIST_SATEU,  	/* 3 */
	RNS_LF_DIST_PID,		/* 3 */
	RNS_LF_ROTATE_SATEU, 	/* 3 */
	RNS_LF_ROTATE_PID, 		/* 3 */
	RNS_LF_FWD_SATEU, 		/* 3 */
	RNS_LF_FWD_PID, 		/* 3 */

	RNS_XYZfeedback, 		/* 3 */
	RNS_CONTROLLER_BASE_SPEED, /* 3 */

	RNS_F_LEFT_VEL_FUZZY_PID_BASE, 	/* 3 */
	RNS_F_RIGHT_VEL_FUZZY_PID_BASE, /* 3 */
	RNS_B_LEFT_VEL_FUZZY_PID_BASE, 	/* 3 */
	RNS_B_RIGHT_VEL_FUZZY_PID_BASE, /* 3 */

	RNS_F_LEFT_VEL_FUZZY_PID_PARAM, /* 3 */
	RNS_F_RIGHT_VEL_FUZZY_PID_PARAM,/* 3 */
	RNS_B_LEFT_VEL_FUZZY_PID_PARAM, /* 3 */
	RNS_B_RIGHT_VEL_FUZZY_PID_PARAM,/* 3 */

	RNS_PPSetXYZerror, 	/* 3 */
	RNS_PPPathPID,
	RNS_PPEndPID,

	RNS_PARAM_4,
	RNS_X_Y_ENC_CONFIG,  					/* 4 */

	RNS_F_LEFT_VEL_FUZZY_PID_UEECES_MAX, 	/* 4 */
	RNS_F_RIGHT_VEL_FUZZY_PID_UEECES_MAX, 	/* 4 */
	RNS_B_LEFT_VEL_FUZZY_PID_UEECES_MAX, 	/* 4 */
	RNS_B_RIGHT_VEL_FUZZY_PID_UEECES_MAX, 	/* 4 */

	RNS_LF_LSA_POS, 						/* 4 */
	RNS_PPZPID,

	RNS_PARAM_5,
//	RNS_PPSendPoint,

	RNS_PARAM_6,

	RNS_PARAM_7,
	RNS_PPSendPoint,

	RNS_PARAM_8,

	RNS_INS_PARAM,
	/**************************************************************************/

	/* Enquiry */
	RNS_POS_BOTH,
	RNS_VEL_BOTH,
	RNS_PDC_BOTH,
	RNS_X_Y_POS,
	RNS_X_Y_RAW,
	RNS_X_Y_IMU_LSA,
	RNS_LSA_ALL,
	RNS_LF_JUNCTION,
	RNS_ANGLE,
	RNS_COORDINATE_X_Y_Z_Zrad,
	RNS_PathPlan_VELOCITY,
	RNS_PathPlanEND,
	RNS_LSR_DIST,
	RNS_LSR_RAW,
	RNS_XYZ_RAW,

	RNS_INS_ENQ,


	//USER
	RNS_INS_USER

	/**************************************************************************/
};


#endif /* INSTRUCTION_H_ */
