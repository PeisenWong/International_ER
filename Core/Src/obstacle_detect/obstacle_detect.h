#ifndef SRC_OBSTACLE_DETECT_H
#define SRC_OBSTACLE_DETECT_H

#include "../adapter.h"

typedef struct
{
	double x;
	double y;
	double distance;
	double angle;
}Pole;

typedef enum
{
	FAR,
	NEAR,
	ONE,
	TWO,
	THREE,
	FOUR,
	FIVE,
	SIX
}Instruction_t; // Give instruction to pi

typedef enum
{
	OK,
	NO
}Response_t; // Check response from pi

typedef enum
{
	CONTINUOUS,
	DISCONTINUOUS
}MODE_t;

typedef enum
{
	POS_PENDING,
	PICK_LEFT,
	UPPER_LEFT,
	CENTER_1,
	CENTER_2,
	CENTER,
	CENTER_3,
	CENTER_4,
	UPPER_RIGHT,
	PICK_RIGHT
}Pos_t; // Try from different pose

// Adjust angle while shooting pole in position respectively
typedef enum
{
	BL,   // back left
	FL,   // front left
	S,   // straight
	FR,   // front right
	BR	 // back right
}Angle_t;

typedef struct
{
	uint8_t obstacle_ack, obstacle_send[200], obstacle_receive[200];
	int obstacle_count;
	uint8_t inst, res;

	UART_HandleTypeDef* lidar_UART;
	MODE_t mode;
	Pole pole;
	Pole* Polelist;
	Response_t response;
	Pos_t pos;
	Angle_t angle;

	/*
	 * Let say now using second strategy, lidar adjustment after every high speed path plan changing position
	 * Every time the path plan stops, measure the nearest pole and compare to preset value to adjust the position
	 */

	// Angles to shoot at different position
	float left_angle_A, left_angle_B, left_angle_C, left_angle_D;
	float right_angle_A, right_angle_B, right_angle_C, right_angle_D;
	float center_angle_A, center_angle_B;

	// Offset for each position
	float left_offset_x, left_offset_y, right_offset_x, right_offset_y, center_offset_x, center_offset_y;

	// Testing purpose only, hope wont use these parameters
	float center_1_offset_x, center_1_offset_y, center_2_offset_x, center_2_offset_y, center_3_offset_x, center_3_offset_y, center_4_offset_x, center_4_offset_y;

	float adj_x, adj_y;
	float center_1_adj_min_x, center_1_adj_max_x, center_1_adj_min_y, center_1_adj_max_y;
	float center_2_adj_min_x, center_2_adj_max_x, center_2_adj_min_y, center_2_adj_max_y;
	float center_adj_min_x, center_adj_max_x, center_adj_min_y, center_adj_max_y;
	float center_3_adj_min_x, center_3_adj_max_x, center_3_adj_min_y, center_3_adj_max_y;
	float center_4_adj_min_x, center_4_adj_max_x, center_4_adj_min_y, center_4_adj_max_y;

	// Counters for pose and angle
	int pos_counter, angle_counter;

	int AdjEnb, fail, laser, autoshot;

	union{
		uint16_t flags;
		struct{
			unsigned new		:	1;
			unsigned reject		:	1;
			unsigned start		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
			unsigned bit8		:	1;
			unsigned bit9		:	1;
			unsigned bit10		:	1;
			unsigned bit11		:	1;
			unsigned bit12		:	1;
			unsigned bit13		:	1;
			unsigned bit14		:	1;
			unsigned bit15		:	1;
		};
	};
}Lidar_t;

extern float Lidar_Offsets[6];
extern float Lidar_Shoot_Angles[10];
extern float Lidar_Center_Offsets[8]; // Testing only
extern float Lidar_Adjust_Lim[10];

Lidar_t lidar;
void LidarInit(UART_HandleTypeDef* lidarUART, MODE_t mode, Lidar_t* lidar);
void LidarSendIns(Instruction_t ins, Lidar_t* lidar);
void ObstacleHandler();
void LidarOffsetInit(float left_offset_x, float left_offset_y, float right_offset_x, float right_offset_y, float center_offset_x,
		float center_offset_y, Lidar_t* lidar);
void LidarAnglesInit(float left_angle_A, float left_angle_B, float left_angle_C, float left_angle_D,
					 float right_angle_A, float right_angle_B, float right_angle_C, float right_angle_D,
					 float center_angle_A, float center_angle_B, Lidar_t* lidar);
void LidarAdjustPP(float adj_x, float adj_y, float z);
void LidarAdjust(Lidar_t* lidar);
void LidarSetAngle(Angle_t angle, Lidar_t* lidar);
void LidarCheckAngle(Lidar_t* lidar);
void LidarSetPos(Pos_t pose, Lidar_t* lidar);
void LidarCheckPos(Lidar_t* lidar);
void LidarControl(Lidar_t* lidar);
void LidarCenterOffsetInit(float center_1_offset_x, float center_1_offset_y, float center_2_offset_x, float center_2_offset_y,
		float center_3_offset_x, float center_3_offset_y, float center_4_offset_x, float center_4_offset_y, Lidar_t* lidar);

#endif
