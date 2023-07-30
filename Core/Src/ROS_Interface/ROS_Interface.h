/*
 * ROS_Interface.h
 * 		Created on: 17 Apr 2022
 * 		Author: Peisen Wong
 */

#ifndef SRC_ROS_INTERFACE_ROS_INTERFACE_H_
#define SRC_ROS_INTERFACE_ROS_INTERFACE_H_

#include "../common.h"
#include "../adapter.h"
#include "../RNS_interface/RNS_interface.h"
#include "../PSx_Interface/PSx_Interface.h"

/*
 * Basic Usage:
 * Interaction between mainboard and ROS Rviz to tune path plan
 *
 *
 * How To Use:
 * PP_Points is a 3D array to store multiple path in it
 * point_num is an array which store the number of points for each correspond path
 * Just call RNSPPStart(PP_Points[counter], point_num[counter], &rns) to run the path stored in array
 * Can set 2 more button (L1 R1 to change the value of counter to run different path
 *
 * test_points is for testing purpose
 * Just call RNSPPStart(test_points, test_point_num, &rns) to test the path plan
 *
 * Edit_buff is just for editing purpose
 * The array will be free after editing
 *
 * Before using, call ROS_Init(${Your desired UART}) and ROS_Read_Flash() in the common.c as initialization
 * If some bugs occur, make sure to comment out ROS_Read_Flash() until user write into flash
 * Call ROS_Handler in USART CallBack function to enable communication between pc and mainboard through bluetooth
 *
 * The format of each instruction message be
 * |Header 1| |Header 2| |Instruction| |Int (if not needed just put 0)|
 *
 * User able to run or stop the path tuned directly through commands from PC without using PS4
 * Make sure to add the following flags in the common.h file
 * 		unsigned ros_test_start  :1;
		unsigned ros_path_start  :1;
		unsigned ros_stop		 :1;
 * These flags are to start or stop the path plan and should be in main.c
 * In the same time, user can repeat same functionality of PS4(start stop etc)
 *
 * Note that the address for the flash of path plan data is in sector 8
 * Change the address in the eeprom.h based on documentation (0x08080004)
 *
 */

// Final points storage for path plan
float*** PP_Points;
int* point_num;

// Testing points storage for testing purpose only
float** test_points;
int test_point_num;

// Edit points storage
float*** Edit_Buff;

// To store edit_point value
float* point_buf;

unsigned char instruction;
enum
{
	TEST,
	REGISTER,
	WRITE,
	EDIT_PATH,
	EDIT_POINT,
	DELETE,
	DELETE_POINT,
	DELETE_ALL,
	READ,
	GET_COUNTER,
	TEST_RUN,
	RUN_PATH,
	STOP_RUN
};

uint8_t ROS_buff[5000];
uint8_t send_buf[500];
uint8_t ack;
int path_num, ros_counter, path_index, edit_index, edit_offset, point_index;
int total_point_num;
UART_HandleTypeDef* ROS_UART;

void ROS_Init(UART_HandleTypeDef* huartx);
uint32_t ROS_Write_Flash(void);
void ROS_Handler(void);
void ROS_Read_Flash(void);
void ROS_Register(void);
void ROS_Delete(int path_index);
void ROS_DeleteAll(void);
void ROS_EditPath(void);

#endif
