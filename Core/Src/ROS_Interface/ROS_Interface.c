/*
 * ROS_Interface.c
 * 		Created on: 17 Apr 2022
 * 		Author: Peisen Wong
 */

#include "ROS_Interface.h"

/*
 * Call this function in set() as initialization
 */
void ROS_Init(UART_HandleTypeDef* huartx)
{
	ROS_UART = huartx;
	path_num = 0;
	ros_counter = 0;
	HAL_UART_Receive_IT(huartx, ROS_buff, 7);
}

/*
 *  Write All the PP_Points into Flash
 *  Including different path
 */
uint32_t ROS_Write_Flash(void)
{
	uint32_t success = 0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_8;
	EraseInitStruct.NbSectors     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}

	uint32_t address = SECTOR8;
	uint32_t temp;

	memcpy(&temp, &path_num, 4);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, temp);
	address += 4;

	for(int i = 0; i < path_num; i++)
	{
		memcpy(&temp, &point_num[i], 4);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, temp);
		address += 4;
	}

	for(int i = 0; i < path_num; i++) // Number of path
	{
		for(int j = 0; j < point_num[i]; j++) // Number of point at specific path
		{
			for(int k = 0; k < 7; k++)
			{
				memcpy(&temp, &PP_Points[i][j][k], 4);
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, temp) == HAL_OK)
				{
					success++;
				}
				address += 4;
			}
		}
	}
	HAL_FLASH_Lock();
	return success;
}

/*
 * Call this function in the USART Receive CallBack
 */
void ROS_Handler(void)
{
	static uint8_t state = 0;
	switch(state)
	{
	case 0:
		if(ROS_buff[0] == 0x01 && ROS_buff[1] == 0x02)
		{
//			led2 = !led2;
			memcpy(&instruction, &ROS_buff[2], 1);
			if(instruction == TEST)
			{
				state = 1;
				memcpy(&test_point_num, &ROS_buff[3], 4);

				test_points = (float**)malloc(test_point_num * sizeof(float*));
				for(int i = 0; i < test_point_num; i++)
					test_points[i] = (float*)malloc(7 * sizeof(float));

				HAL_UART_Transmit(ROS_UART, (uint8_t* )&test_point_num, 4, HAL_MAX_DELAY);
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 28 * test_point_num);
			}

			else if(instruction == REGISTER)
			{
				state = 0;
				ROS_Register();
			}

			else if(instruction == WRITE)
			{
				total_point_num = 0;
				uint32_t write_success = 0;
				write_success += ROS_Write_Flash();

				for(int i = 0; i < path_num; i++)
				{
					total_point_num += point_num[i];
				}

				if(write_success == total_point_num * 7)
				{
					ack = 0x21;
					HAL_UART_Transmit(ROS_UART, &ack, 1, HAL_MAX_DELAY);
				}
				else
				{
					ack = 0x22;
					HAL_UART_Transmit(ROS_UART, &ack, 1, HAL_MAX_DELAY);
				}

				state = 0;
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
			}

			else if(instruction == DELETE)
			{
				state = 0;
				memcpy(&path_index, &ROS_buff[3], 4);

				ROS_Delete(path_index);
			}

			else if(instruction == READ)
			{
				memcpy(&path_index, &ROS_buff[3], 4);

				for(int i = 0; i < point_num[path_index - 1]; i++)
				{
					for(int j = 0; j < 7; j++)
					{
						HAL_UART_Transmit(ROS_UART, (uint8_t*)&PP_Points[path_index - 1][i][j], 4, HAL_MAX_DELAY);
					}
				}
				state = 0;
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
			}

			else if(instruction == DELETE_ALL)
			{
				state = 0;
				ROS_DeleteAll();
			}

			else if(instruction == EDIT_PATH)
			{
				state = 0;
				ROS_EditPath();
			}

			else if(instruction == GET_COUNTER)
			{
				HAL_UART_Transmit(ROS_UART, (uint8_t*)&ros_counter, 4, HAL_MAX_DELAY);

				state = 0;
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
			}

			else if(instruction == EDIT_POINT)
			{
				state = 2;
				memcpy(&path_index, &ROS_buff[3], 4);
				HAL_UART_Transmit(ROS_UART, (uint8_t*)&path_index, 4, HAL_MAX_DELAY);

				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 32); // 7 Data + 1 Point Index
			}

			else if(instruction == DELETE_POINT)
			{
				state = 3;
				memcpy(&path_index, &ROS_buff[3], 4);
				HAL_UART_Transmit(ROS_UART, (uint8_t*)&path_index, 4, HAL_MAX_DELAY);

				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 4);
			}

			else if(instruction == TEST_RUN)
			{
				if(test_point_num)
				{
					sys.ros_test_start = 1;
				}

				state = 0;
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
			}

			else if(instruction == RUN_PATH)
			{
				memcpy(&path_index, &ROS_buff[3], 4);
				if(point_num[path_index - 1])
				{
					sys.ros_path_start = 1;
				}

				state = 0;
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
			}

			else if(instruction == STOP_RUN)
			{
				sys.ros_stop = 1;
				state = 0;
				HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
			}
		}

		else
		{
			HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
		}

		break;

	case 1:
		for(int i = 0; i < test_point_num; i++)
		{
			for(int j = 0; j < 7; j++)
			{
				memcpy(&test_points[i][j], &ROS_buff[j * 4 + i * 28], 4);
			}
		}

		for(int i = 0; i < test_point_num; i++)
		{
			for(int j = 0; j < 7; j++)
			{
				HAL_UART_Transmit(ROS_UART, (uint8_t*)&test_points[i][j], 4, HAL_MAX_DELAY);
			}
		}
		HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
		state = 0;
		break;

	case 2:
		point_buf = (float*)malloc(7 * sizeof(float));
		memcpy(&point_index, &ROS_buff[0], 4);

		for(int i = 0; i < 7; i++)
		{
			memcpy(&point_buf[i], &ROS_buff[4 + i * 4], 4);
		}

		for(int i = 0; i < 7; i++)
		{
			HAL_UART_Transmit(ROS_UART, (uint8_t*)&point_buf[i], 4, HAL_MAX_DELAY);
		}

		for(int i = 0; i < 7; i++)
		{
			PP_Points[path_index - 1][point_index - 1][i] = point_buf[i];
		}

		state = 0;
		HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
		break;

	case 3:
		memcpy(&point_index, &ROS_buff, 4);

		if(point_num[path_index - 1] == 1)
		{
			ROS_Delete(path_index);
		}
		else
		{
			point_num[path_index - 1]--;

			for(int i = point_index - 1; i < point_num[path_index - 1]; i++)
			{
				PP_Points[path_index - 1][i] = PP_Points[path_index - 1][i + 1];
			}

			PP_Points[path_index - 1] = (float**)realloc(PP_Points[path_index - 1], point_num[path_index - 1] * sizeof(float*));
		}

		state = 0;
		HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
		break;
	}

}

/*
 * Call this function in set() to rewrite PP_Points with value stored in Flash
 */
void ROS_Read_Flash(void)
{
	// Enabling checking of FLASH value using usart5 by pressing reset button
	uint32_t address = SECTOR8;
	memcpy(&path_num, (__IO uint32_t *)address, 4);
	address += 4;

	if(path_num)
	{
		sprintf((char*)ROS_buff, "path_num is %d\n\n", path_num);
		HAL_UART_Transmit(&huart5, ROS_buff, strlen((char*)ROS_buff), HAL_MAX_DELAY);

		PP_Points = (float***)malloc(path_num * sizeof(float**));
		point_num = (int*)malloc(path_num * sizeof(int));

		for(int i = 0; i < path_num; i++)
		{
			memcpy(&point_num[i], (__IO uint32_t *)address, 4);
			address += 4;
		}

		for(int i = 0; i < path_num; i++)
		{
			sprintf((char*)ROS_buff, "Point num for path %d is %d\n", i + 1, point_num[i]);
			HAL_UART_Transmit(&huart5, ROS_buff, strlen((char*)ROS_buff), HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart5, (uint8_t*)"\n", 1, HAL_MAX_DELAY);

		for(int i = 0; i < path_num; i++)
		{
			PP_Points[i] = (float**)malloc(point_num[i] * sizeof(float*));
		}

		for(int i = 0; i < path_num; i++)
		{
			for(int j = 0; j < point_num[i]; j++)
			{
				PP_Points[i][j] = (float*)malloc(7 * sizeof(float));
			}
		}

		for(int i = 0; i < path_num; i++)
		{
			for(int j = 0; j < point_num[i]; j++)
			{
				for(int k = 0; k < 7; k++)
				{
					memcpy(&PP_Points[i][j][k], (__IO uint32_t *)address, 4);
					address += 4;
				}
			}
		}

		for(int i = 0; i < path_num; i++)
		{
			for(int j = 0; j < point_num[i]; j++)
			{
				for(int k = 0; k < 7; k++)
				{
					sprintf((char*)ROS_buff, "Point %d of point %d of path %d is %.2f\n", k + 1, j + 1, i + 1, PP_Points[i][j][k]);
					HAL_UART_Transmit(&huart5, ROS_buff, strlen((char*)ROS_buff), HAL_MAX_DELAY);
				}
				HAL_UART_Transmit(&huart5, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
			}
			HAL_UART_Transmit(&huart5, (uint8_t*)"\n\n", 2, HAL_MAX_DELAY);
		}
	}
	else
	{
		sprintf((char*)ROS_buff, "No memory in flash");
		HAL_UART_Transmit(&huart5, ROS_buff, strlen((char*)ROS_buff), HAL_MAX_DELAY);
	}
}

/*
 * Once the testing path is satisfied, calling this function will register the path in PP_Points
 * Only registered path can be run through RNSPPstart(PP_Points[counter])
 */
void ROS_Register(void)
{
	path_num++;
	if(path_num > 1)
	{
		PP_Points = (float***)realloc(PP_Points, path_num * sizeof(float**));
		point_num = (int*)realloc(point_num, path_num * sizeof(int));

		PP_Points[path_num - 1] = (float**)malloc(test_point_num * sizeof(float*));
		for(int i = 0; i < test_point_num; i++)
		{
			PP_Points[path_num - 1][i] = (float*)malloc(7 * sizeof(float));
		}
	}
	else if(path_num == 1)
	{
		PP_Points = (float***)malloc(path_num * sizeof(float**));
		point_num = (int*)malloc(path_num * sizeof(int));

		PP_Points[path_num - 1] = (float**)malloc(test_point_num * sizeof(float*));
		for(int i = 0; i < test_point_num; i++)
		{
			PP_Points[path_num - 1][i] = (float*)malloc(7 * sizeof(float));
		}
	}

	point_num[path_num - 1] = test_point_num;

	for(int i = 0; i < test_point_num; i++)
	{
		for(int j = 0; j < 7; j++)
		{
			PP_Points[path_num - 1][i][j] = test_points[i][j];
		}
	}

	for(int i = 0; i < test_point_num; i++)
	{
		for(int j = 0; j < 7; j++)
		{
			HAL_UART_Transmit(ROS_UART, (uint8_t*)&PP_Points[path_num - 1][i][j], 4, HAL_MAX_DELAY);
		}
	}

	HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
}

/*
 * Delete specific path in PP_Points
 * User not able to run the path again
 * Can only delete registered path
 */
void ROS_Delete(int path_index)
{
	path_num--;
	if(path_index - 1 == path_num)
	{
		point_num = (int*)realloc(point_num, path_num * sizeof(int));
		for(int i = 0; i < path_num; i++)
		{
			total_point_num += point_num[i];
		}
		PP_Points = (float***)realloc(PP_Points, total_point_num * 7 * sizeof(float));

		sprintf((char*)ROS_buff, "Path index is %d\n", path_index);
		HAL_UART_Transmit(&huart5, ROS_buff, 16, HAL_MAX_DELAY);
	}

	else
	{
		for(int i = path_index - 1; i < path_num; i++)
		{
			point_num[i] = point_num[i + 1];
		}
		point_num = (int*)realloc(point_num, path_num * sizeof(int));

		for(int i = path_index - 1; i < path_num; i++)
		{
			PP_Points[i] = PP_Points[i + 1];
		}

		for(int i = 0; i < path_num; i++)
		{
			total_point_num += point_num[i];
		}

		PP_Points = (float***)realloc(PP_Points, total_point_num * 7 * sizeof(float));
		sprintf((char*)ROS_buff, "Path index is %d\n", path_index);
		HAL_UART_Transmit(&huart5, ROS_buff, 16, HAL_MAX_DELAY);
	}

	HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);

}

/*
 * Delete all existing registered path
 * Usually used to reset all path registered
 */
void ROS_DeleteAll(void)
{
	for(int i = 0; i < path_num; i++)
	{
		for(int j = 0; j < point_num[i]; j++)
		{
			free(PP_Points[i][j]);
		}
	}

	for(int i = 0; i < path_num; i++)
	{
		free(PP_Points[i]);
	}

	free(PP_Points);
	free(point_num);

	ROS_Init(ROS_UART);
}

/*
 * Allow user to edit registered path with different number of points than original ones
 * After satisfied the testing path, it will overwrite the desired path in PP_Points
 * Call RNSPPStart(PP_Points[${editing_index}] to see changes
 */
void ROS_EditPath(void)
{
	total_point_num = 0;
	memcpy(&edit_index, &ROS_buff[3], 4);

	if(edit_index == path_num)
	{
		/*
		 Just delete the last path and require user to register again
		 */
		path_num--;
		point_num = (int*)realloc(point_num, path_num * sizeof(int));
		for(int i = 0; i < path_num; i++)
		{
			total_point_num += point_num[i];
		}
		PP_Points = (float***)realloc(PP_Points, total_point_num * 7 * sizeof(float));
	}
	else
	{
		/*
		 -Malloc another 3D array based on paths behind the path to be edit
		 -Copy those to another 3D arr
		 -Realloc the path array based on edit_num + other's path point num except edit index
		 -Copy test_points into specific path and other path into the path array
		 -Free all array except for path array
		*/

		edit_offset = path_num - edit_index;
		Edit_Buff = (float***)malloc(edit_offset * sizeof(float**));

		for(int i = 0; i < edit_offset ; i++)
		{
			Edit_Buff[i] = (float**)malloc(point_num[edit_index + i] * sizeof(float*));
		}

		for(int i = 0; i < edit_offset; i++)
		{
			for(int j = 0; j < point_num[edit_index + i]; j++)
			{
				Edit_Buff[i][j] = (float*)malloc(7 * sizeof(float));
			}
		}

		// Copy value inside buff
		for(int i = 0; i < edit_offset; i++)
		{
			for(int j = 0; j < point_num[edit_index + i]; j++)
			{
				for(int k = 0; k < 7; k++)
				{
					Edit_Buff[i][j][k] = PP_Points[edit_index + i][j][k];
				}
			}
		}

		for(int i = 0; i < path_num; i++)
		{
			if(i % edit_index == 0)
			{
				total_point_num += test_point_num;
			}
			else
			{
				total_point_num += point_num[i];
			}
		}

		// Realloc whole PP_Points
		PP_Points = (float***)realloc(PP_Points, total_point_num * 7 * sizeof(float));
		PP_Points[edit_index - 1] = (float**)malloc(test_point_num * sizeof(float*));

		for(int i = 0; i < test_point_num; i++)
		{
			PP_Points[edit_index - 1][i] = (float*)malloc(7 * sizeof(float));
		}

		// Put the edit value inside specific path
		for(int i = 0; i < test_point_num; i++)
		{
			for(int j = 0; j < 7; j++)
			{
				PP_Points[edit_index - 1][i][j] = test_points[i][j];
			}
		}

		// Malloc for other existing path
		for(int i = 0; i < edit_offset; i++)
		{
			PP_Points[edit_index + i] = (float**)malloc(point_num[edit_index + i] * sizeof(float*));
		}

		for(int i = 0; i < edit_offset; i++)
		{
			for(int j = 0; j < point_num[edit_index + i]; j++)
			{
				PP_Points[edit_index + i][j] = (float*)malloc(7 * sizeof(float));
			}
		}

		// Copy back all remaining path
		for(int i = 0; i < edit_offset; i++)
		{
			for(int j = 0; j < point_num[edit_index + i]; j++)
			{
				for(int k = 0; k < 7; k++)
				{
					PP_Points[edit_index + i][j][k] = Edit_Buff[i][j][k];
				}
			}
		}

		// Rewrite the point_num arr
		point_num[edit_index - 1] = test_point_num;
		for(int i = 0; i < test_point_num; i++)
		{
			for(int j = 0; j < 7; j++)
			{
				HAL_UART_Transmit(ROS_UART, (uint8_t*)&PP_Points[edit_index - 1][i][j], 4, HAL_MAX_DELAY);
			}
		}

		// Free the edit_buf
//		for(int i = 0; i < edit_offset; i++)
//		{
//			for(int j = 0; j < point_num[edit_offset + i]; j++)
//			{
//				free(Edit_Buff[i][j]);
//			}
//		}
//
//		for(int i = 0; i < edit_offset; i++)
//		{
//			free(Edit_Buff[i]);
//		}
//
//		free(Edit_Buff);
		HAL_UART_Receive_IT(ROS_UART, ROS_buff, 7);
	}
}
