/************************************************
 * Title   : CAN
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: Send and Receive Data Using CAN
 *
 *
 * Version History:
 * 1.0 - implemented using hal library and changed
 *  		 to callback instead of interrupt
 *
 * Bugs:
 *
 ************************************************/
#include "../BIOS/bios.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SRC_CAN_CAN_H_
#define SRC_CAN_CAN_H_

#define CAN_Id_Standard             ((uint32_t)0x00000000)  /*!< Standard Id */
#define CAN_Id_Extended             ((uint32_t)0x00000004)  /*!< Extended Id */
#define CAN_RTR_Data                ((uint32_t)0x00000000)  /*!< Data frame */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CAN_FilterTypeDef sFilterConfig1;
CAN_FilterTypeDef sFilterConfig2;

CAN_RxHeaderTypeDef CAN1RxMessage,CAN2RxMessage;

enum{
	CAN_500KHz,
	CAN_1MHz
};

void CANxInit(CAN_HandleTypeDef* hcanx,uint16_t prescaler,uint32_t FilterFIFOAssignment,uint16_t CAN_FilterId_11bits,
		uint16_t CAN_FilterMaskId_11bits,uint8_t CAN_FilterNumber, uint8_t rate);

uint32_t CAN_TxMsg(CAN_HandleTypeDef* hcanx,uint32_t StdId_11bits,uint8_t *Msg ,uint8_t len);
uint32_t CAN_TxMsgEID(CAN_HandleTypeDef* hcanx,uint32_t EID,uint8_t *Msg,uint8_t len);
uint32_t CAN_OPEN_TxMsg(CAN_HandleTypeDef* hcanx, uint8_t FunctionCode, uint8_t NodeID, uint8_t *Msg,uint8_t len);
uint32_t CAN_TxRTR(CAN_HandleTypeDef* hcanx,uint32_t StdId_11bits);
uint32_t CAN_OPEN_TxSync(CAN_HandleTypeDef* hcanx);
#ifdef __cplusplus
}
#endif

#endif /* SRC_CAN_CAN_H_ */
