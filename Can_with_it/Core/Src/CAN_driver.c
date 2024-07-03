/*
 * CAN_driver.c
 *
 *  Created on: Jun 15, 2024
 *      Author: verma
 */

#include "CAN_driver.h"

extern CAN_HandleTypeDef hcan1;

extern bmsData bmsDataObj;

char* rxFailure = "NO data received through can!\t\n";

uint8_t canNotification(uint8_t state){
	if(state){
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
	}
	return state;
}

char canTransmit(){
	CAN_TxHeaderTypeDef txHeader;
	txHeader.DLC = 8;
	txHeader.StdId = 0x65D;
	txHeader.IDE = CAN_ID_STD;
	txHeader.TransmitGlobalTime = DISABLE;
	txHeader.RTR = CAN_RTR_DATA;

	int txData[] = {1,2,3,4,5,6,7,8};

	uint32_t pTxMailbox;

	if(HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &pTxMailbox)!=HAL_OK){
		Error_Handler();
	}

	LOGS("msg sent\r\n",10);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rxHeader;

	canData data[] = {0};

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data->data) != HAL_OK){
		LOGS(rxFailure,strlen(rxFailure));
	}
	if(rxHeader.ExtId == 0x1806EEEE){
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	}
	data->DLC = rxHeader.DLC;
	data->Fifo = 0;

	processCanMsg(data);

	uint8_t fill0 = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
	if(fill0 == 0){
		return;
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rxHeader;

	canData data[] = {0};

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data->data) != HAL_OK){
		LOGS(rxFailure,strlen(rxFailure));
	}
	if(rxHeader.ExtId == 0x1806EEEE){
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	}
	data->DLC = rxHeader.DLC;
	data->Fifo = 1;

	processCanMsg(data);

	uint8_t fill1 = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
	if(fill1 == 0){
		return;
	}
}

void processCanMsg(canData *data){
	if(data->ID == 0x1806EEEE){
		bmsDataObj.soc = data->data;
		char msg[16];
		sprintf(msg,"%hu",bmsDataObj.soc);
		LOGS(msg,strlen(msg));
	}
}


