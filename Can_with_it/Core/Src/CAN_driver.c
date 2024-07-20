/*
 * CAN_driver.c
 *
 *  Created on: Jun 15, 2024
 *      Author: verma
 */

#include "CAN_driver.h"

extern CAN_HandleTypeDef hcan1;

extern bmsData bmsDataObj;

char* txFailure = "No data sent through can!\r\n";
char* rxFailure = "NO data received through can!\r\n";
char* canActivationFault = "CAN not activated!\r\n";
char* txMailboxesFull = "Tx mailboxes full!\r\n";

uint8_t canNotification(){
	uint8_t state = 0;

	if (HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		LOGS((uint8_t*)canActivationFault, strlen(canActivationFault));
		state = 1;
	} else {
		state = 0;
	}

	return state;
}

void canTransmit(uint8_t *data){
	CAN_TxHeaderTypeDef txHeader;
	txHeader.DLC = 8;
	txHeader.StdId = 0x65D;
	txHeader.IDE = CAN_ID_STD;
	txHeader.TransmitGlobalTime = DISABLE;
	txHeader.RTR = CAN_RTR_DATA;

	static int counter = 0;

	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){
		LOGS((uint8_t*)txMailboxesFull,strlen(txMailboxesFull));
		char msg[16];
		sprintf(msg,"%d\r\n",counter);
		LOGS((uint8_t*)msg,strlen(msg));
	}

	if ((HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0) && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX1) && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX2)) == 0) {
		uint32_t mailBox;
		counter++;
		char state = HAL_CAN_AddTxMessage(&hcan1, &txHeader, data,&mailBox);
		if (state != HAL_OK) {
			LOGS((uint8_t*)txFailure,strlen(txFailure));
//			Error_Handler();
		}
	}
	else {
		LOGS((uint8_t*)"CBSY\n", 5);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rxHeader;

	canData data[] = {0};

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data->data) != HAL_OK){
		LOGS((uint8_t*)rxFailure,strlen(rxFailure));
	}
	if (rxHeader.ExtId == 0x1806E5F4) {
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	} else if (rxHeader.ExtId == 0x1806E9F4) {
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	} else if (rxHeader.ExtId == 0x1806E7F4) {
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	} else {
		char msg[32];
		sprintf(msg, "%lu\r\n", rxHeader.ExtId);
		LOGS((uint8_t*)msg, strlen(msg));
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
		LOGS((uint8_t*)rxFailure,strlen(rxFailure));
	}
	if (rxHeader.ExtId == 0x1806E5F4) {
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	} else if (rxHeader.ExtId == 0x1806E9F4) {
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	} else if (rxHeader.ExtId == 0x1806E7F4) {
		data->ID = rxHeader.ExtId;
		data->IDE = CAN_ID_EXT;
	} else {
		char msg[32];
		sprintf(msg, "%lu\r\n", rxHeader.ExtId);
		LOGS((uint8_t*)msg, strlen(msg));
	}
	data->DLC = rxHeader.DLC;
	data->Fifo = 0;

	processCanMsg(data);
	uint8_t fill1 = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
	if(fill1 == 0){
		return;
	}
}

uint16_t byteToDecimal(uint8_t *arr) {
    // Assuming len is the length of the array arr
    uint16_t hex_number = 0;

    // Combine first two bytes into a 16-bit hexadecimal number
    hex_number = (arr[0] << 8) | arr[1];

    return hex_number;
}

void processCanMsg(canData *data){
	if(data->ID == 0x1806E5F4){
		char msg[32];
		bmsDataObj.soc = byteToDecimal(data->data);
		sprintf(msg,"%d.%d",bmsDataObj.soc/10,bmsDataObj.soc%10);
		LOGS((uint8_t*)msg,strlen(msg));
	}
	else if(data->ID == 0x1806E7F4){
		char msg[32];
		bmsDataObj.soc = byteToDecimal(data->data);
		sprintf(msg,"%d.%d",bmsDataObj.soc/10,bmsDataObj.soc%10);
		LOGS((uint8_t*)msg,strlen(msg));
	}
	else if (data->ID == 0x1806E9F4) {
		char msg[32];
		bmsDataObj.soc = byteToDecimal(data->data);
		sprintf(msg, "%d.%d\n", bmsDataObj.soc / 10, bmsDataObj.soc % 10);
		LOGS((uint8_t*)msg, strlen(msg));
	}
}


