/*
 * main_app.c
 *
 *  Created on: Apr 21, 2024
 *      Author: verma
 */

#include <string.h>
#include "stm32l4xx.h"
#include "main_app.h"
#include "stm32l476xx.h"
#include <stdio.h>

#define TRUE 1
#define FALSE 0

void SystemClock_Config();
void UART2_Init();
void gpio_init();
void CAN1_init();
void CAN1_TX();
void CAN1_RX();
void CAN1_FilterConfig();
void Error_handler();
void DeviationCheck();
uint32_t ADCOutput();

//ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
CAN_FilterTypeDef sFilterConfig;

char *user_data = "The application is running\r\n";

uint8_t data_buffer[100];
uint8_t recvd_data;
uint32_t count=0;
uint8_t reception_complete = FALSE;

int main(){
	HAL_Init();
	CAN1_init();
	SystemClock_Config();

	UART2_Init();

//	HAL_ADC_Init(&hadc1);

	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK){
		Error_handler();
	}


	if( HAL_CAN_Start(&hcan1) != HAL_OK){
		Error_handler();
	}

	CAN1_TX();

	return 0;
}

void gpio_init(){
	GPIO_InitTypeDef adc;

	adc.Pin = GPIO_PIN_0|GPIO_PIN_4;
	adc.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	adc.Pull = GPIO_NOPULL;
	adc.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &adc);
}

void UART2_Init(){
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if(HAL_UART_Init(&huart2)!= HAL_OK){
		Error_handler();
	}

}

void CAN1_init(){
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	//bit timings based on calculator http://www.bittiming.can-wiki.info/
	hcan1.Init.Prescaler = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

	if(HAL_CAN_Init(&hcan1) != HAL_OK){
		Error_handler();
	}
}

//clock configuration for 40 mhz
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    Error_handler();
	  }

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 1;
	  RCC_OscInitStruct.PLL.PLLN = 20;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//	  {
//	    Error_handler();
//	  }
	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

//specific tx function
void CAN1_TX(){
	CAN_TxHeaderTypeDef TxHeader;

	char msg[50];
	uint32_t TxMailbox;
	uint8_t data[5] = {1,2,3,4,5};


	TxHeader.DLC = 5;
	TxHeader.StdId = 0x65D;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader,data, &TxMailbox) != HAL_OK){
		Error_handler();
	}

	while( HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));

	sprintf(msg,"Message Transmitted\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

//void CAN1_RX(){
//	CAN_RxHeaderTypeDef RxHeader;
//
//	while(!HAL_CAN_GetRxFifoFillLevel(hcan1, RxFifo));
//
//	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)!= HAL_OK){
//		Error_Handler();
//	}
//}

void CAN1_FilterConfig(){
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x245<<5; //the ID that the filter looks for (switch this for the other microcontroller)
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

void Error_handler(){
	while(1);
}
