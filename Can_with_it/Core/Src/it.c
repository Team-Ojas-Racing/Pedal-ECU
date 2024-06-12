/*
 * it.c
 *
 *  Created on: Apr 21, 2024
 *      Author: verma
 */


#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart2;

void SysTickHandler(){
	HAL_InitTick(0);
	HAL_SYSTICK_IRQHandler();
}

