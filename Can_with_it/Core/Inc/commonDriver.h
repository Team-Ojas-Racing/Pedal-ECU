/*
 * uartDatalogging.h
 *
 *  Created on: Jun 23, 2024
 *      Author: verma
 */

#ifndef INC_COMMONDRIVER_H_
#define INC_COMMONDRIVER_H_

#include "main.h"
#include "CAN_driver.h"

extern UART_HandleTypeDef huart2;

#define 				LOGS(x,y)			HAL_UART_Transmit(&huart2, x, y, 100)

typedef struct{
	uint8_t flag;
}sramLoad;

typedef struct{
	uint8_t soc;
}bmsData;

#endif /* INC_COMMONDRIVER_H_ */
