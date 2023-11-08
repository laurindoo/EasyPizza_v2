/*
 * TaskTemperatura.h
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#ifndef INC_TASKTEMPERATURA_H_
#define INC_TASKTEMPERATURA_H_

#include "main.h"//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
void Temperatura1sec(void);

#include <stdio.h>
//#include "pid.h"

//---CALDEIRA
#define MACRO_LIGA_CALDEIRA		HAL_GPIO_WritePin 	(GPIOB,	GPIO_PIN_1,	GPIO_PIN_SET);
#define MACRO_DESLIGA_CALDEIRA	HAL_GPIO_WritePin 	(GPIOB,	GPIO_PIN_1,	GPIO_PIN_RESET);
#define MLigaCaldeira		PrimitiveStates.Caldeira = true; \
		MACRO_LIGA_CALDEIRA
#define MDesligaCaldeira	PrimitiveStates.Caldeira = false; \
		MACRO_DESLIGA_CALDEIRA




#endif /* INC_TASKTEMPERATURA_H_ */
