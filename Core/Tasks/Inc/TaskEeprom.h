/*
 * TaskEeprom.h
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#ifndef INC_TASKEEPROM_H_
#define INC_TASKEEPROM_H_

#include "main.h"//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Eeprom.h"

extern osThreadId TaskEepromHandle;
extern osMessageQId FilaEepromHandle;
extern osThreadId TaskTemperaturaHandle;




#endif /* INC_TASKEEPROM_H_ */
