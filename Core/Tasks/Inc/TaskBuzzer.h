/*
 * TaskBuzzer.h
 *
 *  Created on: Aug 10, 2023
 *      Author: lucas
 */

#ifndef SRC_TASKS_TASKBUZZER_H_
#define SRC_TASKS_TASKBUZZER_H_

#include "main.h"//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define M_BUZZER_ON HAL_TIM_PWM_Start	(&htim2,TIM_CHANNEL_4);
//#define M_BUZZER_ON HAL_TIM_PWM_Stop	(&htim2,TIM_CHANNEL_4);
#define M_BUZZER_OFF HAL_TIM_PWM_Stop	(&htim2,TIM_CHANNEL_4);

#endif /* SRC_TASKS_TASKBUZZER_H_ */
