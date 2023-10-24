/*
 * TaskBuzzer.c
 *
 *  Created on: Aug 10, 2023
 *      Author: lucas
 */

#include "TaskBuzzer.h"
void sequencia1Buzzer(void);

extern osThreadId TaskBuzzerHandle;
extern TIM_HandleTypeDef htim2;

void StartBuzzer(void const * argument)
{
	/* USER CODE BEGIN StartTaskBuzzer */
	//osThreadSuspend(TaskBuzzerHandle);
	/* Infinite loop */
	for(;;)
	{
		sequencia1Buzzer();

		osThreadSuspend(TaskBuzzerHandle);

		osDelay(100);
	}
	/* USER CODE END StartTaskBuzzer */
}

void sequencia1Buzzer(void){
	M_BUZZER_ON
	HAL_Delay(300);
	M_BUZZER_OFF
	HAL_Delay(50);
	M_BUZZER_ON
	HAL_Delay(250);
	M_BUZZER_OFF
}
