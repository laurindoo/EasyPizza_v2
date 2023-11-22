/*
 * TaskBuzzer.c
 *
 *  Created on: Aug 10, 2023
 *      Author: lucas
 */

#include "TaskBuzzer.h"
void sequencia1Buzzer(void);

extern osThreadId TaskBuzzerHandle;

void StartBuzzer(void const * argument)
{
	/* USER CODE BEGIN StartTaskBuzzer */

	/* Infinite loop */
	for(;;)
	{
		//todo sequencia de piscar até receber o comando de iniciar ou de parar completamente
		sequencia1Buzzer();

		osThreadSuspend(TaskBuzzerHandle);
		osThreadYield();
		osDelay(100);
	}
	/* USER CODE END StartTaskBuzzer */
}

void sequencia1Buzzer(void){
	M_BUZZER_ON
	onOutput(&PrimitiveStates.Lampada);
	osDelay(300);
	M_BUZZER_OFF
	offOutput(&PrimitiveStates.Lampada);
	osDelay(50);
	M_BUZZER_ON
	onOutput(&PrimitiveStates.Lampada);
	osDelay(250);
	M_BUZZER_OFF
	offOutput(&PrimitiveStates.Lampada);

}
