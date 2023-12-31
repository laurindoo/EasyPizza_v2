/*
 * TaskBuzzer.c
 *
 *  Created on: Aug 10, 2023
 *      Author: lucas
 */

#include "TaskBuzzer.h"
void sequencia1Buzzer(void);
void sequencia2Buzzer(void);
void waitBuzzerSignal(void);

extern osThreadId TaskBuzzerHandle;

void StartBuzzer(void const * argument)
{
	/* USER CODE BEGIN StartTaskBuzzer */
	sequencia2Buzzer();
	//	sequencia1Buzzer();

	/* Infinite loop */
	for(;;)
	{
		waitBuzzerSignal();
		osThreadYield();

		osDelay(100);
	}
	/* USER CODE END StartTaskBuzzer */
}

void sequencia1Buzzer(void){
	for (int i = 0; i < 10; ++i) {
		M_BUZZER_ON
		onDigital(&PrimitiveStates.Lampada);
		osDelay(250);
		M_BUZZER_OFF
		offDigital(&PrimitiveStates.Lampada);
		osDelay(100);
	}
}
void sequencia2Buzzer(void){
	M_BUZZER_ON
	onDigital(&PrimitiveStates.Lampada);
	osDelay(100);
	M_BUZZER_OFF
	offDigital(&PrimitiveStates.Lampada);
}
void waitBuzzerSignal(void){
	const int32_t signals = SINAL_TEMP_REACH | SINAL_COMFIRMA | SINAL_NEGADO | SINAL_PRONTO | SINAL_CONECTOU;
	osEvent evt;

	// Aguarde até que qualquer um dos sinais seja recebido
	evt = osSignalWait(signals, osWaitForever);

	// Verifique os sinais
	if (evt.status == osEventSignal) {

		if (evt.value.signals & SINAL_TEMP_REACH) {
			onDigital(&PrimitiveStates.Lampada);
			// execute o padrão de buzzer para SINAL_TEMP_REACH
			for (int i = 0; i < 5; ++i) {
				M_BUZZER_ON
				osDelay(50);
				M_BUZZER_OFF
				osDelay(30);
			}
		}

		if (evt.value.signals & SINAL_COMFIRMA) {
			//retorna caso buzzer desabilitado
			if(!PrimitiveStates.Buzzer){
				M_BUZZER_OFF
				return;
			}

			// execute o padrão de buzzer para SINAL_COMFIRMA
			M_BUZZER_ON
			osDelay(70);
			M_BUZZER_OFF
			osDelay(500);
		}

		if (evt.value.signals & SINAL_NEGADO) {
			//retorna caso buzzer desabilitado
			if(!PrimitiveStates.Buzzer){
				M_BUZZER_OFF
				return;
			}

			// execute o padrão de buzzer para SINAL_NEGADO
			for (int i = 0; i < N_REP_SINAL_NEGADO; ++i) {
				M_BUZZER_ON
				osDelay(70);
				M_BUZZER_OFF
				osDelay(50);
			}
		}

		if (evt.value.signals & SINAL_PRONTO) {
			// execute o padrão de buzzer para SINAL_PRONTO
			for (int i = 0; i < N_REP_SINAL_PRONTO; ++i) {
				M_BUZZER_ON
				onDigital(&PrimitiveStates.Lampada);
				osDelay(250);
				M_BUZZER_OFF
				offDigital(&PrimitiveStates.Lampada);
				osDelay(100);
			}
			onDigital(&PrimitiveStates.Lampada);//DEIXA LAMPADA LIGADA
		}

		if (evt.value.signals & SINAL_CONECTOU) {
			//retorna caso buzzer desabilitado
			if(!PrimitiveStates.Buzzer){
				M_BUZZER_OFF
			}else{
				M_BUZZER_ON
				osDelay(30);
				M_BUZZER_OFF
				osDelay(20);
				M_BUZZER_ON
				osDelay(30);
				M_BUZZER_OFF
				osDelay(20);
			}

			onDigital(&PrimitiveStates.LedTeto);
			osDelay(150);
			onDigital(&PrimitiveStates.LedLastro);
			osDelay(150);
			onDigital(&PrimitiveStates.LedVerde);
			osDelay(150);
			offDigital(&PrimitiveStates.LedTeto);
			osDelay(150);
			offDigital(&PrimitiveStates.LedLastro);
			osDelay(150);
			offDigital(&PrimitiveStates.LedVerde);
			osDelay(150);

		}
	}
}
