/*
 * TaskReceita.c
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#include "TaskTimer.h"


void funcionamentoTimer(void);
void verificaErro(void);


//handle para a tarefa de receita
extern osThreadId TaskTimerHandle;
extern osThreadId TaskBuzzerHandle;
//fila de envio TX ble
extern osMessageQId FilaEepromHandle;

extern double TempTeto, TempLastro, PIDOutTeto, PIDOutLastro, TempSPTeto, TempSPLastro;

//variaveis globais
extern GlobalPrimitiveIOStates PrimitiveStates;


void StartTimer(void const * argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		verificaErro();
		funcionamentoTimer();
		osThreadYield();
		osDelayUntil(&xLastWakeTime,1000);
	}
}

void funcionamentoTimer(void){

	//Decremento de minutos e segundos
	if((PrimitiveStates.RTTimerMinutos>0 && PrimitiveStates.RTTimerSegundos==0) && PrimitiveStates.stateTimer == TIMER_decrementando){
		PrimitiveStates.RTTimerSegundos = 59;
		PrimitiveStates.RTTimerMinutos--;
	}else if((PrimitiveStates.RTTimerMinutos>0 || PrimitiveStates.RTTimerSegundos>0) && PrimitiveStates.stateTimer == TIMER_decrementando){
		PrimitiveStates.RTTimerSegundos--;

		//chegou ao zero --- ROTINA DE FIM DE CICLO
		if(PrimitiveStates.RTTimerSegundos==0 && PrimitiveStates.RTTimerMinutos==0 && PrimitiveStates.stateTimer != TIMER_idle){
			Calendario.TotalCiclos=Calendario.TotalCiclos+1;
			osMessagePut(FilaEepromHandle, CEepromNewCile, 0);
			//notifica buzzer
			osSignalSet(TaskBuzzerHandle, SINAL_PRONTO);
			PrimitiveStates.stateTimer = TIMER_idle;
			osThreadResume(TaskBuzzerHandle);
		}
	}
}


void verificaErro(void){
	osEvent evt;
	evt = osSignalWait (ERRO_CRITICO, 0);
	if (evt.status == osEventSignal){

		if(evt.value.v == ERRO_CRITICO){
			osThreadSuspend(TaskTimerHandle);
		}
	}
}




