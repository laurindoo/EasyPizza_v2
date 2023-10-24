/*
 * TaskReceita.c
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#include "TaskTimer.h"
//#include "TaskEeprom.h"


void funcionamentoTimer(void);
void funcionamentoLampada(void);
void verificaErro(void);


//handle para a tarefa de receita
extern osThreadId TaskTimerHandle;
extern osThreadId TaskBuzzerHandle;
//fila de envio TX ble
extern osMessageQId FilaTXBluetoothHandle,FilaEepromHandle;

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

		funcionamentoLampada();

		osDelayUntil(&xLastWakeTime,1000);
	}
}

void funcionamentoTimer(void){

	//Decremento de minutos e segundos
	if((PrimitiveStates.RTTimerMinutos>0 && PrimitiveStates.RTTimerSegundos==0) && PrimitiveStates.stateTimer){
		PrimitiveStates.RTTimerSegundos = 59;
		PrimitiveStates.RTTimerMinutos--;
	}else if((PrimitiveStates.RTTimerMinutos>0 || PrimitiveStates.RTTimerSegundos>0) && PrimitiveStates.stateTimer){
		PrimitiveStates.RTTimerSegundos--;

		//chegou ao zero --- ROTINA DE FIM DE CICLO
		if(PrimitiveStates.RTTimerSegundos==0 && PrimitiveStates.RTTimerMinutos==0){
			PrimitiveStates.SetPointTeto 	= 0;
			PrimitiveStates.SetPointLastro 	= 0;
			PrimitiveStates.SetPointTeto	= 0;
			PrimitiveStates.SetPointLastro	= 0;
			osThreadResume(TaskBuzzerHandle);
		}
	}
}

void funcionamentoLampada(void){
	osEvent evt;
	//notificacao via piscar de lamapda
	evt = osSignalWait (PISCADA_LAMPADA, 0);
	if (evt.status == osEventSignal){

		if(evt.value.v == PISCADA_LAMPADA){
			//TODO LÓGICA DE PISCADA DE LAMPADA
		}
	}

	//decremento e apos desligamento lampada
	if(PrimitiveStates.SegundosLampada>0){
		PrimitiveStates.SegundosLampada--;
		if(PrimitiveStates.SegundosLampada==0){
			LAMPADA_OFF
		}else{
			LAMPADA_ON
		}
	}
}

void verificaErro(void){
	osEvent evt;
	evt = osSignalWait (ERRO_CRITICO, 0);
	if (evt.status == osEventSignal){

		if(evt.value.v == ERRO_CRITICO){
			//TODO REVISAR
			osThreadSuspend(TaskTimerHandle);
		}
	}
}




