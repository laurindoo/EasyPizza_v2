/*
 * TaskTemperatura.c
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#define TIME_PID_CALC 500

#include "TaskTemperatura.h"

void initPID(void);
void computaPID(void);

//handle tasks
extern osThreadId TaskTemperaturaHandle;

//---variaveis PID
double TempTeto, TempLastro, PIDOutTeto, PIDOutLastro;
PID_TypeDef TPIDTeto,TPIDLastro;


void StartTemperatura(void const * argument){

	initPID();

	for(;;)	{

		computaPID();
		osThreadYield();

		/*	-Em aquecimento
		 * 		podendo:
		 * 			->resetar timers realtime
		 * 			->no else, vindo do aquecimento gera notificacao de temperatura alcancada	*/
		if(PrimitiveStates.RealtimeTeto<(PrimitiveStates.SetPointTeto)-5 ||
				PrimitiveStates.RealtimeLastro<(PrimitiveStates.SetPointLastro)-5){

			PrimitiveStates.MaquinaAquecimento 	= buscandoTemp;
			PrimitiveStates.stateMaquina 		= aquecendo;

		}else 	if(PrimitiveStates.RealtimeTeto>=PrimitiveStates.SetPointTeto ||
				PrimitiveStates.RealtimeLastro>=PrimitiveStates.SetPointLastro){

			PrimitiveStates.MaquinaAquecimento = mantendoTemp;

		}

		osDelay(TIME_PID_CALC);
	}
}


void initPID(void){

	PID(&TPIDTeto, 		&PrimitiveStates.RealtimeTeto, 		&PIDOutTeto, 	&PrimitiveStates.SetPointTeto, 	30, 0.01, 0.3, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&TPIDLastro, 	&PrimitiveStates.RealtimeLastro, 	&PIDOutLastro, 	&PrimitiveStates.SetPointLastro, 	30, 0.01, 0.3, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_SetMode(&TPIDTeto, 		_PID_MODE_AUTOMATIC);
	PID_SetMode(&TPIDLastro, 	_PID_MODE_AUTOMATIC);

	PID_SetSampleTime(&TPIDTeto, TIME_PID_CALC);
	PID_SetSampleTime(&TPIDLastro, TIME_PID_CALC);

	PID_SetOutputLimits(&TPIDTeto, 0, 100);
	PID_SetOutputLimits(&TPIDLastro, 0, 100);
}
void computaPID(void){

	PID_Compute(&TPIDTeto);
	PID_Compute(&TPIDLastro);

	htim3.Instance->CCR3 = PIDOutTeto;
	htim3.Instance->CCR4 = PIDOutLastro;
}
void taskTemperatura1sec(void){

	//MONITOR DE ERRO DE AQUECIMENTO
	static uint16_t contadorAquecimento;
	if(PrimitiveStates.stateMaquina == aquecendo){

		if(contadorAquecimento>=TIME_MAX_AQUECIMENTO){
			//verifica erro temperatura lastro
			if(PrimitiveStates.RealtimeLastro < PrimitiveStates.SetPointLastro-5)
				PrimitiveStates.Erro.bit.IdleLastro=1;

			//verifica erro temperatura teto
			if(PrimitiveStates.RealtimeTeto < PrimitiveStates.SetPointTeto-5)
				PrimitiveStates.Erro.bit.IdleTeto=1;

		}else{
			contadorAquecimento++;
		}
	}else
		contadorAquecimento=0;
}
