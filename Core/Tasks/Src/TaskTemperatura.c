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
void WatchDogLimitesTemperatura(void);
void SaidasLeds(void);
void Horimetro(void);
void sinalizacaoReachTemp(void);

//handle tasks
extern osThreadId TaskTemperaturaHandle;

//---variaveis PID
double TempTeto, TempLastro, PIDOutTeto, PIDOutLastro;
PID_TypeDef TPIDTeto,TPIDLastro;


void StartTemperatura(void const * argument){

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	initPID();
	for(;;)	{

		computaPID();

		osDelayUntil(&xLastWakeTime,TIME_PID_CALC);
		osThreadYield();

	}
}


void initPID(void){

	PID(&TPIDTeto, 		&PrimitiveStates.Teto.realtime, 	&PrimitiveStates.Teto.PWMOut, 	&PrimitiveStates.Teto.setPoint, 	PrimitiveStates.Teto.kp,PrimitiveStates.Teto.ki, PrimitiveStates.Teto.kd, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(&TPIDLastro, 	&PrimitiveStates.Lastro.realtime, 	&PrimitiveStates.Lastro.PWMOut,	&PrimitiveStates.Lastro.setPoint, 	PrimitiveStates.Lastro.kp,PrimitiveStates.Lastro.ki, PrimitiveStates.Lastro.kd, _PID_P_ON_E, _PID_CD_DIRECT);

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

	//transformar para a classe realizar isso //
	IndviduoPID_SetPWMValueDirect(&PrimitiveStates.Teto, (uint32_t)PrimitiveStates.Teto.PWMOut);
	IndviduoPID_SetPWMValueDirect(&PrimitiveStates.Lastro, (uint32_t)PrimitiveStates.Lastro.PWMOut);

}
void taskTemperatura1sec(void){

	//Verifica chegada em temperatura maxima
	WatchDogLimitesTemperatura();

	//sinalia ao chegar na temperatura
	sinalizacaoReachTemp();

	//Controla sequencia de luzes
	SaidasLeds();

	//Contador do horimetro
	Horimetro();
}
void WatchDogLimitesTemperatura(void){
	static bool flagMaxLastro,flagMaxTeto,flagTimer,flagTimerLastro;

	//Atingiu limite no TETO com incremento a cada ciclo
	if(PrimitiveStates.Teto.realtime>PrimitiveStates.Teto.limite){
		if(!flagMaxTeto){
			flagMaxTeto = 1;
			osMessagePut(FilaEepromHandle, CEepromTempMaxTetoAgain, 0);
		}
		if(!flagTimer && PrimitiveStates.stateTimer == TIMER_decrementando){
			flagTimer = 1;
			osMessagePut(FilaEepromHandle, CEepromTempMaxTetoAgain, 0);
		}
	}
	//cancela flags
	if(flagMaxTeto && PrimitiveStates.Teto.realtime<PrimitiveStates.Teto.limite-20)
		flagMaxTeto = 0;
	if(flagTimer && PrimitiveStates.stateTimer == TIMER_idle)
		flagTimer=0;

	//Atingiu limite no LASTRO com incremento a cada ciclo
	if(PrimitiveStates.Lastro.realtime>PrimitiveStates.Lastro.limite){
		if(!flagMaxLastro){
			flagMaxLastro = 1;
			osMessagePut(FilaEepromHandle, CEepromTempMaxLastroAgain, 0);
		}
		if(!flagTimerLastro && PrimitiveStates.stateTimer == TIMER_decrementando){
			flagTimerLastro = 1;
			osMessagePut(FilaEepromHandle, CEepromTempMaxLastroAgain, 0);
		}
	}
	//cancela flags
	if(flagMaxLastro && PrimitiveStates.Lastro.realtime<PrimitiveStates.Lastro.limite-20)
		flagMaxLastro = 0;
	if(flagTimerLastro && PrimitiveStates.stateTimer == TIMER_idle)
		flagTimerLastro=0;


}
void SaidasLeds(void){

	//LED VERDE - indica que as 2 saidas estao ok
	if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando ){
		onDigital(&PrimitiveStates.LedVerde);
	}else{
		offDigital(&PrimitiveStates.LedVerde);
	}

	//LED VERMELHO TETO
	if(PrimitiveStates.Teto._PWMstate == buscando){
		onDigital(&PrimitiveStates.LedTeto);
	}else{
		offDigital(&PrimitiveStates.LedTeto);
	}

	//LED VERMELHO LASTRO
	if(PrimitiveStates.Lastro._PWMstate == buscando){
		onDigital(&PrimitiveStates.LedLastro);
	}else{
		offDigital(&PrimitiveStates.LedLastro);
	}
}
void Horimetro(void){
	static uint8_t segundos=0;
	//esperar implementacao digital output
	if(PrimitiveStates.Lastro._PWMstate != idle || PrimitiveStates.Teto._PWMstate != idle){
		segundos++;
		if(segundos == 60){
			segundos = 0;
			if(Calendario.Horimetro_parcial_min<59){
				Calendario.Horimetro_parcial_min++;
			}else{
				Calendario.Horimetro_parcial_min=0;
				Calendario.Horimetro_horas++;
			}
			osMessagePut(FilaEepromHandle, CEepromHorimetro, 0);
		}

	}else{
		segundos = 1;
	}
}
void sinalizacaoReachTemp(void){
	static GenericPwmState __StateTeto, __StateLastro;

	//verifica se o estado anterior estava em buscando em qualquer dos dois PID
	if(__StateTeto == buscando || __StateLastro == buscando){
		if(PrimitiveStates.Teto._PWMstate != buscando && PrimitiveStates.Lastro._PWMstate != buscando ){
			//notifica que chegou na temperatura
			osSignalSet(TaskBuzzerHandle, SINAL_TEMP_REACH);
		}
	}
	__StateTeto 	= PrimitiveStates.Teto._PWMstate;
	__StateLastro 	= PrimitiveStates.Lastro._PWMstate;


}
