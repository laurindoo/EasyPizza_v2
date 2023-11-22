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

	PrimitiveStates.pwmLastro._pwmValue 	= PIDOutLastro; //incremento para monitor de valores
	PrimitiveStates.pwmTeto._pwmValue 		= PIDOutTeto;	 //incremento para monitor de valores

	htim3.Instance->CCR3 = PIDOutTeto;
	htim3.Instance->CCR4 = PIDOutLastro;
}
void taskTemperatura1sec(void){

	//Verifica chegada em temperatura maxima
	WatchDogLimitesTemperatura();

	//Controla sequencia de luzes
	SaidasLeds();

	//Contador do horimetro
	Horimetro();
}

void WatchDogLimitesTemperatura(void){

	static bool flagMaxLastro,flagMaxTeto;

	//Atingiu limite no teto
	if(PrimitiveStates.RealtimeTeto>PrimitiveStates.LimiteTemp){
		if(!flagMaxTeto){
			flagMaxTeto = 1;
			osMessagePut(FilaEepromHandle, CEepromTempMaxTetoAgain, 0);
		}
	}
	if(flagMaxTeto && PrimitiveStates.RealtimeTeto<PrimitiveStates.LimiteTemp-20){
		flagMaxTeto = 0;
	}

	//Atingiu limite no lastro
	if(PrimitiveStates.RealtimeLastro>PrimitiveStates.LimiteTemp){
		if(!flagMaxLastro){
			flagMaxLastro = 1;
			osMessagePut(FilaEepromHandle, CEepromTempMaxLastroAgain, 0);
		}
	}
	if(flagMaxLastro && PrimitiveStates.RealtimeLastro<PrimitiveStates.LimiteTemp-20){
		flagMaxLastro = 0;
	}
}

void SaidasLeds(void){

	//LED VERDE - indica que as 2 saidas estao ok
	if(PrimitiveStates.pwmLastro._PWMstate != buscando && PrimitiveStates.pwmTeto._PWMstate != buscando ){
		onOutput(&PrimitiveStates.LedVerde);
	}else{
		offOutput(&PrimitiveStates.LedVerde);
	}

	//LED VERMELHO TETO
	if(PrimitiveStates.pwmTeto._PWMstate == buscando){
		onOutput(&PrimitiveStates.LedTeto);
	}else{
		offOutput(&PrimitiveStates.LedTeto);
	}

	//LED VERMELHO LASTRO
	if(PrimitiveStates.pwmLastro._PWMstate == buscando){
		onOutput(&PrimitiveStates.LedLastro);
	}else{
		offOutput(&PrimitiveStates.LedLastro);
	}


}

void Horimetro(void){
	//esperar implementacao digital output
	if(PrimitiveStates.pwmLastro._PWMstate != idle || PrimitiveStates.pwmTeto._PWMstate != idle){
		osMessagePut(FilaEepromHandle, CEepromHorimetro, 0);
	}
}
