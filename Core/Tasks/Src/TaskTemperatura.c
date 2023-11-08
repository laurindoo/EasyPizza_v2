/*
 * TaskTemperatura.c
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#define TIME_PID_CALC 500

#include "TaskTemperatura.h"

float GetTemp(double * PIDOut);
void initPID(void);
void computaPID(void);

//handle tasks
extern osThreadId TaskTemperaturaHandle;

//---variaveis PID
double TempTeto, TempLastro, PIDOutTeto, PIDOutLastro, TempSPTeto, TempSPLastro;
PID_TypeDef TPIDTeto,TPIDLastro;


void StartTemperatura(void const * argument){

	initPID();

	//	osThreadSuspend(TaskTemperaturaHandle);
	for(;;)	{

		computaPID();
		osThreadYield();
		osDelay(TIME_PID_CALC);
	}
}

float GetTemp(double * PIDOut){
	/*
	 * Simulacao temperatura usando saida do PID
	 */

	static float tmp = 0;

	tmp += (*PIDOut / 1000);

	return tmp;
}
void initPID(void){

	TempSPTeto 		= 0;
	TempSPLastro 	= 0;

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
void Temperatura1sec(void){

}
