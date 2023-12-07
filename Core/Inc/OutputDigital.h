/*
 * OutputDigital.h
 *
 *  Created on: Nov 11, 2023
 *      Author: lucas
 */

/*
 *
 * 1 - crie OutputDigital outPuts; -> main.c
 *
 *
 * */

#ifndef INC_OUTPUTDIGITAL_H_
#define INC_OUTPUTDIGITAL_H_

//Include libraries

#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "string.h"


/*
 * Tipos de saida
 */
typedef enum
{   Digital,
	PIDVal, 		// nao implementada
	Analogica,	// nao implementada
} TypeOutput;

/*
 * Estado de saida
 */
typedef enum
{   off = 0,
	on,
} GenericState;

/*
 * Estado de pwm
 */
typedef enum
{   idle = 0,
	buscando,
	mantendo,
} GenericPwmState;


/*
 * Estrutura saida
 */
typedef struct
{
	uint16_t 			GPIO_Pin;
	GPIO_TypeDef* 		GPIOx;

	GenericState	_state;

	uint16_t 		timeOn;
	uint16_t 		timeOff;

	uint16_t		limitOn;
	uint16_t		limitOff;

	void (*timeOut)();

}IndviduoOutput;

typedef struct
{
	//handle do timer
	TIM_HandleTypeDef 	*TimHandle;
	uint32_t 			Channel;

	//PID
	double 		kp;
	double 		ki;
	double 		kd;
	double 		PWMOut;

	//var values
	double 			realtime;
	double			setPoint;
	uint16_t		limite;
	uint16_t		histerese;
	GenericPwmState _PWMstate;

	//timeout e callback
	uint16_t		timeOn;
	uint16_t		limiteOn;
	void (*timeOut)();

}IndviduoPID;


/*
 * Conjunto saidas
 */
typedef struct
{
	//Variavel para receber lista de comandos
	IndviduoOutput* _OutDigitalArr[6];
	IndviduoPID* _OutPidArr[4];
	uint8_t _DigitalCount;
	uint8_t _PidCount;

}OutputDigital;

uint8_t OutputAddDigital(OutputDigital* Output, IndviduoOutput* _individ, uint16_t _pinoOUT, GPIO_TypeDef *_portaOUT, void (*callback)(),uint16_t limitOn,uint16_t limitOff);
uint8_t OutputAddPID(OutputDigital* Output,IndviduoPID* _individ, TIM_HandleTypeDef *htim, uint32_t Channel, double Kp, double Ki, double Kd, uint16_t histerese,uint16_t limit_on,void (*callback)());
void onDigital(IndviduoOutput* outPut);
void offDigital(IndviduoOutput* outPut);
void contadorOutput(OutputDigital* Output);
void IndviduoPID_SetPWMValue(IndviduoPID *pid, double pwmValue);
void IndviduoPID_SetPWMValueDirect(IndviduoPID *pid, uint32_t pwmValue) ;

//extern OutputDigital outPuts;

#endif /* INC_OUTPUTDIGITAL_H_ */
