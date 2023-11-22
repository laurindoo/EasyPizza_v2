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
	TypeOutput 		_tipo;

	uint16_t 		GPIO_Pin;
	GPIO_TypeDef* 	GPIOx;

	GenericPwmState _PWMstate;
	double			_pwmValue;

	GenericState	_state;

	uint16_t 		timeOn;
	uint16_t 		timeOff;

	uint16_t		limitOn;
	uint16_t		limitOff;

	void (*timeOut)();

}IndviduoOutput;


/*
 * Conjunto saidas
 */
typedef struct
{
	//Variavel para receber lista de comandos
	IndviduoOutput* _OutCommArr[6];
	uint8_t _OutCommCount;

}OutputDigital;

uint8_t OutputAddComp(OutputDigital* Output, IndviduoOutput* _individ, TypeOutput __tipo, uint16_t _pinoOUT, GPIO_TypeDef *_portaOUT, void (*callback)(),uint16_t limitOn,uint16_t limitOff);
void onOutput(IndviduoOutput* outPut);
void offOutput(IndviduoOutput* valv);
void contadorOutput(OutputDigital* Output);

//extern OutputDigital outPuts;

#endif /* INC_OUTPUTDIGITAL_H_ */
