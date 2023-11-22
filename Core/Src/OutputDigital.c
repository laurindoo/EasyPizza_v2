/*
 * OutputDigital.c
 *
 *  Created on: Nov 11, 2023
 *      Author: lucas
 */
#include "OutputDigital.h"

uint8_t OutputAddComp(OutputDigital* Output,IndviduoOutput* _individ,TypeOutput __tipo,	uint16_t _pinoOUT,GPIO_TypeDef *_portaOUT,
		void (*callback)(),uint16_t limitOn,uint16_t limitOff){

	//Pass the corresponding type
	_individ->_tipo = __tipo;

	//montando a fila
	Output->_OutCommArr[Output->_OutCommCount] = _individ;
	Output->_OutCommCount++;

	//definicao dos pinos
	_individ->GPIO_Pin	=	_pinoOUT;
	_individ->GPIOx		=	_portaOUT;

	//Bind the correct callback functions together
	_individ->timeOut = callback;

	//se houver limite ligado
	_individ->limitOn = limitOn;

	//se houver limite desligado
	_individ->limitOff = limitOff;


	//Return OK
	return 0;
}

void onOutput(IndviduoOutput* outPut) {
	// Implementação do método ON.
	HAL_GPIO_WritePin(outPut->GPIOx, outPut->GPIO_Pin, GPIO_PIN_SET);
	outPut->_state = on; // Exemplo hipotético
}

void offOutput(IndviduoOutput* valv) {
	// Implementação do método OFF.
	HAL_GPIO_WritePin(valv->GPIOx, valv->GPIO_Pin, GPIO_PIN_RESET);
	valv->_state = off; // Exemplo hipotético
}

void contadorOutput(OutputDigital* Output){
	//chamar essa funcao em um timer com passo de 1 segundo
	//Varregura pelas saidas -----------------------------
	for(uint8_t i = 0; i < Output->_OutCommCount; i++)	{

		//--- CONTADORES DIGITAIS
		if(Output->_OutCommArr[i]->_tipo == Digital){
			if(Output->_OutCommArr[i]->_state == on){
				Output->_OutCommArr[i]->timeOff = 0;
				(Output->_OutCommArr[i]->timeOn<UINT16_MAX)?Output->_OutCommArr[i]->timeOn++:0;
			}else{
				Output->_OutCommArr[i]->timeOn = 0;
				(Output->_OutCommArr[i]->timeOff<UINT16_MAX)?Output->_OutCommArr[i]->timeOff++:0;
			}
		}
		//---CONTADORES PID
		else if(Output->_OutCommArr[i]->_tipo == PIDVal){
			if(Output->_OutCommArr[i]->_pwmValue >=80){ 		//BUSCANDO SETPOINT
				Output->_OutCommArr[i]->timeOff = 0;
				(Output->_OutCommArr[i]->timeOn<UINT16_MAX)?Output->_OutCommArr[i]->timeOn++:0;
				Output->_OutCommArr[i]->_PWMstate = buscando;
			}else if(Output->_OutCommArr[i]->_pwmValue > 1){ 	//MANTENDO
				Output->_OutCommArr[i]->_PWMstate = mantendo;
				Output->_OutCommArr[i]->timeOn = 0;
				(Output->_OutCommArr[i]->timeOff<UINT16_MAX)?Output->_OutCommArr[i]->timeOff++:0;
			}else{												//SEM USO
				Output->_OutCommArr[i]->_PWMstate = idle;
				Output->_OutCommArr[i]->timeOn = 0;
				(Output->_OutCommArr[i]->timeOff<UINT16_MAX)?Output->_OutCommArr[i]->timeOff++:0;
			}
		}

		//## --- ANALISE DE POSSIVEIS TIMEOUT ---
		//------ possui limite ligado
		if(Output->_OutCommArr[i]->limitOn != 0){
			if(Output->_OutCommArr[i]->timeOn >= Output->_OutCommArr[i]->limitOn){
				//chama callback de timeout
				Output->_OutCommArr[i]->timeOut();
			}
		}

		//------ possui limite desligado
		if(Output->_OutCommArr[i]->limitOff != 0){
			if(Output->_OutCommArr[i]->timeOff >= Output->_OutCommArr[i]->limitOff){
				//chama callback de timeout
				Output->_OutCommArr[i]->timeOut();
			}
		}
	}
}
