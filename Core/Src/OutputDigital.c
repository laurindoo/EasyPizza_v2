/*
 * OutputDigital.c
 *
 *  Created on: Nov 11, 2023
 *      Author: lucas
 */
#include "OutputDigital.h"

uint8_t OutputAddDigital(OutputDigital* Output,IndviduoOutput* _individ, uint16_t _pinoOUT,GPIO_TypeDef *_portaOUT,
		void (*callback)(),uint16_t limitOn,uint16_t limitOff){

	//montando a fila
	Output->_OutDigitalArr[Output->_DigitalCount] = _individ;
	Output->_DigitalCount++;

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

uint8_t OutputAddPID(OutputDigital* Output,IndviduoPID* _individ, TIM_HandleTypeDef *htim, uint32_t Channel, double Kp, double Ki, double Kd, uint16_t histerese,
		uint16_t limit_on,void (*callback)()){

	//montando a fila
	Output->_OutPidArr[Output->_PidCount] = _individ;
	Output->_PidCount++;

	//definicao do timer
	_individ->TimHandle = htim;
	_individ->Channel   = Channel;

	//tunning de PID
	_individ->kp	= Kp;
	_individ->ki	= Ki;
	_individ->kd	= Kd;
	_individ->histerese	= histerese;

	//Bind the correct callback functions together
	_individ->timeOut = callback;

	//Return OK
	return 0;
}

void onDigital(IndviduoOutput* outPut) {
	// Implementação do método ON.
	HAL_GPIO_WritePin(outPut->GPIOx, outPut->GPIO_Pin, GPIO_PIN_SET);
	outPut->_state = on; // Exemplo hipotético
}

void offDigital(IndviduoOutput* outPut) {
	// Implementação do método OFF.
	HAL_GPIO_WritePin(outPut->GPIOx, outPut->GPIO_Pin, GPIO_PIN_RESET);
	outPut->_state = off; // Exemplo hipotético
}

void contadorOutput(OutputDigital* Output){
	//chamar essa funcao em um timer com passo de 1 segundo

	//Varregura pelas saidas -------------DIGITAIS----------------
	for(uint8_t i = 0; i < Output->_DigitalCount; i++)	{

		//--- CONTADORES DIGITAIS
		if(Output->_OutDigitalArr[i]->_state == on){
			Output->_OutDigitalArr[i]->timeOff = 0;
			(Output->_OutDigitalArr[i]->timeOn<UINT16_MAX)?Output->_OutDigitalArr[i]->timeOn++:0;
		}else{
			Output->_OutDigitalArr[i]->timeOn = 0;
			(Output->_OutDigitalArr[i]->timeOff<UINT16_MAX)?Output->_OutDigitalArr[i]->timeOff++:0;
		}

		//## --- ANALISE DE POSSIVEIS TIMEOUT ---
		//------ possui limite ligado
		if(Output->_OutDigitalArr[i]->limitOn != 0){
			if(Output->_OutDigitalArr[i]->timeOn >= Output->_OutDigitalArr[i]->limitOn){
				//chama callback de timeout
				Output->_OutDigitalArr[i]->timeOut();
			}
		}

		//------ possui limite desligado
		if(Output->_OutDigitalArr[i]->limitOff != 0){
			if(Output->_OutDigitalArr[i]->timeOff >= Output->_OutDigitalArr[i]->limitOff){
				//chama callback de timeout
				Output->_OutDigitalArr[i]->timeOut();
			}
		}
	}

	//Varregura pelas saidas -------------PID----------------
	for(uint8_t i = 0; i < Output->_PidCount; i++)	{

//#define CALCULA_POR_PWMOUT

#ifdef CALCULA_POR_PWMOUT
		//---CATEGORIZA STATE
		if(Output->_OutPidArr[i]->PWMOut == 0 ){
					Output->_OutPidArr[i]->_PWMstate = idle;
		}else if(Output->_OutPidArr[i]->PWMOut <= Output->_OutPidArr[i]->histerese){
			Output->_OutPidArr[i]->_PWMstate = mantendo;
		}else
			Output->_OutPidArr[i]->_PWMstate = buscando;
#endif
#ifndef CALCULA_POR_PWMOUT
		//---CATEGORIZA STATE
		if(Output->_OutPidArr[i]->realtime >=  Output->_OutPidArr[i]->setPoint ){
					Output->_OutPidArr[i]->_PWMstate = idle;
		}else if(Output->_OutPidArr[i]->realtime + Output->_OutPidArr[i]->histerese >  Output->_OutPidArr[i]->setPoint ){
			Output->_OutPidArr[i]->_PWMstate = mantendo;
		}else
			Output->_OutPidArr[i]->_PWMstate = buscando;
#endif
		//---CONTADORES PID
		if(Output->_OutPidArr[i]->_PWMstate == mantendo){
			(Output->_OutPidArr[i]->timeOn<UINT16_MAX)?Output->_OutPidArr[i]->timeOn++:0;
		}else{
			Output->_OutPidArr[i]->timeOn=0;
		}

		//## --- ANALISE DE POSSIVEIS TIMEOUT ---
		//------ possui limite ligado
		if(Output->_OutPidArr[i]->limiteOn != 0){
			if(Output->_OutPidArr[i]->timeOn >= Output->_OutPidArr[i]->limiteOn){
				//chama callback de timeout
				Output->_OutPidArr[i]->timeOut();
			}
		}
	}
}

void IndviduoPID_SetPWMValue(IndviduoPID *pid, double pwmValue) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwmValue; // O valor do duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(pid->TimHandle, &sConfigOC, pid->Channel) != HAL_OK) {
        // Tratamento de erro
    }

    if (HAL_TIM_PWM_Start(pid->TimHandle, pid->Channel) != HAL_OK) {
        // Tratamento de erro
    }
}

void IndviduoPID_SetPWMValueDirect(IndviduoPID *pid, uint32_t pwmValue) {
    // diretamente acessando o registro de comparação do canal apropriado
    // Esta é uma abordagem mais arriscada e assume que você sabe o que está fazendo
    volatile uint32_t *ccrAddress = &pid->TimHandle->Instance->CCR1 + (pid->Channel >> 2);
    *ccrAddress = pwmValue;
}
