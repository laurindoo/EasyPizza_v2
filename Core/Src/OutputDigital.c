/*
 * OutputDigital.c
 *
 *  Created on: Nov 11, 2023
 *      Author: lucas
 */
#include "OutputDigital.h"

OUTPUT_ErrorCode OutputAddDigital(OutputDigital* Output,IndviduoOutput* _individ, uint16_t _pinoOUT,GPIO_TypeDef *_portaOUT,
		void (*callback)(),uint16_t limitOn,uint16_t limitOff){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (Output == NULL || _individ == NULL || _portaOUT == NULL ) {
		outputError_Handler(OUTPUT_OBJETO_NULO);
		return OUTPUT_OBJETO_NULO;
	}

	// apenas um limite aceitavel.
	if(limitOn && limitOff){
		outputError_Handler(OUTPUT_MORE_LIMITS);
		return OUTPUT_MORE_LIMITS;
	}

	// preenchendo objeto.
	_individ->GPIO_Pin	= _pinoOUT;		// pino a ser controlado.
	_individ->GPIOx		= _portaOUT;	// porta a ser controlada.
	_individ->timeOut 	= callback;		// callback caso extrapole qualquer um dos limites.
	_individ->limitOn 	= limitOn;		// se houver limite ligado.
	_individ->limitOff 	= limitOff;		// se houver limite desligado.

	// montando lista.
	Output->_OutDigitalArr[Output->_DigitalCount] = _individ;
	Output->_DigitalCount++;

	return OUTPUT_SUCCESS;
}
uint8_t OutputAddPID(OutputDigital* Output,IndviduoPID* _individ, TIM_HandleTypeDef *htim, uint32_t Channel, double Kp, double Ki, double Kd, uint16_t histerese,
		uint16_t limit_on,void (*callback)()){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (Output == NULL || _individ == NULL || htim == NULL ) {
		outputError_Handler(OUTPUT_OBJETO_NULO);
		return OUTPUT_OBJETO_NULO;
	}

	// preenchendo objeto.
	_individ->TimHandle = htim;		// definicao do timer.
	_individ->Channel   = Channel;	// channel do timer.
	_individ->kp	= Kp;
	_individ->ki	= Ki;
	_individ->kd	= Kd;
	_individ->histerese	= histerese;// histerese dada em graus.
	_individ->timeOut = callback;	// callback caso extrapole tempo de acionamento.

	// montando lista.
	Output->_OutPidArr[Output->_PidCount] = _individ;
	Output->_PidCount++;

	IndviduoPID_SetPWMValue(_individ, 0);

	return OUTPUT_SUCCESS;
}
void onDigital(IndviduoOutput* outPut) {
	// Implementação do método ON.
	HAL_GPIO_WritePin(outPut->GPIOx, outPut->GPIO_Pin, GPIO_PIN_SET);
	outPut->_state = on;
}
void offDigital(IndviduoOutput* outPut) {
	// Implementação do método OFF.
	HAL_GPIO_WritePin(outPut->GPIOx, outPut->GPIO_Pin, GPIO_PIN_RESET);
	outPut->_state = off;
}
void contadorOutput(OutputDigital* Output){
	// todo criar arquivo de instruçoes: chamar essa funcao em um timer com passo de 1 segundo.

	// varredura saidas digitais.
	for(uint8_t i = 0; i < Output->_DigitalCount; i++)	{

		// processamento dos contadores digitais.
		if(Output->_OutDigitalArr[i]->_state == on){
			Output->_OutDigitalArr[i]->timeOff = 0;
			(Output->_OutDigitalArr[i]->timeOn<UINT16_MAX)?Output->_OutDigitalArr[i]->timeOn++:0;
		}else{
			Output->_OutDigitalArr[i]->timeOn = 0;
			(Output->_OutDigitalArr[i]->timeOff<UINT16_MAX)?Output->_OutDigitalArr[i]->timeOff++:0;
		}

		// verifica se atingiu timeout ligado.
		if(Output->_OutDigitalArr[i]->limitOn != 0){
			if(Output->_OutDigitalArr[i]->timeOn >= Output->_OutDigitalArr[i]->limitOn){
				Output->_OutDigitalArr[i]->timeOut();//chama callback de timeout.
			}
		}

		// verifica se atingiu timeout desligado.
		if(Output->_OutDigitalArr[i]->limitOff != 0){
			if(Output->_OutDigitalArr[i]->timeOff >= Output->_OutDigitalArr[i]->limitOff){
				Output->_OutDigitalArr[i]->timeOut();//chama callback de timeout.
			}
		}
	}

	// varredura saidas PID.
	for(uint8_t i = 0; i < Output->_PidCount; i++)	{

		//#define CALCULA_POR_PWMOUT
#ifdef CALCULA_POR_PWMOUT
		// processa state do item.
		if(Output->_OutPidArr[i]->PWMOut == 0 ){
			Output->_OutPidArr[i]->_PWMstate = idle;
		}else if(Output->_OutPidArr[i]->PWMOut <= Output->_OutPidArr[i]->histerese){
			Output->_OutPidArr[i]->_PWMstate = mantendo;
		}else
			Output->_OutPidArr[i]->_PWMstate = buscando;
#endif
#ifndef CALCULA_POR_PWMOUT
		// processa state do item.
		if(Output->_OutPidArr[i]->realtime >=  Output->_OutPidArr[i]->setPoint ){
			Output->_OutPidArr[i]->_PWMstate = idle;
		}else if(Output->_OutPidArr[i]->realtime + Output->_OutPidArr[i]->histerese >  Output->_OutPidArr[i]->setPoint ){
			Output->_OutPidArr[i]->_PWMstate = mantendo;
		}else
			Output->_OutPidArr[i]->_PWMstate = buscando;
#endif
		// processamento dos contadores PID.
		if(Output->_OutPidArr[i]->_PWMstate == mantendo){
			(Output->_OutPidArr[i]->timeOn<UINT16_MAX)?Output->_OutPidArr[i]->timeOn++:0;
		}else{
			Output->_OutPidArr[i]->timeOn=0;
		}

		// verifica se atingiu timeout ligado.
		if(Output->_OutPidArr[i]->limiteOn != 0){
			if(Output->_OutPidArr[i]->timeOn >= Output->_OutPidArr[i]->limiteOn){
				Output->_OutPidArr[i]->timeOut();//chama callback de timeout
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
		outputError_Handler(OUTPUT_TIMERSET_ERROR);
	}

	if (HAL_TIM_PWM_Start(pid->TimHandle, pid->Channel) != HAL_OK) {
		// Tratamento de erro
		outputError_Handler(OUTPUT_TIMERSTART_ERROR);
	}
}
void IndviduoPID_SetPWMValueDirect(IndviduoPID *pid, uint32_t pwmValue) {
	// diretamente acessando o registro de comparação do canal apropriado
	volatile uint32_t *ccrAddress = &pid->TimHandle->Instance->CCR1 + (pid->Channel >> 2);
	*ccrAddress = pwmValue;
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void outputError_Handler(OUTPUT_ErrorCode erro)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
