/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define ERRO_CRITICO 0x0f
#define PISCADA_LAMPADA 0x1f
#define CHEGOU_ADDR_BLE 0xbf

#define DMA_RX_BUFFER_SIZE      64
typedef struct DMA_SERIAL_Struct{
	uint8_t			DMA_RX_Buffer	[DMA_RX_BUFFER_SIZE]	;	//BUFFER Endereco_central VIA UART
	uint8_t			DMA_RX_size					;	//TAMANHO DA INFORMACAO RECEBIDA

}DMA_SERIAL_Struct;//TODO TRANSFORMAR EM CLASSE

//---ESTRUTURA AGRUPAMENTO DE 8 BITS EM 1 BYTE-------------------------------------
typedef union BIT_TO_BYTE_ERROS
{
	struct
	{ //todo revisar
		unsigned char TimeoutTeto		: 1;
		unsigned char TimeoutLastro		: 1;
		unsigned char IdleTeto			: 1;
		unsigned char IdleLastro		: 1;
		unsigned char eeprom			: 1;
		unsigned char aux2				: 1;
		unsigned char aux3	 			: 1;
		unsigned char aux4				: 1;

	}bit;
	unsigned char byte;
}BIT_TO_BYTE_ERROS;

//---ESTRUTURA VARIAVEIS Calendario ------------------------------------
typedef struct TYPE_CALENDARIO{

	//	RTC_DateTypeDef 		Data_instalacao;
	uint16_t				Horimetro_horas;		//total de horas da maquina
	uint8_t					Horimetro_parcial_min; 	//a cada minuto eu incremento, comparo com o gravado, e hora++ se for o caso

}
TYPE_CALENDARIO;//

//---Comandos Eeprom
typedef enum
{   CEepromShifiting,
	CEepromHorimetro,
	CEepromDataInstalacao,
	CEepromHardReset,
	CEepromSoftReset,
	CEepromAtualizaHora,
} ComandosEeprom;

//---Estados da Aplicacao
typedef enum
{   sInicio = 0,
	sManual,
	sInteligente,
	sPronta,
	sPreparandoManual,
	sPreparandoInteligente,
	sError,
	sPizzaPronta,
} state_t;

//---Estrutura variaveis principais
typedef struct
{
	bool 		ConectionBle;
	bool 		Teto; 		//on/off
	bool 		Lastro;		//on/off
	bool 		Lampada;	//on/off
	bool 		Led1;		//on/off
	bool 		Led2;
	uint8_t 	Receita; //Definido em @AcaoReceita | possiveis estados para receita
	double 		RealtimeTeto;
	double 		RealtimeLastro;
	double		SetPointTeto;
	double  	SetPointLastro;
	state_t 	MaquinaMaster;
	uint8_t		SPTimerMinutos;
	uint8_t		SPTimerSegundos;
	uint8_t		RTTimerMinutos;
	uint8_t		RTTimerSegundos;
	uint8_t		SegundosLampada;
	bool		stateTimer;

}GlobalPrimitiveIOStates;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define RELE_1_ON 	HAL_GPIO_WritePin		(RELE_1_GPIO_Port,	RELE_1_Pin	,GPIO_PIN_SET);
#define RELE_1_OFF 	HAL_GPIO_WritePin		(RELE_1_GPIO_Port,	RELE_1_Pin	,GPIO_PIN_RESET);
#define TESTE_RELE_1	  RELE_1_ON \
		HAL_Delay(100);\
		RELE_1_OFF

#define RELE_2_ON 	HAL_GPIO_WritePin		(RELE_2_GPIO_Port,	RELE_2_Pin	,GPIO_PIN_SET);
#define RELE_2_OFF 	HAL_GPIO_WritePin		(RELE_2_GPIO_Port,	RELE_2_Pin	,GPIO_PIN_RESET);
#define TESTE_RELE_2	  RELE_2_ON \
		HAL_Delay(100);\
		RELE_2_OFF

#define RELE_3_ON 	HAL_GPIO_WritePin		(RELE_3_GPIO_Port,	RELE_3_Pin	,GPIO_PIN_SET);
#define RELE_3_OFF 	HAL_GPIO_WritePin		(RELE_3_GPIO_Port,	RELE_3_Pin	,GPIO_PIN_RESET);
#define TESTE_RELE_3	  RELE_3_ON \
		HAL_Delay(100);\
		RELE_3_OFF

#define RELE_4_ON 	HAL_GPIO_WritePin		(RELE_4_GPIO_Port,	RELE_4_Pin	,GPIO_PIN_SET);
#define RELE_4_OFF 	HAL_GPIO_WritePin		(RELE_4_GPIO_Port,	RELE_4_Pin	,GPIO_PIN_RESET);
#define TESTE_RELE_4	  RELE_4_ON \
		HAL_Delay(100);\
		RELE_4_OFF

#define RELE_5_ON 	HAL_GPIO_WritePin		(RELE_5_GPIO_Port,	RELE_5_Pin	,GPIO_PIN_SET);
#define RELE_5_OFF 	HAL_GPIO_WritePin		(RELE_5_GPIO_Port,	RELE_5_Pin	,GPIO_PIN_RESET);
#define TESTE_RELE_5	  RELE_5_ON \
		HAL_Delay(100);\
		RELE_5_OFF

#define LAMPADA_ON RELE_5_ON\
		PrimitiveStates.Lampada = true;
#define LAMPADA_OFF RELE_5_OFF\
		PrimitiveStates.SegundosLampada = 0;\
		PrimitiveStates.Lampada = false;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLE_EN_Pin GPIO_PIN_13
#define BLE_EN_GPIO_Port GPIOB
#define BLE_RESET_Pin GPIO_PIN_14
#define BLE_RESET_GPIO_Port GPIOB
#define BLE_STATUS_Pin GPIO_PIN_15
#define BLE_STATUS_GPIO_Port GPIOB
#define BOTAO_BLE_Pin GPIO_PIN_12
#define BOTAO_BLE_GPIO_Port GPIOA
#define RELE_1_Pin GPIO_PIN_15
#define RELE_1_GPIO_Port GPIOA
#define RELE_2_Pin GPIO_PIN_3
#define RELE_2_GPIO_Port GPIOB
#define RELE_3_Pin GPIO_PIN_4
#define RELE_3_GPIO_Port GPIOB
#define RELE_4_Pin GPIO_PIN_5
#define RELE_4_GPIO_Port GPIOB
#define RELE_5_Pin GPIO_PIN_6
#define RELE_5_GPIO_Port GPIOB
#define EEPROM_EN_Pin GPIO_PIN_7
#define EEPROM_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

extern TIM_HandleTypeDef htim3,htim2;
extern GlobalPrimitiveIOStates PrimitiveStates;
extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart1;
//extern DMA_HandleTypeDef hdma_usart1_rx;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
