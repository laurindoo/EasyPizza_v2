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
#include "OutputDigital.h"
#include "pid.h"
#include "stdbool.h"
#include "Crc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

//SENHA: uK9--Pj8

//---ESTRUTURA AGRUPAMENTO DE 8 BITS EM 1 BYTE-------------------------------------
typedef union BIT_TO_BYTE_ERROS
{
	struct
	{
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

	RTC_DateTypeDef 		Data_instalacao;
	uint16_t				Horimetro_horas;		//total de horas da maquina
	uint8_t					Horimetro_parcial_min; 	//a cada minuto eu incremento, comparo com o gravado, e hora++ se for o caso
	uint16_t				TotalCiclos;
	uint16_t				ContMaxTeto;
	uint16_t				ContMaxLastro;
}
TYPE_CALENDARIO;//

//---Comandos Eeprom
typedef enum
{   CEepromHorimetro,
	CEepromDataInstalacao,
	CEepromHardReset,
	CEepromSoftReset,
	CEepromAtualizaHora,
	CEepromLimiteTemp,
	CEepromLimiteLuz,
	CEepromNewCile,
	CEepromTempMaxTetoAgain,
	CEepromTempMaxLastroAgain,
	CEepromTunning,
	CEepromToogleBuzzer,
} ComandosEeprom;

//---Estados maquina
typedef enum
{   TIMER_idle = 0		,
	TIMER_decrementando	,
	TIMER_pausado		,
} Stimer;


//---Estados aquecimento
typedef enum
{   PID_idle = 0		,
	PID_buscandoTemp	,
	PID_mantendoTemp	,
} Sheat;

//---Estrutura variaveis principais
typedef struct
{

	OutputDigital 				outPuts;

	//saidas digitais
	IndviduoOutput 	Lampada, Cooler; 	//saidas digitais
	IndviduoOutput	LedVerde;			//Led verde sinalizando ok
	IndviduoOutput	LedTeto, LedLastro;	//Leds das resistencias
	//saidas pid
	IndviduoPID		Teto;
	IndviduoPID		Lastro;

	bool 		ConectionBle;
	uint8_t		Buzzer;

	uint8_t		SPTimerMinutos;
	uint8_t		SPTimerSegundos;
	uint8_t		RTTimerMinutos;
	uint8_t		RTTimerSegundos;

	Stimer		stateTimer;

	BIT_TO_BYTE_ERROS		Erro;

}GlobalPrimitiveIOStates;


typedef union {
    struct {
        uint8_t part1;
        uint8_t part2;
        uint8_t part3;
        uint8_t part4;
    } parts;
    float floatValue;
} FloatContainer;

//typedef union {
//    float value; // Valor de ponto flutuante
//    uint8_t bytes[4]; // Representação em bytes do ponto flutuante
//} FloatAsBytes;
//
//typedef union {
//    uint16_t value; // Valor de ponto flutuante
//    uint8_t bytes[2]; // Representação em bytes do ponto flutuante
//} shortAsBytes;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


//----SINAIS DE BUZZER -------
#define SINAL_TEMP_REACH	0b00000001
#define SINAL_NEGADO		0b00000010
#define SINAL_COMFIRMA		0b00000100
#define SINAL_PRONTO		0b00001000
//#define SINAL_TEMP_REACH	0x01
//#define SINAL_NEGADO		0x20
//#define SINAL_COMFIRMA		0x30
//#define SINAL_PRONTO		0x40

//-----------------DEFINES DE FABRICA------------------------------
#define ERRO_CRITICO 0x0f

#define ON_FAN_TEMPERATURA 		200 	// graus celcius
#define TIME_INATIVO_SETUP 		1800 // tempo de inatividade limite
#define TIME_MAX_AQUECIMENTO 	600  // tempo maximo permitido para nao entrar em erro de aquecimento

//tunning PID
#define STD_KP		30
#define STD_KI		(0.01)
#define STD_KD		(0.3)
#define STD_HISTERESE	3
#define STD_LIMITE		450

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

extern void desligaForno(void);
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim3,htim2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern GlobalPrimitiveIOStates PrimitiveStates;

extern BIT_TO_BYTE_ERROS		Erro;
extern double TempTeto, TempLastro, PIDOutTeto, PIDOutLastro;
extern TYPE_CALENDARIO Calendario;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
