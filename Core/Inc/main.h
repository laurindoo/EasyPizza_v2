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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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

/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim3;
extern GlobalPrimitiveIOStates PrimitiveStates;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
