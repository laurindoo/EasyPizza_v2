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
#include <errno.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

//SENHA: uK9--Pj8


//---Comandos Eeprom
typedef enum
{   CEepromHorimetro,
	CEepromHardReset,
	CEepromSoftReset,
	CEepromLimiteTemp,
	CEepromLimiteLuz,
	CEepromNewCile,
	CEepromTempMaxTetoAgain,
	CEepromTempMaxLastroAgain,
	CEepromTunning,
	CEepromToogleBuzzer,
	CEepromClearErrors,
} ComandosEeprom;

//---Estados maquina
typedef enum
{   TIMER_idle = 0		,
	TIMER_decrementando	,
	TIMER_pausado		,
} Stimer;

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

// estrutura Calendario, salvamento de eventos
typedef struct TYPE_CALENDARIO{

	uint16_t		Horimetro_horas;		//total de horas da maquina
	uint8_t			Horimetro_parcial_min; 	//a cada minuto eu incremento, comparo com o gravado, e hora++ se for o caso
	uint16_t		TotalCiclos;
	uint16_t		ContMaxTeto;
	uint16_t		ContMaxLastro;
}
TYPE_CALENDARIO;//

//---Estrutura variaveis principais
typedef struct
{
	//saidas digitais
	IndviduoOutput 	Lampada, Cooler; 	//saidas digitais
	IndviduoOutput	LedVerde;			//Led verde sinalizando ok
	IndviduoOutput	LedTeto, LedLastro;	//Leds das resistencias

	//saidas pid
	IndviduoPID		Teto;
	IndviduoPID		Lastro;

	uint8_t		Buzzer;
	uint8_t		SPTimerMinutos;
	uint8_t		SPTimerSegundos;
	uint8_t		RTTimerMinutos;
	uint8_t		RTTimerSegundos;
	Stimer		stateTimer;

	BIT_TO_BYTE_ERROS		Erro;

}GlobalPrimitiveIOStates;





/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

//--- defines de fabrica
#define STD_REF_EEPROM			0xb2		// valor aleatorio para referencia e auto-reset.
#define ON_FAN_TEMPERATURA 		200  		// temperatura de acionamento do cooler.
#define TIME_INATIVO_SETUP 		1800 		// tempo de inatividade limite.
#define TIME_MAX_AQUECIMENTO 	600  		// tempo maximo permitido para nao entrar em erro de aquecimento.
#define TIME_LAMPADA 			45   		// tempo padrao de lampada acionada antes de desligamento auto.
#define NOME_DEVICE 			"EasyPizza" // nome que irá aparece nas pesquisas de bluetooth devices.
#define ENVIO_DE_SINCRONIAS		3			// repeticao de sincronia ao re-conectar.
#define N_REP_SINAL_PRONTO		10			// repeticoes de sinal pronto ao fim receita.
#define N_REP_SINAL_NEGADO		2			// repeticoes de sinal negado.
#define STD_BUZZER				1			// configuracao do buzzer (1-on / 0-off).
#define STD_LIMITE_TEMP			600			// limite maximo aceito para temperatura
#define STD_LIMITE_HISTERESE	20			// limite aceitavel para clase

//--- defines de PID
#define STD_KP				30
#define STD_KI				(0.01)
#define STD_KD				(0.3)
#define STD_HISTERESE		3
#define STD_LIMITE			450
#define STD_LIMITETETO		450
#define STD_LIMITELASTRO	475


//--- sinais de buzzer notify
#define SINAL_TEMP_REACH	0b00000001
#define SINAL_NEGADO		0b00000010
#define SINAL_COMFIRMA		0b00000100
#define SINAL_PRONTO		0b00001000
#define SINAL_CONECTOU		0b00010000

//--- sinal de erro.
#define ERRO_CRITICO 			0x0f
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
