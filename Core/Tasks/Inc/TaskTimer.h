/*
 * TaskReceita.h
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#ifndef INC_TASKRECEITA_H_
#define INC_TASKRECEITA_H_

#include "main.h"//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

//---WARMER SUPERIOR
#define MACRO_LIGA_WARMER_SUP		HAL_GPIO_WritePin 	(WARMER_SUP_GPIO_Port,	WARMER_SUP_Pin,	GPIO_PIN_SET);
#define MACRO_DESLIGA_WARMER_SUP	HAL_GPIO_WritePin 	(WARMER_SUP_GPIO_Port,	WARMER_SUP_Pin,	GPIO_PIN_RESET);
#define MLigaWarmerSup		PrimitiveStates.WarmerSup = true; \
		MACRO_LIGA_WARMER_SUP
#define MDesligaWarmerSup	PrimitiveStates.WarmerSup = false; \
		MACRO_DESLIGA_WARMER_SUP

//---WARMER INFERIOR
#define MACRO_LIGA_WARMER_INF		HAL_GPIO_WritePin 	(WARMER_INF_GPIO_Port,	WARMER_INF_Pin,	GPIO_PIN_SET);
#define MACRO_DESLIGA_WARMER_INF	HAL_GPIO_WritePin 	(WARMER_INF_GPIO_Port,	WARMER_INF_Pin,	GPIO_PIN_RESET);
#define MLigaWarmerInf		PrimitiveStates.WarmerInf = true; \
		MACRO_LIGA_WARMER_INF
#define MDesligaWarmerInf	PrimitiveStates.WarmerInf = false; \
		MACRO_DESLIGA_WARMER_INF

//---SOLUCAO 1
#define MACRO_ABRE_SOL1		HAL_GPIO_WritePin 	(VALV_OUT_1_GPIO_Port,	VALV_OUT_1_Pin,	GPIO_PIN_SET);
#define MACRO_FECHA_SOL1 	HAL_GPIO_WritePin 	(VALV_OUT_1_GPIO_Port,	VALV_OUT_1_Pin,	GPIO_PIN_RESET);
#define MAbreVlvDis1		PrimitiveStates.ValvDisp1 = true; \
		MACRO_ABRE_SOL1
#define MFechaVlvDis1		PrimitiveStates.ValvDisp1 = false; \
		MACRO_FECHA_SOL1

//---SOLUCAO 2
#define MACRO_ABRE_SOL2		HAL_GPIO_WritePin 	(VALV_OUT_2_GPIO_Port,	VALV_OUT_2_Pin,	GPIO_PIN_SET);
#define MACRO_FECHA_SOL2 	HAL_GPIO_WritePin 	(VALV_OUT_2_GPIO_Port,	VALV_OUT_2_Pin,	GPIO_PIN_RESET);
#define MAbreVlvDis2		PrimitiveStates.ValvDisp2 = true; \
		MACRO_ABRE_SOL2
#define MFechaVlvDis2		PrimitiveStates.ValvDisp2 = false; \
		MACRO_FECHA_SOL2

#endif /* INC_TASKRECEITA_H_ */
