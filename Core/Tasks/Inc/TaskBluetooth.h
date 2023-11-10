/*
 * Aplicacao.h
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#ifndef INC_APLICACAO_H_
#define INC_APLICACAO_H_

#include "main.h"//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Bluetooth.h"

#include "TaskEeprom.h"
#define VERSAO 	1// versao salva no dia 31/05/2023 -

extern osMessageQId FilaRXBluetoothHandle,FilaTXBluetoothHandle,FilaEepromHandle;
extern int recorrencia;
extern Bluetooth bluetooth;

//#define MACRO_ANULA_INATIVIDADE PrimitiveStates.intatividadeTime = 0;

#endif /* INC_APLICACAO_H_ */
