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
#include "bluetooth.h"

//#define VERSAO 	1// versao salva no dia 31/05/2023 -
//#define VERSAO 	2// versao salva no dia 23/11/2023 -
#define VERSAO 	3// versao salva no dia 22/12/2023 -


extern Bluetooth bluetooth;
extern OutputDigital 	outPuts;
extern PID_TypeDef TPIDTeto,TPIDLastro;
extern osThreadId TaskBuzzerHandle;
extern osMessageQId FilaEepromHandle;

//#define MACRO_ANULA_INATIVIDADE PrimitiveStates.intatividadeTime = 0;

#endif /* INC_APLICACAO_H_ */
