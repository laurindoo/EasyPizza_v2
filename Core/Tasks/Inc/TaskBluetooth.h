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
//#define VERSAO 	1// versao salva no dia 31/05/2023 -
#define VERSAO 	2// versao salva no dia 23/11/2023 -

extern osThreadId TaskBuzzerHandle;
extern osMessageQId FilaRXBluetoothHandle,FilaTXBluetoothHandle,FilaEepromHandle, FilaBleComandoHandle;
extern int recorrencia;
extern Bluetooth bluetooth;

//---Comandos Ble
typedef enum
{
	RX_ATUALIZA_HORA 		= 0x03,
	RX_RESTAURA_HARD		= 0x09,
	RX_RESTAURA 			= 0x10,
	RX_SOLICITA_REALTIME 	= 0x15,
	RX_SOLICITA_SINCRONIA 	= 0x17,
	RX_SP_TEMP_TETO			= 0x21,
	RX_SP_TEMP_LASTRO		= 0x22,
	RX_SP_TEMPO				= 0x23,
	RX_TOGGLE_TEMPO			= 0x24,
	RX_RECEITA			 	= 0x25,
	RX_LIMITE_TEMPERATURA 	= 0x26,
	RX_LIGA_LAMPADA		 	= 0x27,
	RX_DESLIGA_LAMPADA	 	= 0x28,
	RX_CANCELA_PROCESSO	 	= 0x29,
	RX_LIMITE_LAMPADA	 	= 0x30,
	RX_TUNNING_TETO 		= 0x33,
	RX_TUNNING_LASTRO	 	= 0x34,
	RX_TOGGLE_BUZZER	 	= 0x35,

} ComandosBleRX;

//---Comandos Ble
typedef enum
{
	TX_REALTIME_DATA 		= 0x16,
	TX_REALTIME_DATA2		= 0x17,
	TX_SINCRONIA 			= 0x18,
	TX_SINCRONIA2 			= 0x19,
	TX_SINCRONIA3 			= 0x20,
	TX_RESETANDO 			= 0x29,
	TX_RESETADO_OK 			= 0x30,

} ComandosBleTX;

//#define MACRO_ANULA_INATIVIDADE PrimitiveStates.intatividadeTime = 0;

#endif /* INC_APLICACAO_H_ */
