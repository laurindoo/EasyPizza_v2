/*
 * TaskEeprom.h
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#ifndef INC_TASKEEPROM_H_
#define INC_TASKEEPROM_H_

#include "main.h"//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "Eeprom.h"

extern osThreadId TaskEepromHandle,TaskComandoHandle,TaskBluetoothHandle;
extern TYPE_CALENDARIO			Calendario;
extern osMessageQId FilaEepromHandle;

//variaveis globais da EEprom
extern  Eeprom eeprom;
extern	EepromVariaveis horimetroHoras,horimetroMinutos;
extern	EepromVariaveis instalacaoDia,instalacaoMes,instalacaoAno;
extern	EepromVariaveis totalCiclos,LimiteTemperatura;
extern  EepromVariaveis tempoDelayLuz;


void atualizaDataEeprom(RTC_DateTypeDef data, RTC_TimeTypeDef hora);


#endif /* INC_TASKEEPROM_H_ */
