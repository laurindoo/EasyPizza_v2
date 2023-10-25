/*
 * TaskEeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */
#include "TaskEeprom.h"
void initEeprom(void);
void processaEeprom(void);


extern osThreadId TaskEepromHandle,TaskComandoHandle,TaskNextionHandle,TaskBluetoothHandle;
extern osMessageQId FilaEepromHandle;
extern TYPE_CALENDARIO			Calendario;

void StartEeprom(void const * argument)
{
	initEeprom();

	for(;;)
	{
		processaEeprom();

		osDelay(500);
	}
}

void initEeprom(void){
	//inicializacao ad eeprom
	EepromInit(&eeprom, &hi2c1 ,&FilaEepromHandle);
	//	zeraTodosMeses(&eeprom);

	//criacao dos objetos variaveis
	EepromAddVar(&eeprom, &tempoDelayLuz, 			"addrTEMPO_LUZ", 		addrTEMPO_LUZ,		DATA8BITS,	0,		0,		0		,0);
	EepromAddVar(&eeprom, &horimetroHoras, 			"addrHORIMETRO", 		addrHORIMETRO,		DATA8BITS,	0,		0,		0		,0);
	EepromAddVar(&eeprom, &horimetroMinutos, 		"addrMINUTIMETRO", 		addrMINUTIMETRO,	DATA8BITS,	0,		0,		0		,0);
	EepromAddVar(&eeprom, &instalacaoDia, 			"addrINST_DIA", 		addrINST_DIA,		DATA8BITS,	1,		0,		31		,0);
	EepromAddVar(&eeprom, &instalacaoMes, 			"addrINST_MES", 		addrINST_MES,		DATA8BITS,	1,		1,		12		,0);
	EepromAddVar(&eeprom, &instalacaoAno, 			"addrINST_ANO", 		addrINST_ANO,		DATA8BITS,	23,		23,		99		,0);
	EepromAddVar(&eeprom, &totalCiclos, 			"addrTOTAL_GERAL", 		addrTOTAL_GERAL,	DATA16BITS,	0,		0,		0		,0);
	EepromAddVar(&eeprom, &LimiteTemperatura, 		"addrLIMITE_TEMP", 		addrLIMITE_TEMP,	DATA16BITS,	0,		0,		0		,0);

		RestauraPadraoTudo(&eeprom);
	//faz o download dos objetos
	//	EepromDownloadValores(&eeprom);


	//	osThreadResume(TaskComandoHandle);
	//	osThreadResume(TaskBluetoothHandle);
}

void processaEeprom(void){
	osEvent  evt;

	evt = osMessageGet(FilaEepromHandle, osWaitForever);  // wait for message
	if (evt.status == osEventMessage) {

		switch ((unsigned int)evt.value.p) {

		case CEepromShifiting:
			__NOP();
			//usar um semaphore para evitar de shiffiting em duplicidade
			break;
		case CEepromHorimetro:
			//disparar o horimetro ao fim de cada receita e/ou evento de cancelamento
			break;
		case CEepromDataInstalacao:
			break;
		case CEepromHardReset:
			break;
		case CEepromSoftReset:
			break;
		case CEepromAtualizaHora:
//
			//				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK){
			//					Error_Handler();
			//				}
			//				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK){
			//					Error_Handler();
			//				}
			break;
		default:
			break;
		}
	}
}
