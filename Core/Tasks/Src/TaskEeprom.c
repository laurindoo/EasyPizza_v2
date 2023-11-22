/*
 * TaskEeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */
#include "TaskEeprom.h"
void initEeprom(void);
void processaEeprom(void);
Eeprom eeprom;
EepromVariaveis horimetroHoras,horimetroMinutos;
EepromVariaveis instalacaoDia,instalacaoMes,instalacaoAno;
EepromVariaveis totalCiclos,LimiteTemperatura;
EepromVariaveis tempoDelayLuz,ContTetoMax,ContLastroMax;


RTC_DateTypeDef datetoUpdate;
RTC_TimeTypeDef timeToUpdate;

void StartEeprom(void const * argument)
{
	initEeprom();

	for(;;)
	{
		processaEeprom();
		osThreadYield();
		osDelay(500);
	}
}

void initEeprom(void){
	//inicializacao ad eeprom
	EepromInit(&eeprom, &hi2c1 ,&FilaEepromHandle);
	//	zeraTodosMeses(&eeprom);

	//criacao dos objetos variaveis
	EepromAddVar(&eeprom, &tempoDelayLuz, 			"addrTEMPO_LUZ", 		addrTEMPO_LUZ,		DATA8BITS,	5,		35,		250		,(uint32_t *)&PrimitiveStates.SPLampada);
	EepromAddVar(&eeprom, &horimetroHoras, 			"addrHORIMETRO", 		addrHORIMETRO,		DATA16BITS,	0,		0,		0		,(uint32_t *)&Calendario.Horimetro_horas);
	EepromAddVar(&eeprom, &horimetroMinutos, 		"addrMINUTIMETRO", 		addrMINUTIMETRO,	DATA8BITS,	0,		0,		0		,(uint32_t *)&Calendario.Horimetro_parcial_min);
	EepromAddVar(&eeprom, &instalacaoDia, 			"addrINST_DIA", 		addrINST_DIA,		DATA8BITS,	1,		0,		31		,(uint32_t *)&Calendario.Data_instalacao.Date);
	EepromAddVar(&eeprom, &instalacaoMes, 			"addrINST_MES", 		addrINST_MES,		DATA8BITS,	1,		1,		12		,(uint32_t *)&Calendario.Data_instalacao.Month);
	EepromAddVar(&eeprom, &instalacaoAno, 			"addrINST_ANO", 		addrINST_ANO,		DATA8BITS,	23,		23,		99		,(uint32_t *)&Calendario.Data_instalacao.Year);
	EepromAddVar(&eeprom, &totalCiclos, 			"addrTOTAL_GERAL", 		addrTOTAL_GERAL,	DATA16BITS,	0,		0,		0		,(uint32_t *)&Calendario.TotalCiclos);
	EepromAddVar(&eeprom, &LimiteTemperatura, 		"addrLIMITE_TEMP", 		addrLIMITE_TEMP,	DATA16BITS,	0,		0,		0		,(uint32_t *)&PrimitiveStates.LimiteTemp);


	EepromAddVar(&eeprom, &ContTetoMax, 			"addrCONT_MAX_TETO", 	addrCONT_MAX_TETO,	DATA16BITS,	0,		400,	700		,(uint32_t *)&Calendario.ContMaxTeto);
	EepromAddVar(&eeprom, &ContLastroMax, 			"addrCONT_MAX_LASTRO", 	addrCONT_MAX_LASTRO,DATA16BITS,	0,		400,	700		,(uint32_t *)&Calendario.ContMaxLastro);

//	RestauraPadraoTudo(&eeprom);
	//faz o download dos objetos
		EepromDownloadValores(&eeprom);
}

void processaEeprom(void){
	osEvent  evt;

	evt = osMessageGet(FilaEepromHandle, osWaitForever);  // wait for message
	if (evt.status == osEventMessage) {

		switch ((unsigned int)evt.value.p) {

		case CEepromNewCile:
			Calendario.TotalCiclos+=1;
			EepromSetVar(&eeprom, &totalCiclos, 	0);
			break;
		case CEepromHorimetro:
			if(Calendario.Horimetro_parcial_min<59){
				Calendario.Horimetro_parcial_min++;
			}else{
				Calendario.Horimetro_parcial_min=0;
				Calendario.Horimetro_horas++;
			}
			EepromSetVar(&eeprom, &horimetroHoras, 		0);
			EepromSetVar(&eeprom, &horimetroMinutos, 	0);
			break;
		case CEepromDataInstalacao:
			EepromSetVar(&eeprom, &instalacaoDia, 	datetoUpdate.Date);
			EepromSetVar(&eeprom, &instalacaoMes, 	datetoUpdate.Month);
			EepromSetVar(&eeprom, &instalacaoAno, 	datetoUpdate.Year);
			if (HAL_RTC_SetTime(&hrtc, &timeToUpdate, RTC_FORMAT_BIN) != HAL_OK){
				Error_Handler();
			}
			if (HAL_RTC_SetDate(&hrtc, &datetoUpdate, RTC_FORMAT_BIN) != HAL_OK){
				Error_Handler();
			}
			break;
		case CEepromHardReset:
			break;
		case CEepromSoftReset:
			break;
		case CEepromAtualizaHora:

			if (HAL_RTC_SetTime(&hrtc, &timeToUpdate, RTC_FORMAT_BIN) != HAL_OK){
				Error_Handler();
			}
			if (HAL_RTC_SetDate(&hrtc, &datetoUpdate, RTC_FORMAT_BIN) != HAL_OK){
				Error_Handler();
			}
			break;
		case CEepromLimiteTemp:
			EepromSetVar(&eeprom, &LimiteTemperatura, 	0);
			break;
		case CEepromLimiteLuz:
			EepromSetVar(&eeprom, &tempoDelayLuz, 	0);
			break;
		case CEepromTempMaxTetoAgain:
			Calendario.ContMaxTeto+=1;
			EepromSetVar(&eeprom, &ContTetoMax, 	0);
			break;
		case CEepromTempMaxLastroAgain:
			Calendario.ContMaxLastro+=1;
			EepromSetVar(&eeprom, &ContLastroMax, 	0);
			break;
		default:
			break;
		}
	}
}

void atualizaDataEeprom(RTC_DateTypeDef data, RTC_TimeTypeDef hora){
	datetoUpdate = data;
	timeToUpdate = hora;

	if(		(instalacaoDia.valor == 0 || instalacaoDia.valor > 31) &&
			(instalacaoMes.valor == 0 || instalacaoMes.valor > 12 ) &&
			(instalacaoAno.valor == 0 || instalacaoAno.valor > 200)){
		//primeiro recebimento de data
		osMessagePut(FilaEepromHandle, CEepromDataInstalacao, 0);
	}else{
		//gravacao padrao de data e hora
		osMessagePut(FilaEepromHandle, CEepromAtualizaHora, 0);
	}
}


