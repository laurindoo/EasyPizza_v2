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
EepromVariaveis hitereseStateTeto,kPTeto,kITeto,kDTeto,limiteTeto;
EepromVariaveis hitereseStateLastro,kPLastro,kILastro,kDLastro,limiteLastro;


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
//	taskENTER_CRITICAL();

	//inicializacao ad eeprom
	EepromInit(&eeprom, &hi2c1 ,&FilaEepromHandle);

	//criacao dos objetos variaveis
	EepromAddVar(&eeprom, &tempoDelayLuz, 			"addrTEMPO_LUZ", 		addrTEMPO_LUZ,		DATA8BITS,	5,		45,		250		,(uint32_t *)&PrimitiveStates.Lampada.limitOn);
	EepromAddVar(&eeprom, &horimetroHoras, 			"addrHORIMETRO", 		addrHORIMETRO,		DATA16BITS,	0,		0,		0		,(uint32_t *)&Calendario.Horimetro_horas);
	EepromAddVar(&eeprom, &horimetroMinutos, 		"addrMINUTIMETRO", 		addrMINUTIMETRO,	DATA8BITS,	0,		0,		0		,(uint32_t *)&Calendario.Horimetro_parcial_min);
	EepromAddVar(&eeprom, &instalacaoDia, 			"addrINST_DIA", 		addrINST_DIA,		DATA8BITS,	1,		0,		31		,(uint32_t *)&Calendario.Data_instalacao.Date);
	EepromAddVar(&eeprom, &instalacaoMes, 			"addrINST_MES", 		addrINST_MES,		DATA8BITS,	1,		1,		12		,(uint32_t *)&Calendario.Data_instalacao.Month);
	EepromAddVar(&eeprom, &instalacaoAno, 			"addrINST_ANO", 		addrINST_ANO,		DATA8BITS,	23,		23,		99		,(uint32_t *)&Calendario.Data_instalacao.Year);
	EepromAddVar(&eeprom, &totalCiclos, 			"addrTOTAL_GERAL", 		addrTOTAL_GERAL,	DATA16BITS,	0,		0,		0		,(uint32_t *)&Calendario.TotalCiclos);
	EepromAddVar(&eeprom, &ContTetoMax, 			"addrCONT_MAX_TETO", 	addrCONT_MAX_TETO,	DATA16BITS,	0,		0,		700		,(uint32_t *)&Calendario.ContMaxTeto);
	EepromAddVar(&eeprom, &ContLastroMax, 			"addrCONT_MAX_LASTRO", 	addrCONT_MAX_LASTRO,DATA16BITS,	0,		0,		700		,(uint32_t *)&Calendario.ContMaxLastro);

	EepromAddVar(&eeprom, &hitereseStateTeto, 		"addrTETO_HIST", 		addrTETO_HIST,		DATA16BITS,	0,		STD_HISTERESE,0	,(uint32_t *)&PrimitiveStates.Teto.histerese);
	EepromAddVar(&eeprom, &kPTeto, 					"addrTETO_KP", 			addrTETO_KP,		DATADOUBLE,	0,		STD_KP		,0	,(uint32_t *)&PrimitiveStates.Teto.kp);
	EepromAddVar(&eeprom, &kITeto, 					"addrTETO_KI", 			addrTETO_KI,		DATADOUBLE,	0,		STD_KI		,0	,(uint32_t *)&PrimitiveStates.Teto.ki);
	EepromAddVar(&eeprom, &kDTeto, 					"addrTETO_KD", 			addrTETO_KD,		DATADOUBLE,	0,		STD_KD		,0	,(uint32_t *)&PrimitiveStates.Teto.kd);
	EepromAddVar(&eeprom, &limiteTeto, 				"addrTETO_LIMIT", 		addrTETO_LIMIT,		DATADOUBLE,	0,		STD_LIMITE	,0	,(uint32_t *)&PrimitiveStates.Teto.limite);

	EepromAddVar(&eeprom, &hitereseStateLastro,		"addrLASTRO_HIST", 		addrLASTRO_HIST,	DATA16BITS,	0,		STD_HISTERESE,0	,(uint32_t *)&PrimitiveStates.Lastro.histerese);
	EepromAddVar(&eeprom, &kPLastro,				"addrLASTRO_KP", 		addrLASTRO_KP,		DATADOUBLE,	0,		STD_KP		,0	,(uint32_t *)&PrimitiveStates.Lastro.kp);
	EepromAddVar(&eeprom, &kILastro,				"addrLASTRO_KI", 		addrLASTRO_KI,		DATADOUBLE,	0,		STD_KI		,0	,(uint32_t *)&PrimitiveStates.Lastro.ki);
	EepromAddVar(&eeprom, &kDLastro,				"addrLASTRO_KD", 		addrLASTRO_KD,		DATADOUBLE,	0,		STD_KD		,0	,(uint32_t *)&PrimitiveStates.Lastro.kd);
	EepromAddVar(&eeprom, &limiteLastro,			"addrLASTRO_LIMIT", 	addrLASTRO_LIMIT,	DATADOUBLE,	0,		STD_LIMITE	,0	,(uint32_t *)&PrimitiveStates.Lastro.limite);

//	RestauraPadraoTudo(&eeprom);
	//faz o download dos objetos
	EepromDownloadValores(&eeprom);

//	taskEXIT_CRITICAL();
	osThreadResume(TaskTemperaturaHandle);
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
			taskENTER_CRITICAL();
			RestauraPadraoTudo(&eeprom);
			taskEXIT_CRITICAL();
			osMessagePut(FilaTXBluetoothHandle, TX_RESETADO_OK, 0);

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
			EepromSetVar(&eeprom, &limiteTeto, 	0);
			EepromSetVar(&eeprom, &limiteLastro, 	0);
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
		case CEepromTunning:
			//atualizacao de valores de teto
			EepromSetVar(&eeprom, &hitereseStateTeto, 	0);
			EepromSetVar(&eeprom, &kPTeto, 	0);
			EepromSetVar(&eeprom, &kITeto, 	0);
			EepromSetVar(&eeprom, &kDTeto, 	0);
			EepromSetVar(&eeprom, &limiteTeto, 	0);
			PID_SetTunings(&TPIDTeto, PrimitiveStates.Teto.kp, PrimitiveStates.Teto.ki, PrimitiveStates.Teto.kd);

			//atualizacao de valores de lastro
			EepromSetVar(&eeprom, &hitereseStateTeto, 	0);
			EepromSetVar(&eeprom, &kPTeto, 	0);
			EepromSetVar(&eeprom, &kITeto, 	0);
			EepromSetVar(&eeprom, &kDTeto, 	0);
			EepromSetVar(&eeprom, &limiteTeto, 	0);
			PID_SetTunings(&TPIDLastro, PrimitiveStates.Lastro.kp, PrimitiveStates.Lastro.ki, PrimitiveStates.Lastro.kd);

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


