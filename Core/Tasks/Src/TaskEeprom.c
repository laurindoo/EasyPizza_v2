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
EepromVariaveis hitereseStateTeto,limiteTeto;
EepromVariaveis hitereseStateLastro,limiteLastro;

EepromVarFloating kPTeto,kITeto,kDTeto,kPLastro,kILastro,kDLastro;
EepromVariaveis FlagMemoria;


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

	//criacao dos objetos variaveis
	EepromAddVar(&eeprom,0,&tempoDelayLuz, 		"addrTEMPO_LUZ", 		addrTEMPO_LUZ,		DATA16BITS,	0,		45,		60000	,&PrimitiveStates.Lampada.limitOn);
	EepromAddVar(&eeprom,1,&horimetroHoras, 	"addrHORIMETRO", 		addrHORIMETRO,		DATA16BITS,	0,		0,		60000	,&Calendario.Horimetro_horas);
	EepromAddVar(&eeprom,1,&horimetroMinutos, 	"addrMINUTIMETRO", 		addrMINUTIMETRO,	DATA8BITS,	0,		0,		254		,&Calendario.Horimetro_parcial_min);
	EepromAddVar(&eeprom,1,&instalacaoDia, 		"addrINST_DIA", 		addrINST_DIA,		DATA8BITS,	1,		0,		31		,&Calendario.Data_instalacao.Date);
	EepromAddVar(&eeprom,1,&instalacaoMes, 		"addrINST_MES", 		addrINST_MES,		DATA8BITS,	1,		1,		12		,&Calendario.Data_instalacao.Month);
	EepromAddVar(&eeprom,1,&instalacaoAno, 		"addrINST_ANO", 		addrINST_ANO,		DATA8BITS,	23,		23,		99		,&Calendario.Data_instalacao.Year);
	EepromAddVar(&eeprom,1,&totalCiclos, 		"addrTOTAL_GERAL", 		addrTOTAL_GERAL,	DATA16BITS,	0,		0,		60000	,&Calendario.TotalCiclos);
	EepromAddVar(&eeprom,1,&ContTetoMax, 		"addrCONT_MAX_TETO", 	addrCONT_MAX_TETO,	DATA16BITS,	0,		0,		60000	,&Calendario.ContMaxTeto);
	EepromAddVar(&eeprom,1,&ContLastroMax, 		"addrCONT_MAX_LASTRO", 	addrCONT_MAX_LASTRO,DATA16BITS,	0,		0,		60000	,&Calendario.ContMaxLastro);

	EepromAddVar(&eeprom,0,&hitereseStateTeto, 	"addrTETO_HIST", 		addrTETO_HIST,		DATA16BITS,	0,		STD_HISTERESE,10	,&PrimitiveStates.Teto.histerese);
	EepromAddVar(&eeprom,0,&limiteTeto, 		"addrTETO_LIMIT", 		addrTETO_LIMIT,		DATA16BITS,	0,		475			,500	,&PrimitiveStates.Teto.limite);
	EepromAddVar(&eeprom,0,&hitereseStateLastro,"addrLASTRO_HIST", 		addrLASTRO_HIST,	DATA16BITS,	0,		STD_HISTERESE,10	,&PrimitiveStates.Lastro.histerese);
	EepromAddVar(&eeprom,0,&limiteLastro,		"addrLASTRO_LIMIT", 	addrLASTRO_LIMIT,	DATA16BITS,	0,		STD_LIMITE	,500	,&PrimitiveStates.Lastro.limite);

	EepromAddVarFloating(&eeprom, 0, &kPTeto, 	"addrTETO_KP", 			addrTETO_KP,		DATADOUBLE,	0,		STD_KP		,1000	,&PrimitiveStates.Teto.kp);
	EepromAddVarFloating(&eeprom, 0, &kITeto, 	"addrTETO_KI", 			addrTETO_KI,		DATADOUBLE,	0,		STD_KI		,1000	,&PrimitiveStates.Teto.ki);
	EepromAddVarFloating(&eeprom, 0, &kDTeto, 	"addrTETO_KD", 			addrTETO_KD,		DATADOUBLE,	0,		STD_KD		,1000	,&PrimitiveStates.Teto.kd);
	EepromAddVarFloating(&eeprom, 0, &kPLastro,	"addrLASTRO_KP", 		addrLASTRO_KP,		DATADOUBLE,	0,		STD_KP		,1000	,&PrimitiveStates.Lastro.kp);
	EepromAddVarFloating(&eeprom, 0, &kILastro,	"addrLASTRO_KI", 		addrLASTRO_KI,		DATADOUBLE,	0,		STD_KI		,1000	,&PrimitiveStates.Lastro.ki);
	EepromAddVarFloating(&eeprom, 0, &kDLastro,	"addrLASTRO_KD", 		addrLASTRO_KD,		DATADOUBLE,	0,		STD_KD		,1000	,&PrimitiveStates.Lastro.kd);

	//todo primeiro testar se realmente ele salva o valro com virgula - feito
	//todo altera funcao setavar e retirar possibilidade de alterar o valor por ela
	//todo criar uma funcao que restaure tudo

	//	RestauraSoft(&eeprom);

	__NOP();

	//faz o download dos objetos
	EepromDownloadValores(&eeprom);

	__NOP();

	//	RestauraSoft(&eeprom);

	__NOP();

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
		case CEepromSoftReset:
			RestauraEeprom(&eeprom,softReset);	// restaura
			EepromDownloadValores(&eeprom);		//le

			osMessagePut(FilaTXBluetoothHandle, TX_RESETADO_OK, 0);
			break;
		case CEepromHardReset:
			RestauraEeprom(&eeprom,hardReset); 	// restaura
			EepromDownloadValores(&eeprom);		// le

			osMessagePut(FilaTXBluetoothHandle, TX_RESETADO_OK, 0);

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
			//atualizacao de valores de teto9
			EepromSetVar(&eeprom, &hitereseStateTeto, 	0);
			EepromSetVarFloating(&eeprom, &kPTeto, 	0);
			EepromSetVarFloating(&eeprom, &kITeto, 	0);
			EepromSetVarFloating(&eeprom, &kDTeto, 	0);
			EepromSetVar(&eeprom, &limiteTeto, 	0);
			PID_SetTunings(&TPIDTeto, PrimitiveStates.Teto.kp, PrimitiveStates.Teto.ki, PrimitiveStates.Teto.kd);

			//atualizacao de valores de lastro
			EepromSetVar(&eeprom, &hitereseStateLastro, 0);
			EepromSetVarFloating(&eeprom, &kPLastro, 	0);
			EepromSetVarFloating(&eeprom, &kILastro, 	0);
			EepromSetVarFloating(&eeprom, &kDLastro, 	0);
			EepromSetVar(&eeprom, &limiteLastro, 	0);
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


