/*
 * TaskEeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */
#include "TaskEeprom.h"
void initEeprom(void);
void setupEepromVarArr(void);
void processaEeprom(void);
Eeprom eeprom;

// objetos de eeeprom.
eepromVarArr E_tempoDelayLuz;
eepromVarArr E_FlagBuzzer;
eepromVarArr E_buffVar;
eepromVarArr E_horimetroHoras;
eepromVarArr E_horimetroMinutos;
eepromVarArr E_totalCiclos;
eepromVarArr E_ContTetoMax,E_ContLastroMax;
eepromVarArr E_hitereseStateTeto,E_limiteTeto;
eepromVarArr E_kDTeto,E_kITeto,E_kPTeto;
eepromVarArr E_kPLastro,E_kILastro,E_kDLastro;
eepromVarArr E_hitereseStateLastro,E_limiteLastro;


void StartEeprom(void const * argument)
{
	init_containerEeprom(&eeprom, &hi2c1 ,&FilaEepromHandle);

	setupEepromVarArr();

	eeprom.M_downloadAllVar(&eeprom);	//le

	osThreadResume(TaskTemperaturaHandle);

	for(;;)
	{
		processaEeprom();
		osThreadYield();
		osDelay(500);
	}
}

void setupEepromVarArr(void){

	// tempo de LUZ.
	init_objArrEeprom(&E_tempoDelayLuz, SOFT_RESET, addrTEMPO_LUZ, DATA_16BITS, &PrimitiveStates.Lampada.limitOn);
	E_tempoDelayLuz.M_setStdValues16bits(&E_tempoDelayLuz, 0 ,TIME_LAMPADA,6000);
	eeprom.M_AddOnArr(&eeprom,	&E_tempoDelayLuz);

	// HORIMETRO, total apenas de HORAS em funcionamento.
	init_objArrEeprom(&E_horimetroHoras, HARD_RESET, addrHORIMETRO, DATA_16BITS, &Calendario.Horimetro_horas);
	E_horimetroHoras.M_setStdValues16bits(&E_horimetroHoras, 0,0,60000);
	eeprom.M_AddOnArr(&eeprom,	&E_horimetroHoras);

	// HORIMETRO, parte em MINUTOS do total.
	init_objArrEeprom(&E_horimetroMinutos, HARD_RESET, addrMINUTIMETRO, DATA_8BITS, &Calendario.Horimetro_parcial_min);
	E_horimetroMinutos.M_setStdValues8bits(&E_horimetroMinutos, 0,0,60);
	eeprom.M_AddOnArr(&eeprom,	&E_horimetroMinutos);

	// contagem total de ciclos realizados pelo TIMER.
	init_objArrEeprom(&E_totalCiclos, HARD_RESET, addrTOTAL_GERAL, DATA_16BITS, &Calendario.TotalCiclos);
	E_totalCiclos.M_setStdValues16bits(&E_totalCiclos, 0,0,60000);
	eeprom.M_AddOnArr(&eeprom,	&E_totalCiclos);

	// contagem total de vezes que o sensor de TETO chegou em temperatura maxima.
	init_objArrEeprom(&E_ContTetoMax, HARD_RESET, addrCONT_MAX_TETO, DATA_16BITS, &Calendario.ContMaxTeto);
	E_ContTetoMax.M_setStdValues16bits(&E_ContTetoMax, 0,0,60000);
	eeprom.M_AddOnArr(&eeprom,	&E_ContTetoMax);

	// contagem total de vezes que o sensor de LASTRO chegou em temperatura maxima.
	init_objArrEeprom(&E_ContLastroMax, HARD_RESET, addrCONT_MAX_LASTRO, DATA_16BITS, &Calendario.ContMaxLastro);
	E_ContLastroMax.M_setStdValues16bits(&E_ContLastroMax, 0,0,60000);
	eeprom.M_AddOnArr(&eeprom,	&E_ContLastroMax);

	/*----------------- MEMORIA DE CONTROLE TEMPERATURA DE TETO -----------------*/
	// HISTERESE em graus do TETO.
	init_objArrEeprom(&E_hitereseStateTeto, SOFT_RESET, addrTETO_HIST, DATA_16BITS, &PrimitiveStates.Teto.histerese);
	E_hitereseStateTeto.M_setStdValues16bits(&E_hitereseStateTeto, 0 ,STD_HISTERESE,STD_LIMITE_HISTERESE);
	eeprom.M_AddOnArr(&eeprom,	&E_hitereseStateTeto);

	// LIMITE para TETO.
	init_objArrEeprom(&E_limiteTeto, SOFT_RESET, addrTETO_LIMIT, DATA_16BITS, &PrimitiveStates.Teto.limite);
	E_limiteTeto.M_setStdValues16bits(&E_limiteTeto, 0 ,STD_LIMITETETO,STD_LIMITE_TEMP);
	eeprom.M_AddOnArr(&eeprom,	&E_limiteTeto);

	// KP value para TETO.
	init_objArrEeprom(&E_kPTeto, SOFT_RESET, addrTETO_KP, DATA_DOUBLE, &PrimitiveStates.Teto.kp);
	E_kPTeto.M_setStdValuesDouble(&E_kPTeto, 0 ,STD_KP,1000);
	eeprom.M_AddOnArr(&eeprom,	&E_kPTeto);

	// KI value para TETO.
	init_objArrEeprom(&E_kITeto, SOFT_RESET, addrTETO_KI, DATA_DOUBLE, &PrimitiveStates.Teto.ki);
	E_kITeto.M_setStdValuesDouble(&E_kITeto, 0 ,STD_KI,1000);
	eeprom.M_AddOnArr(&eeprom,	&E_kITeto);

	// KD value para TETO.
	init_objArrEeprom(&E_kDTeto, SOFT_RESET, addrTETO_KD, DATA_DOUBLE, &PrimitiveStates.Teto.kd);
	E_kDTeto.M_setStdValuesDouble(&E_kDTeto, 0 ,STD_KD,1000);
	eeprom.M_AddOnArr(&eeprom,	&E_kDTeto);

	/*----------------- MEMORIA DE CONTROLE TEMPERATURA DE LASTRO -----------------*/
	// HISTERESE em graus do LASTRO.
	init_objArrEeprom(&E_hitereseStateLastro, SOFT_RESET, addrLASTRO_HIST, DATA_16BITS, &PrimitiveStates.Lastro.histerese);
	E_hitereseStateLastro.M_setStdValues16bits(&E_hitereseStateLastro, 0 ,STD_HISTERESE,STD_LIMITE_HISTERESE);
	eeprom.M_AddOnArr(&eeprom,	&E_hitereseStateLastro);

	// LIMITE para LASTRO.
	init_objArrEeprom(&E_limiteLastro, SOFT_RESET, addrLASTRO_LIMIT, DATA_16BITS, &PrimitiveStates.Lastro.limite);
	E_limiteLastro.M_setStdValues16bits(&E_limiteLastro, 0 ,STD_LIMITELASTRO,STD_LIMITE_TEMP);
	eeprom.M_AddOnArr(&eeprom,	&E_limiteLastro);

	// KP value para LASTRO.
	init_objArrEeprom(&E_kPLastro, SOFT_RESET, addrLASTRO_KP, DATA_DOUBLE, &PrimitiveStates.Lastro.kp);
	E_kPLastro.M_setStdValuesDouble(&E_kPLastro, 0 ,STD_KP,1000);
	eeprom.M_AddOnArr(&eeprom,	&E_kPLastro);

	// KI value para LASTRO.
	init_objArrEeprom(&E_kILastro, SOFT_RESET, addrLASTRO_KI, DATA_DOUBLE, &PrimitiveStates.Lastro.ki);
	E_kILastro.M_setStdValuesDouble(&E_kILastro, 0 ,STD_KI,1000);
	eeprom.M_AddOnArr(&eeprom,	&E_kILastro);

	// KD value para LASTRO.
	init_objArrEeprom(&E_kDLastro, SOFT_RESET, addrLASTRO_KD, DATA_DOUBLE, &PrimitiveStates.Lastro.kd);
	E_kDLastro.M_setStdValuesDouble(&E_kDLastro, 0 ,STD_KD,1000);
	eeprom.M_AddOnArr(&eeprom,	&E_kDLastro);

	//flag buzzer
	init_objArrEeprom(&E_FlagBuzzer, SOFT_RESET, addrBUZZER, DATA_8BITS, &PrimitiveStates.Buzzer);
	E_FlagBuzzer.M_setStdValues8bits(&E_FlagBuzzer, 0,0,1);
	eeprom.M_AddOnArr(&eeprom,	&E_FlagBuzzer);

}


void processaEeprom(void){
	osEvent  evt;

	evt = osMessageGet(FilaEepromHandle, osWaitForever);  // wait for message
	if (evt.status == osEventMessage) {

		switch ((unsigned int)evt.value.p) {

		case CEepromNewCile:
			E_totalCiclos.M_update_eepromValue(&E_totalCiclos);
			break;
		case CEepromHorimetro:
			E_horimetroHoras.M_update_eepromValue(&E_horimetroHoras);
			E_horimetroMinutos.M_update_eepromValue(&E_horimetroMinutos);
			break;
		case CEepromSoftReset:
			eeprom.M_resetAllVar(&eeprom,SOFT_RESET);
			break;
		case CEepromHardReset:
			eeprom.M_resetAllVar(&eeprom,HARD_RESET);
			break;
		case CEepromLimiteTemp:
			E_limiteTeto.M_update_eepromValue(&E_limiteTeto);
			E_limiteLastro.M_update_eepromValue(&E_limiteLastro);
			break;
		case CEepromLimiteLuz:
			E_tempoDelayLuz.M_update_eepromValue(&E_tempoDelayLuz);
			break;
		case CEepromTempMaxTetoAgain:
			Calendario.ContMaxTeto+=1;
			E_ContTetoMax.M_update_eepromValue(&E_ContTetoMax);
			break;
		case CEepromTempMaxLastroAgain:
			Calendario.ContMaxLastro+=1;
			E_ContLastroMax.M_update_eepromValue(&E_ContLastroMax);
			break;
		case CEepromTunning:
			// atualizacao de valores de teto.
			E_hitereseStateTeto.M_update_eepromValue(&E_hitereseStateTeto);
			E_kPTeto.M_update_eepromValue(&E_kPTeto);
			E_kITeto.M_update_eepromValue(&E_kITeto);
			E_kDTeto.M_update_eepromValue(&E_kDTeto);
			E_limiteTeto.M_update_eepromValue(&E_limiteTeto);

			// atualizacao de valores de lastro.
			E_hitereseStateLastro.M_update_eepromValue(&E_hitereseStateLastro);
			E_kPLastro.M_update_eepromValue(&E_kPLastro);
			E_kILastro.M_update_eepromValue(&E_kILastro);
			E_kDLastro.M_update_eepromValue(&E_kDLastro);
			E_limiteLastro.M_update_eepromValue(&E_limiteLastro);
			break;
		case CEepromToogleBuzzer:
			E_FlagBuzzer.M_update_eepromValue(&E_FlagBuzzer);
			break;
		default:
			break;
		}
	}
}
