/*
 * Aplicacao.c
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#include "TaskBluetooth.h"
void initBluetooth(void);
void txBluetooth(void);
void rxBluetooth(void);
void taskBluetooth1sec(void);

BleComando BLEAtualizaRealtime;
BleComando BLESolicitaSincronia;
BleComando BLEAtualizaDataHora,BLEAlteraLimiteTemp,BLERestaura,BLESPTeto,BLESPLastro,BLESPtempo,BLEToggleTempo,BLEReceita,BLESPTempo,BLELightOn,BLELightOff;
BleComando BLESetaLampada,BLECancelaProcesso;

void verificaLimiteSetpoint(IndviduoPID	*canal);

void StartBluetooth(void const * argument)
{
	initBluetooth();

	for(;;)
	{
		rxBluetooth();

		txBluetooth();

		osThreadYield();
		osDelay(50);
	}
}

static uint16_t tempoSemAtividade;
#define MACRO_ANULA_INATIVIDADE tempoSemAtividade = 0;

void taskBluetooth1sec(void){
	if(tempoSemAtividade>=TIME_INATIVO_SETUP){
		desligaForno();
	}else
		tempoSemAtividade++;
}

void initBluetooth(void){
	//inicializacao do bluetooth
	BluetoothInit(&bluetooth, &huart1, &hdma_usart1_rx, &FilaRXBluetoothHandle,FilaTXBluetoothHandle);

	//inicializacao do hardware
	iniciaBleHm10(&bluetooth);

	//possiveis comandos a serem recebidos pelo bluetooth
	BluetoothAddComp(&bluetooth, &BLEAtualizaRealtime, 	"RX_SOLICITA_REALTIME", 	RX_SOLICITA_REALTIME, 		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESolicitaSincronia,	"RX_SOLICITA_SINCRONIA", 	RX_SOLICITA_SINCRONIA, 		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLEAlteraLimiteTemp, 	"RX_ALTERA_VALOR_LIMITE", 	RX_LIMITE_TEMPERATURA,		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLEAtualizaDataHora, 	"RX_ATUALIZA_HORA", 		RX_ATUALIZA_HORA,			ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLERestaura, 			"RX_RESTAURA", 				RX_RESTAURA,				ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESPTeto,     		"RX_SP_TEMP_TETO",        	RX_SP_TEMP_TETO,          	ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESPLastro,     		"RX_SP_TEMP_LASTRO",       	RX_SP_TEMP_LASTRO,          ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESPTempo,     		"RX_SP_TEMPO",       		RX_SP_TEMPO,        		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLEToggleTempo,     	"RX_TOGGLE_TEMPO",       	RX_TOGGLE_TEMPO,        	ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLEReceita,     		"RX_RECEITA",     		  	RX_RECEITA,        			ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLELightOn,     		"RX_LIGA_LAMADA",     	  	RX_LIGA_LAMPADA,     		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLELightOff,     		"RX_DESLIGA_LAMPADA",    	RX_DESLIGA_LAMPADA,  		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESetaLampada,     	"RX_LIMITE_LAMPADA",    	RX_LIMITE_LAMPADA,  		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLECancelaProcesso,  	"RX_CANCELA_PROCESSO",    	RX_CANCELA_PROCESSO,  		ComandoBasico);
}

void txBluetooth(void){
	unsigned char	Buffer		[BLUETOOTH_MAX_BUFF_LEN];
	osEvent  evttx;
	evttx = osMessageGet(FilaTXBluetoothHandle, 100);
	if (evttx.status == osEventMessage) {
		switch ((unsigned int)evttx.value.p) {
		case TX_REALTIME_DATA:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x16;									// FUNÇÃO -
			Buffer[2] 	= PrimitiveStates.Erro.byte;			// Conjunto de erros
			Buffer[3] 	= PrimitiveStates.Teto._PWMstate;
			Buffer[4] 	= PrimitiveStates.Lastro._PWMstate;
			Buffer[5] 	= (uint16_t)PrimitiveStates.Teto.realtime 		>>8;
			Buffer[6] 	= (uint16_t)PrimitiveStates.Teto.realtime 		& 0x00FF;
			Buffer[7] 	= (uint16_t)PrimitiveStates.Teto.setPoint 		>>8;
			Buffer[8] 	= (uint16_t)PrimitiveStates.Teto.setPoint 		& 0x00FF;
			Buffer[9] 	= (uint16_t)PrimitiveStates.Lastro.realtime		>>8;
			Buffer[10] 	= (uint16_t)PrimitiveStates.Lastro.realtime 	& 0x00FF;
			Buffer[11] 	= (uint16_t)PrimitiveStates.Lastro.setPoint 	>>8;
			Buffer[12] 	= (uint16_t)PrimitiveStates.Lastro.setPoint 	& 0x00FF;

			BluetoothEnviaComando(Buffer, 12);
			osDelay(10);
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA2, 0);
			break;
		case TX_REALTIME_DATA2:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x17;									// FUNÇÃO -
			Buffer[2] 	= PrimitiveStates.stateTimer;			// State da maquina de timer
			Buffer[3] 	= PrimitiveStates.RTTimerMinutos;
			Buffer[4] 	= PrimitiveStates.RTTimerSegundos;
			Buffer[5] 	= PrimitiveStates.SPTimerMinutos;
			Buffer[6] 	= PrimitiveStates.SPTimerSegundos;
			Buffer[7] 	= (uint16_t)horimetroHoras.valor >> 8;
			Buffer[8] 	= (uint16_t)horimetroHoras.valor & 0x00FF;
			Buffer[9] 	= (uint8_t)horimetroMinutos.valor;
			Buffer[10]	= PrimitiveStates.Lampada._state;
			Buffer[11] 	= (uint16_t)Calendario.TotalCiclos >> 8;
			Buffer[12] 	= (uint16_t)Calendario.TotalCiclos & 0x00FF;
			BluetoothEnviaComando(Buffer, 12);

			osDelay(10);
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA, 0);

			break;
		case TX_SINCRONIA:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x18;									// FUNÇÃO -
			Buffer[2] 	= 0x01;									// Modelo
			Buffer[3] 	= (uint8_t)PrimitiveStates.Lampada.limitOn;
			Buffer[4] 	= (uint8_t)instalacaoDia.valor;
			Buffer[5] 	= (uint8_t)instalacaoMes.valor;
			Buffer[6] 	= (uint8_t)instalacaoAno.valor;
			Buffer[7]	= VERSAO;
			Buffer[8] 	= (uint16_t)Calendario.ContMaxTeto >> 8;
			Buffer[9] 	= (uint16_t)Calendario.ContMaxTeto & 0x00FF;
			Buffer[10] 	= (uint16_t)Calendario.ContMaxLastro >> 8;
			Buffer[11] 	= (uint16_t)Calendario.ContMaxLastro & 0x00FF;
			BluetoothEnviaComando(Buffer, 11);

			osDelay(10);
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA2, 0);

			break;
		case TX_SINCRONIA2:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x19;									// FUNÇÃO -
			Buffer[2] 	= (uint16_t)PrimitiveStates.Teto.kp	>> 8 ;
			Buffer[3] 	= (uint16_t)PrimitiveStates.Teto.kp	& 0x00ff ;
			Buffer[4] 	= (uint16_t)PrimitiveStates.Teto.ki	>> 8 ;
			Buffer[5] 	= (uint16_t)PrimitiveStates.Teto.ki	& 0x00ff ;
			Buffer[6] 	= (uint16_t)PrimitiveStates.Teto.kd	>> 8 ;
			Buffer[7]	= (uint16_t)PrimitiveStates.Teto.kd	& 0x00ff ;
			Buffer[8] 	= (uint16_t)PrimitiveStates.Teto.histerese >> 8;
			Buffer[9] 	= (uint16_t)PrimitiveStates.Teto.histerese & 0x00FF;
			Buffer[10] 	= (uint16_t)PrimitiveStates.Teto.limite >> 8;
			Buffer[11] 	= (uint16_t)PrimitiveStates.Teto.limite & 0x00FF;
			BluetoothEnviaComando(Buffer, 11);

			osDelay(10);
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA3, 0);

			break;
		case TX_SINCRONIA3:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x20;									// FUNÇÃO -
			Buffer[2] 	= (uint16_t)PrimitiveStates.Lastro.kp	>> 8 ;
			Buffer[3] 	= (uint16_t)PrimitiveStates.Lastro.kp	& 0x00ff ;
			Buffer[4] 	= (uint16_t)PrimitiveStates.Lastro.ki	>> 8 ;
			Buffer[5] 	= (uint16_t)PrimitiveStates.Lastro.ki	& 0x00ff ;
			Buffer[6] 	= (uint16_t)PrimitiveStates.Lastro.kd	>> 8 ;
			Buffer[7]	= (uint16_t)PrimitiveStates.Lastro.kd	& 0x00ff ;
			Buffer[8] 	= (uint16_t)PrimitiveStates.Lastro.histerese >> 8;
			Buffer[9] 	= (uint16_t)PrimitiveStates.Lastro.histerese & 0x00FF;
			Buffer[10] 	= (uint16_t)PrimitiveStates.Lastro.limite >> 8;
			Buffer[11] 	= (uint16_t)PrimitiveStates.Lastro.limite & 0x00FF;
			BluetoothEnviaComando(Buffer, 11);

			break;
		case TX_RESETANDO:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x19;									// FUNÇÃO -
			Buffer[2] 	= 0x19;									// FUNÇÃO -
			BluetoothEnviaComando(Buffer, 2);

			break;
		case TX_RESETADO_OK:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x20;									// FUNÇÃO -
			Buffer[2] 	= 0x20;									// FUNÇÃO -
			BluetoothEnviaComando(Buffer, 2);
			break;
		}
	}
}

void rxBluetooth(void){
	osEvent  evtrx;
	evtrx = osMessageGet(FilaRXBluetoothHandle, 100);
	if (evtrx.status == osEventMessage) {
		switch (bluetooth._RxDataArr[1]) {
		case RX_SOLICITA_REALTIME:
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA, 0);
			break;
		case RX_SOLICITA_SINCRONIA:
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA, 0);
			break;
		case RX_ATUALIZA_HORA:

			RTC_DateTypeDef datetoUpdate;
			RTC_TimeTypeDef timeToUpdate;

			datetoUpdate.WeekDay 	= bluetooth._RxDataArr[2]; //Dia da semana p/atualizar
			datetoUpdate.Date 		= bluetooth._RxDataArr[3]; //Dia do mes p/atualizar
			datetoUpdate.Month 		= bluetooth._RxDataArr[4]; //mes p/atualizar
			datetoUpdate.Year 		= bluetooth._RxDataArr[5]; //ano p/atualizar
			timeToUpdate.Hours 		= bluetooth._RxDataArr[6]; //hora p/atualizar
			timeToUpdate.Minutes 	= bluetooth._RxDataArr[7]; //minuto p/atualizar
			timeToUpdate.Seconds 	= bluetooth._RxDataArr[8]; //segundos p/atualizar

			atualizaDataEeprom(datetoUpdate, timeToUpdate);

			MACRO_ENVIA_AKNOLADGE_(RX_ATUALIZA_HORA)
			break;
		case RX_RESTAURA:
			//---------ENDEREÇO | 0x10 | 0x10 | CRC | CRC
			osMessagePut(FilaEepromHandle, CEepromHardReset, 0);
			osMessagePut(FilaTXBluetoothHandle, TX_RESETANDO, 0);
			break;
		case RX_SP_TEMP_TETO:
			//---------ENDEREÇO | 0x21 | SP_Teto.high | SP_Teto.low | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			PrimitiveStates.Teto.setPoint = (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];

			verificaLimiteSetpoint(&PrimitiveStates.Teto);

			MACRO_ENVIA_AKNOLADGE_(RX_SP_TEMP_TETO)
			break;
		case RX_SP_TEMP_LASTRO:
			//---------ENDEREÇO | 0x22 | SP_Lastro.high | SP_Lastro.low | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			PrimitiveStates.Lastro.setPoint = (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];
			verificaLimiteSetpoint(&PrimitiveStates.Lastro);

			MACRO_ENVIA_AKNOLADGE_(RX_SP_TEMP_LASTRO)
			break;
		case RX_SP_TEMPO:
		{	//---------ENDEREÇO | 0x23 | TimerMinutos | TimerSegundos |CRC | CRC
			MACRO_ANULA_INATIVIDADE

			PrimitiveStates.SPTimerMinutos 	= bluetooth._RxDataArr[2];
			PrimitiveStates.SPTimerSegundos = bluetooth._RxDataArr[3];

			PrimitiveStates.RTTimerMinutos = PrimitiveStates.SPTimerMinutos;
			PrimitiveStates.RTTimerSegundos = PrimitiveStates.SPTimerSegundos;

			if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
				PrimitiveStates.stateTimer 	= TIMER_decrementando;
			}

			MACRO_ENVIA_AKNOLADGE_(RX_SP_TEMPO)
		}
		break;
		case RX_TOGGLE_TEMPO:
		{
			//---------ENDEREÇO | 0x24 | 0x24 | TimerSegundos |CRC | CRC
			MACRO_ANULA_INATIVIDADE

			switch (PrimitiveStates.stateTimer) {
			case TIMER_idle:

				PrimitiveStates.RTTimerMinutos 	= PrimitiveStates.SPTimerMinutos;
				PrimitiveStates.RTTimerSegundos = PrimitiveStates.SPTimerSegundos;

				if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
					PrimitiveStates.stateTimer 	= TIMER_decrementando;
				}else{
					PrimitiveStates.stateTimer = TIMER_idle;
				}
				break;
			case TIMER_pausado:

				if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
					PrimitiveStates.stateTimer 	= TIMER_decrementando;
				}else{
					PrimitiveStates.stateTimer = TIMER_idle;
				}
				break;
			case TIMER_decrementando:

				PrimitiveStates.stateTimer 	= TIMER_pausado;
				break;
			}

			MACRO_ENVIA_AKNOLADGE_(RX_TOGGLE_TEMPO)
		}
		break;
		case RX_RECEITA:
		{
			MACRO_ANULA_INATIVIDADE
			//---------ENDEREÇO | 0x25 | TemperaturaTeto.hi~.lo | TemperaturaLastro.hi~.lo | Minutos | Segundos | CRC | CRC
			PrimitiveStates.Teto.setPoint 	= (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];
			PrimitiveStates.Lastro.setPoint = (bluetooth._RxDataArr[4]<< 8) | bluetooth._RxDataArr[5];

			//verifica limite
			verificaLimiteSetpoint(&PrimitiveStates.Lastro);
			verificaLimiteSetpoint(&PrimitiveStates.Teto);

			PrimitiveStates.SPTimerMinutos 	= bluetooth._RxDataArr[6];
			PrimitiveStates.SPTimerSegundos = bluetooth._RxDataArr[7];

			//atribuicao para disparar decremento
			PrimitiveStates.RTTimerMinutos = PrimitiveStates.SPTimerMinutos;
			PrimitiveStates.RTTimerSegundos = PrimitiveStates.SPTimerSegundos;


			if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
				PrimitiveStates.stateTimer 	= TIMER_decrementando;
			}else{
				PrimitiveStates.stateTimer = TIMER_idle;
			}

			MACRO_ENVIA_AKNOLADGE_(RX_RECEITA)
		}
		break;
		case RX_LIMITE_TEMPERATURA:
			//---------ENDEREÇO | 0x26 | TempMaxTeto.hi | TempMaxTeto.lo | TempMaxLastro.hi | TempMaxLastro.lo | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			PrimitiveStates.Teto.limite = (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];
			PrimitiveStates.Lastro.limite = (bluetooth._RxDataArr[4]<< 8) | bluetooth._RxDataArr[5];
			osMessagePut(FilaEepromHandle, CEepromLimiteTemp, 0);

			//verifica limite
			verificaLimiteSetpoint(&PrimitiveStates.Lastro);
			verificaLimiteSetpoint(&PrimitiveStates.Teto);

			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA, 0);
			//			MACRO_ENVIA_AKNOLADGE_(RX_LIMITE_TEMPERATURA)
			break;
		case RX_LIGA_LAMPADA:
			//---------ENDEREÇO | 0x27 | 0x27 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			onDigital(&PrimitiveStates.Lampada);
			MACRO_ENVIA_AKNOLADGE_(RX_LIGA_LAMPADA)
			break;
		case RX_DESLIGA_LAMPADA:
			//---------ENDEREÇO | 0x28 | 0x28 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			offDigital(&PrimitiveStates.Lampada);
			MACRO_ENVIA_AKNOLADGE_(RX_DESLIGA_LAMPADA)
			break;
		case RX_LIMITE_LAMPADA:
			//---------ENDEREÇO | 0x30 | 0x30 | SPLampada | CRC | CRC
			PrimitiveStates.Lampada.limitOn = bluetooth._RxDataArr[3];
			osMessagePut(FilaEepromHandle, CEepromLimiteLuz, 0);
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA, 0);
			//			MACRO_ENVIA_AKNOLADGE_(RX_LIMITE_LAMPADA)
			break;
		case RX_CANCELA_PROCESSO:
			//---------ENDEREÇO | 0x29 | 0x29 | CRC | CRC
			desligaForno();
			MACRO_ENVIA_AKNOLADGE_(RX_CANCELA_PROCESSO)
			break;

		}
	}
}

void verificaLimiteSetpoint(IndviduoPID	*canal){
	if(canal->setPoint < canal->limite)
		return;

	canal->setPoint = canal->limite;
	return;
}
