/*
 * Aplicacao.c
 *
 *  Created on: Jul 22, 2023
 *      Author: lucas
 */

#include "TaskBluetooth.h"
#include "Conversoes.h"
void initBluetooth(void);
void txBluetooth(void);
void rxBluetooth(void);
void taskBluetooth1sec(void);

#define MACRO_ANULA_INATIVIDADE tempoSemAtividade = 0; 	//variaveis do forno
static uint16_t tempoSemAtividade;						//variaveis do forno

BleComando BLEAtualizaRealtime;
BleComando BLESolicitaSincronia;
BleComando BLEAtualizaDataHora,BLEAlteraLimiteTemp,BLERestaura,BLERestauraHard,BLESPTeto,BLESPLastro,BLESPtempo,BLEToggleTempo,BLEReceita,BLESPTempo,BLELightOn,BLELightOff;
BleComando BLESetaLampada,BLECancelaProcesso,BLEToggleBuzzer,BLETunningTeto,BLETunningLastro;

void verificaLimiteSetpoint(IndviduoPID	*canal);
bool sincAutomatico(void);
typedef struct{
	bool 	flag;
	uint8_t cont;
} valuesSincronia;
valuesSincronia FlagSincronia;

void StartBluetooth(void const * argument)
{
	initBluetooth();

	for(;;)
	{
		//executado internamente no bluetooth
		txBleComando(&bluetooth);

		//lista de funcoes RECEBIDAS
		rxBluetooth();

		//lista de funcoes a serem ENVIADAS
		txBluetooth();

		osThreadYield();
		osDelay(40);
	}
}
void initBluetooth(void){

	//inicializacao do bluetooth
	//todo tratar returns
	bleConstrutora(&bluetooth, &huart1, &hdma_usart1_rx, &FilaRXBluetoothHandle,&FilaTXBluetoothHandle,&FilaBleComandoHandle);

	//possiveis comandos a serem recebidos pelo bluetooth
	bleAddComp(&bluetooth, &BLEAtualizaRealtime,	RX_SOLICITA_REALTIME	);
	bleAddComp(&bluetooth, &BLESolicitaSincronia, 	RX_SOLICITA_SINCRONIA	);
	bleAddComp(&bluetooth, &BLEAlteraLimiteTemp, 	RX_LIMITE_TEMPERATURA	);
	bleAddComp(&bluetooth, &BLEAtualizaDataHora,	RX_ATUALIZA_HORA		);
	bleAddComp(&bluetooth, &BLERestaura, 			RX_RESTAURA				);
	bleAddComp(&bluetooth, &BLERestauraHard, 		RX_RESTAURA_HARD		);
	bleAddComp(&bluetooth, &BLESPTeto,     			RX_SP_TEMP_TETO			);
	bleAddComp(&bluetooth, &BLESPLastro,     	   	RX_SP_TEMP_LASTRO		);
	bleAddComp(&bluetooth, &BLESPTempo,     		RX_SP_TEMPO				);
	bleAddComp(&bluetooth, &BLEToggleTempo,      	RX_TOGGLE_TEMPO			);
	bleAddComp(&bluetooth, &BLEReceita,     	  	RX_RECEITA				);
	bleAddComp(&bluetooth, &BLELightOn,     	  	RX_LIGA_LAMPADA			);
	bleAddComp(&bluetooth, &BLELightOff,     	 	RX_DESLIGA_LAMPADA		);
	bleAddComp(&bluetooth, &BLESetaLampada,     	RX_LIMITE_LAMPADA		);
	bleAddComp(&bluetooth, &BLECancelaProcesso,   	RX_CANCELA_PROCESSO		);
	bleAddComp(&bluetooth, &BLETunningTeto,     	RX_TUNNING_TETO			);
	bleAddComp(&bluetooth, &BLETunningLastro,  		RX_TUNNING_LASTRO		);
	bleAddComp(&bluetooth, &BLEToggleBuzzer,  		RX_TOGGLE_BUZZER		);

}
void taskBluetooth1sec(void){

	//---monitor de inatividade
	if(tempoSemAtividade>=TIME_INATIVO_SETUP){
		desligaForno();
	}else if(PrimitiveStates.stateTimer != TIMER_decrementando){
		tempoSemAtividade++;
	}

	//---sequenciamento de envio de sincronia
	if(bluetooth.MaquinaConexao == RX_DESCONECTADO){
		FlagSincronia.cont=0;
		FlagSincronia.flag=0;
	}
	/*
	 * ao conectar, seta a FlagSincronia
	 * ao desconectar, reseta a FlagSincronia
	 * */
}
void txBluetooth(void){
	unsigned char	Buffer		[BLUETOOTH_MAX_BUFF_LEN];
	osEvent  evttx;
	evttx = osMessageGet(FilaTXBluetoothHandle, 10);
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
			Buffer[13] 	= PrimitiveStates.Buzzer;

			bluetoothEnviaComando(&bluetooth,Buffer, 13);
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
			bluetoothEnviaComando(&bluetooth,Buffer, 12);
			break;
		case TX_SINCRONIA:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x18;									// FUNÇÃO -
			Buffer[2] 	= 0x01;									// Modelo
			Buffer[3] 	= (uint16_t)PrimitiveStates.Lampada.limitOn >> 8;
			Buffer[4] 	= (uint16_t)PrimitiveStates.Lampada.limitOn & 0x00FF;
			Buffer[5] 	= (uint8_t)instalacaoDia.valor;
			Buffer[6] 	= (uint8_t)instalacaoMes.valor;
			Buffer[7] 	= (uint8_t)instalacaoAno.valor;
			Buffer[8]	= VERSAO;
			Buffer[9] 	= (uint16_t)Calendario.ContMaxTeto >> 8;
			Buffer[10] 	= (uint16_t)Calendario.ContMaxTeto & 0x00FF;
			Buffer[11] 	= (uint16_t)Calendario.ContMaxLastro >> 8;
			Buffer[12] 	= (uint16_t)Calendario.ContMaxLastro & 0x00FF;
			bluetoothEnviaComando(&bluetooth,Buffer, 12);
			break;
		case TX_SINCRONIA2:
			// Endereço
			Buffer[0] = 0x01;
			Buffer[1] = 0x19;

			// kp
			float_TO_vetor4b(PrimitiveStates.Teto.kp, Buffer, 2);
			// Seguimos o mesmo processo para ki e kd
			float_TO_vetor4b(PrimitiveStates.Teto.ki, Buffer, 6);
			float_TO_vetor4b(PrimitiveStates.Teto.kd, Buffer, 10);

			// Histerese e limite, assumindo que são 16 bits (2 bytes)
			// Histerese e limite, assumindo que são 16 bits (2 bytes)
			Buffer[14] 	= (uint16_t)PrimitiveStates.Teto.histerese 	>>8;
			Buffer[15] 	= (uint16_t)PrimitiveStates.Teto.histerese 	& 0x00FF;

			Buffer[16] 	= (uint16_t)PrimitiveStates.Teto.limite 		>>8;
			Buffer[17] 	= (uint16_t)PrimitiveStates.Teto.limite 		& 0x00FF;

			bluetoothEnviaComando(&bluetooth,Buffer, 17);

			break;
		case TX_SINCRONIA3:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x20;									// FUNÇÃO -

			// kp
			float_TO_vetor4b(PrimitiveStates.Lastro.kp, Buffer, 2);
			// Seguimos o mesmo processo para ki e kd
			float_TO_vetor4b(PrimitiveStates.Lastro.ki, Buffer, 6);
			float_TO_vetor4b(PrimitiveStates.Lastro.kd, Buffer, 10);

			// Histerese e limite, assumindo que são 16 bits (2 bytes)
			Buffer[14] 	= (uint16_t)PrimitiveStates.Lastro.histerese 	>>8;
			Buffer[15] 	= (uint16_t)PrimitiveStates.Lastro.histerese 	& 0x00FF;

			Buffer[16] 	= (uint16_t)PrimitiveStates.Lastro.limite 		>>8;
			Buffer[17] 	= (uint16_t)PrimitiveStates.Lastro.limite 		& 0x00FF;

			bluetoothEnviaComando(&bluetooth,Buffer, 17);

			break;
		case TX_RESETANDO:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x29;									// FUNÇÃO -
			Buffer[2] 	= 0x29;									// FUNÇÃO -
			bluetoothEnviaComando(&bluetooth,Buffer, 2);

			break;
		case TX_RESETADO_OK:
			FlagSincronia.flag=0;
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x30;									// FUNÇÃO -
			Buffer[2] 	= 0x30;									// FUNÇÃO -
			bluetoothEnviaComando(&bluetooth,Buffer, 2);

			break;
		}
	}
}
void rxBluetooth(void){

	//todo possivelmente atualizar para sinal
	//fila nao esta fazendo sentido algum
	osEvent  evtrx;
	evtrx = osMessageGet(FilaRXBluetoothHandle, 10);
	if (evtrx.status == osEventMessage) {
		switch (bluetooth._RxDataArr[1]) {
		case RX_SOLICITA_REALTIME:
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA, 0);
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA2, 0);
			if(sincAutomatico()){
				osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA,  0);
				osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA2, 0);
				osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA3, 0);
			}

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

			bluetooth.aknowladge(&bluetooth,RX_ATUALIZA_HORA);
			break;
		case RX_RESTAURA_HARD:
			//---------ENDEREÇO | 0x09 | 0x09 | CRC | CRC
			osMessagePut(FilaEepromHandle, CEepromHardReset, 0);
			osMessagePut(FilaTXBluetoothHandle, TX_RESETANDO, 0);
			break;
		case RX_RESTAURA:
			//---------ENDEREÇO | 0x10 | 0x10 | CRC | CRC
			osMessagePut(FilaEepromHandle, CEepromSoftReset, 0);
			osMessagePut(FilaTXBluetoothHandle, TX_RESETANDO, 0);
			break;
		case RX_SP_TEMP_TETO:
			//---------ENDEREÇO | 0x21 | SP_Teto.high | SP_Teto.low | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			vetor2b_TO_Double(&PrimitiveStates.Teto.setPoint,bluetooth._RxDataArr,2);

			verificaLimiteSetpoint(&PrimitiveStates.Teto);
			bluetooth.aknowladge(&bluetooth,RX_SP_TEMP_TETO);
			break;
		case RX_SP_TEMP_LASTRO:
			//---------ENDEREÇO | 0x22 | SP_Lastro.high | SP_Lastro.low | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			vetor2b_TO_Double(&PrimitiveStates.Lastro.setPoint,bluetooth._RxDataArr,2);

			verificaLimiteSetpoint(&PrimitiveStates.Lastro);
			bluetooth.aknowladge(&bluetooth,RX_SP_TEMP_LASTRO);
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
				osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			}else{
				osSignalSet(TaskBuzzerHandle, SINAL_NEGADO);
			}
			bluetooth.aknowladge(&bluetooth,RX_SP_TEMPO);
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
					osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
				}else{
					PrimitiveStates.stateTimer = TIMER_idle;
					osSignalSet(TaskBuzzerHandle, SINAL_NEGADO);
				}
				break;
			case TIMER_pausado:

				if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
					PrimitiveStates.stateTimer 	= TIMER_decrementando;
					osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
				}else{
					PrimitiveStates.stateTimer = TIMER_idle;
				}
				break;
			case TIMER_decrementando:

				PrimitiveStates.stateTimer 	= TIMER_pausado;
				osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
				break;
			}
			bluetooth.aknowladge(&bluetooth,RX_TOGGLE_TEMPO);
		}
		break;
		case RX_RECEITA:
		{
			MACRO_ANULA_INATIVIDADE
			//---------ENDEREÇO | 0x25 | TemperaturaTeto.hi~.lo | TemperaturaLastro.hi~.lo | Minutos | Segundos | CRC | CRC

			vetor2b_TO_Double(&PrimitiveStates.Teto.setPoint,bluetooth._RxDataArr	,2);
			vetor2b_TO_Double(&PrimitiveStates.Lastro.setPoint,bluetooth._RxDataArr	,4);

			//verifica limite
			verificaLimiteSetpoint(&PrimitiveStates.Lastro);
			verificaLimiteSetpoint(&PrimitiveStates.Teto);

			PrimitiveStates.SPTimerMinutos 	= bluetooth._RxDataArr[6];
			PrimitiveStates.SPTimerSegundos = bluetooth._RxDataArr[7];

			//atribuicao para disparar decremento
			PrimitiveStates.RTTimerMinutos = PrimitiveStates.SPTimerMinutos;
			PrimitiveStates.RTTimerSegundos = PrimitiveStates.SPTimerSegundos;

			contadorOutput(&PrimitiveStates.outPuts);

			if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
				PrimitiveStates.stateTimer 	= TIMER_decrementando;
				osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			}else{
				PrimitiveStates.stateTimer = TIMER_idle;
				osSignalSet(TaskBuzzerHandle, SINAL_NEGADO);
			}
			bluetooth.aknowladge(&bluetooth,RX_RECEITA);
		}
		break;
		case RX_LIGA_LAMPADA:
			//---------ENDEREÇO | 0x27 | 0x27 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			onDigital(&PrimitiveStates.Lampada);
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA2, 0);
			bluetooth.aknowladge(&bluetooth,RX_LIGA_LAMPADA);
			break;
		case RX_DESLIGA_LAMPADA:
			//---------ENDEREÇO | 0x28 | 0x28 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			offDigital(&PrimitiveStates.Lampada);
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA2, 0);
			bluetooth.aknowladge(&bluetooth,RX_DESLIGA_LAMPADA);
			break;
		case RX_LIMITE_LAMPADA:
			//---------ENDEREÇO | 0x30 | 0x30 | SPLampada | CRC | CRC
			PrimitiveStates.Lampada.limitOn = bluetooth._RxDataArr[3];
			osMessagePut(FilaEepromHandle, CEepromLimiteLuz, 0);

			//responde sincronia
			FlagSincronia.cont=0;
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA, 0);

			//sinal sonoro
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			break;
		case RX_CANCELA_PROCESSO:
			//---------ENDEREÇO | 0x29 | 0x29 | CRC | CRC
			desligaForno();
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);

			bluetooth.aknowladge(&bluetooth,RX_CANCELA_PROCESSO);
			break;
		case RX_TUNNING_TETO:
			//---------ENDEREÇO | 0x33 | Teto.kp[4] | Teto.ki[4] | Teto.kd[4] | Teto.histerese[2] | Teto.limite[2] | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			vetor4b_TO_Double(&PrimitiveStates.Teto.kp,bluetooth._RxDataArr,2);
			vetor4b_TO_Double(&PrimitiveStates.Teto.ki,bluetooth._RxDataArr,6);
			vetor4b_TO_Double(&PrimitiveStates.Teto.kd,bluetooth._RxDataArr,10);

			vetor2b_TO_uint16(&PrimitiveStates.Teto.histerese,bluetooth._RxDataArr,14);
			vetor2b_TO_uint16(&PrimitiveStates.Teto.limite ,bluetooth._RxDataArr,16);

			//verifica limite TETO
			verificaLimiteSetpoint(&PrimitiveStates.Teto);

			//grava
			osMessagePut(FilaEepromHandle, CEepromTunning, 0);

			//responde sincronia
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA2, 0);

			//sinal sonoro
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);

			break;
		case RX_TUNNING_LASTRO:
			//---------ENDEREÇO | 0x34 | Lastro.kp[4] | Lastro.ki[4] | Lastro.kd[4] | Lastro.histerese[2] | Lastro.limite[2] | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			vetor4b_TO_Double(&PrimitiveStates.Lastro.kp,bluetooth._RxDataArr,2);
			vetor4b_TO_Double(&PrimitiveStates.Lastro.ki,bluetooth._RxDataArr,6);
			vetor4b_TO_Double(&PrimitiveStates.Lastro.kd,bluetooth._RxDataArr,10);

			vetor2b_TO_uint16(&PrimitiveStates.Lastro.histerese,bluetooth._RxDataArr,14);
			vetor2b_TO_uint16(&PrimitiveStates.Lastro.limite ,bluetooth._RxDataArr,16);

			//verifica limite TETO
			verificaLimiteSetpoint(&PrimitiveStates.Lastro);

			//grava
			osMessagePut(FilaEepromHandle, CEepromTunning, 0);

			//responde sincronia
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA3, 0);

			//sinal sonoro
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);

			break;
		case RX_TOGGLE_BUZZER:
			//---------ENDEREÇO | 0x35 | 0x35 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			PrimitiveStates.Buzzer = !PrimitiveStates.Buzzer;
			if(PrimitiveStates.Buzzer){
				osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
			}
			//grava
			osMessagePut(FilaEepromHandle, CEepromToogleBuzzer, 0);
			bluetooth.aknowladge(&bluetooth,RX_TOGGLE_BUZZER);
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
bool sincAutomatico(void){ //1==permite 0==recusa

	//envia sincronia junto do realtime apenas 3 vezes por reconexao
	if(!FlagSincronia.flag){
		if(FlagSincronia.cont<3){
			FlagSincronia.cont++; //permite 3 envios antes de resetar a flag
			return 1;
		}else{
			FlagSincronia.cont = 0; //zera o contador
			FlagSincronia.flag = 1; //reseta a flag
			return 0;
		}
	}
	return 0;
}
