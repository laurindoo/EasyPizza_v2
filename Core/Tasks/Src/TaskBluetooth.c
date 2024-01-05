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

void verificaLimiteSetpoint(IndviduoPID	*canal);
bool sincAutomatico(void);
typedef struct{
	bool 	flag;
	uint8_t cont;
} valuesSincronia;
valuesSincronia FlagSincronia;

void StartBluetooth(void const * argument)
{
	osEvent evt;

	initBluetooth();

	for(;;)
	{

		evt = osSignalWait(newMessage, osWaitForever);
		if (evt.status == osEventSignal) {

			// executado internamente no bluetooth.
			txBleComando(&bluetooth);
			// lista de funcoes RECEBIDAS (1 item por vez).
			rxBluetooth();
			// lista de funcoes a serem ENVIADAS (fila de até 10 comandos).
			txBluetooth();

		}
		// se ainda tiver item na fila de transmissao, sinaliza a task.
		if (!bluetooth.myQ_dataTx->is_empty(bluetooth.myQ_dataTx))
			osSignalSet(bluetooth.Task, newMessage);

		osDelay(40);
	}
}
void initBluetooth(void){

	//inicializacao do bluetooth
	//todo tratar returns
	bleConstrutora(&bluetooth, &huart1, &hdma_usart1_rx, TaskBluetoothHandle);

	//possiveis comandos a serem recebidos pelo bluetooth
	createBleComp(&bluetooth, RX_SOLICITA_REALTIME);
	createBleComp(&bluetooth, RX_SOLICITA_SINCRONIA);
	createBleComp(&bluetooth, RX_LIMITE_TEMPERATURA);
	createBleComp(&bluetooth, RX_RESTAURA);
	createBleComp(&bluetooth, RX_RESTAURA_HARD);
	createBleComp(&bluetooth, RX_SP_TEMP_TETO);
	createBleComp(&bluetooth, RX_SP_TEMP_LASTRO);
	createBleComp(&bluetooth, RX_SP_TEMPO);
	createBleComp(&bluetooth, RX_TOGGLE_TEMPO);
	createBleComp(&bluetooth, RX_RECEITA);
	createBleComp(&bluetooth, RX_LIGA_LAMPADA);
	createBleComp(&bluetooth, RX_DESLIGA_LAMPADA);
	createBleComp(&bluetooth, RX_LIMITE_LAMPADA);
	createBleComp(&bluetooth, RX_CANCELA_PROCESSO);
	createBleComp(&bluetooth, RX_TUNNING_TETO);
	createBleComp(&bluetooth, RX_TUNNING_LASTRO);
	createBleComp(&bluetooth, RX_TOGGLE_BUZZER);
	createBleComp(&bluetooth, RX_APAGA_ERROS);

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
	int bufferQ_dataTx;

	// exite item na lista.
	if (!bluetooth.myQ_dataTx->is_empty(bluetooth.myQ_dataTx)) {

		// remove item da lista.
		bufferQ_dataTx = bluetooth.myQ_dataTx->remove(bluetooth.myQ_dataTx);
		if (bufferQ_dataTx > 0) {

			switch ((uint8_t)bufferQ_dataTx) {
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
				Buffer[7] 	= (uint16_t)Calendario.Horimetro_horas >> 8;
				Buffer[8] 	= (uint16_t)Calendario.Horimetro_horas & 0x00FF;
				Buffer[9] 	= (uint8_t)Calendario.Horimetro_parcial_min;
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
				Buffer[5] 	= (uint8_t)0;
				Buffer[6] 	= (uint8_t)0;
				Buffer[7] 	= (uint8_t)0;
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
			case TX_ERROS:
				Buffer[0] 	= 0x01;									// ENDEREÇO
				Buffer[1] 	= 0x70;									// FUNÇÃO -
				Buffer[2] 	= eeprom.errorBuffer.errors[0];
				Buffer[3] 	= eeprom.errorBuffer.errors[1];
				Buffer[4] 	= eeprom.errorBuffer.errors[2];
				Buffer[5] 	= eeprom.errorBuffer.errors[3];
				Buffer[6] 	= eeprom.errorBuffer.errors[4];
				Buffer[7] 	= eeprom.errorBuffer.errors[5];
				Buffer[8] 	= eeprom.errorBuffer.errors[6];
				Buffer[9] 	= eeprom.errorBuffer.errors[7];
				Buffer[10] 	= eeprom.errorBuffer.errors[8];
				Buffer[11] 	= eeprom.errorBuffer.errors[9];
				bluetoothEnviaComando(&bluetooth,Buffer, 11);
				break;
			}
		}
	}
}
void rxBluetooth(void){

	// exite item na lista.
	if (!bluetooth.myQ_dataRx->is_empty(bluetooth.myQ_dataRx)) {

		// remove item da lista.
		if (bluetooth.myQ_dataRx->remove(bluetooth.myQ_dataRx) > 0) {
			switch (bluetooth.ComandoAtual._comando) {
			case RX_SOLICITA_REALTIME:
				putQueueDataTx(&bluetooth, TX_REALTIME_DATA);
				putQueueDataTx(&bluetooth, TX_REALTIME_DATA2);
				if(sincAutomatico()){
					putQueueDataTx(&bluetooth, TX_SINCRONIA);
					putQueueDataTx(&bluetooth, TX_SINCRONIA2);
					putQueueDataTx(&bluetooth, TX_SINCRONIA3);
					putQueueDataTx(&bluetooth, TX_ERROS);
				}

				break;
			case RX_SOLICITA_SINCRONIA:
				putQueueDataTx(&bluetooth, TX_SINCRONIA);
				break;
			case RX_RESTAURA_HARD:
				//---------ENDEREÇO | 0x09 | 0x09 | CRC | CRC
				osMessagePut(FilaEepromHandle, CEepromHardReset, 0);
				putQueueDataTx(&bluetooth, TX_RESETANDO);
				osDelay(50);
				putQueueDataTx(&bluetooth, TX_RESETADO_OK);
				break;
			case RX_RESTAURA:
				//---------ENDEREÇO | 0x10 | 0x10 | CRC | CRC
				osMessagePut(FilaEepromHandle, CEepromSoftReset, 0);
				putQueueDataTx(&bluetooth, TX_RESETANDO);
				osDelay(50);
				putQueueDataTx(&bluetooth, TX_RESETADO_OK);

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

//				if(PrimitiveStates.Lastro._PWMstate != buscando && PrimitiveStates.Teto._PWMstate != buscando){
//					PrimitiveStates.stateTimer 	= TIMER_decrementando;
//					osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
//				}else{
//					osSignalSet(TaskBuzzerHandle, SINAL_NEGADO);
//				}

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

				contadorOutput(&outPuts);

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
				putQueueDataTx(&bluetooth, TX_REALTIME_DATA2);
				bluetooth.aknowladge(&bluetooth,RX_LIGA_LAMPADA);
				break;
			case RX_DESLIGA_LAMPADA:
				//---------ENDEREÇO | 0x28 | 0x28 | CRC | CRC
				MACRO_ANULA_INATIVIDADE
				osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);
				offDigital(&PrimitiveStates.Lampada);
				putQueueDataTx(&bluetooth, TX_REALTIME_DATA2);
				bluetooth.aknowladge(&bluetooth,RX_DESLIGA_LAMPADA);
				break;
			case RX_LIMITE_LAMPADA:
				//---------ENDEREÇO | 0x30 | 0x30 | SPLampada | CRC | CRC
				PrimitiveStates.Lampada.limitOn = bluetooth._RxDataArr[3];
				osMessagePut(FilaEepromHandle, CEepromLimiteLuz, 0);

				//responde sincronia
				FlagSincronia.cont=0;
				putQueueDataTx(&bluetooth, TX_SINCRONIA);

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

				// update dos valores na classe de PID obj teto.
				PID_SetTunings(&TPIDTeto, PrimitiveStates.Teto.kp, PrimitiveStates.Teto.ki, PrimitiveStates.Teto.kd);

				//responde sincronia
				putQueueDataTx(&bluetooth, TX_SINCRONIA2);

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

				// update dos valores na classe de PID obj lastro.
				PID_SetTunings(&TPIDLastro, PrimitiveStates.Lastro.kp, PrimitiveStates.Lastro.ki, PrimitiveStates.Lastro.kd);

				//responde sincronia
				putQueueDataTx(&bluetooth, TX_SINCRONIA3);

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
			case RX_APAGA_ERROS:
				//---------ENDEREÇO | 0x75 | 0x75 | CRC | CRC
				MACRO_ANULA_INATIVIDADE
				osSignalSet(TaskBuzzerHandle, SINAL_COMFIRMA);

				// limpa erros.
				osMessagePut(FilaEepromHandle, CEepromClearErrors, 0);
				bluetooth.aknowladge(&bluetooth,RX_APAGA_ERROS);
				break;
			}
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
		if(FlagSincronia.cont<ENVIO_DE_SINCRONIAS){
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
