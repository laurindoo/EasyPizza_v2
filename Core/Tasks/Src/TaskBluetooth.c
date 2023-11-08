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
BleComando BLEPedeSenha,BLERecebeuSenha,BLESetaLampada,BLECancelaProcesso;


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
	BluetoothAddComp(&bluetooth, &BLEPedeSenha,   		"RX_PEDE_SENHA",  			RX_PEDE_SENHA,   			ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLERecebeuSenha,     	"RX_RECEBEU_SENHA",        	RX_RECEBEU_SENHA,          	ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESPTeto,     		"RX_SP_TEMP_TETO",        	RX_SP_TEMP_TETO,          	ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESPLastro,     		"RX_SP_TEMP_LASTRO",       	RX_SP_TEMP_LASTRO,          ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESPTempo,     		"RX_SP_TEMPO",       		RX_SP_TEMPO,        		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLEToggleTempo,     	"RX_TOGGLE_TEMPO",       	RX_TOGGLE_TEMPO,        	ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLEReceita,     		"RX_RECEITA",     		  	RX_RECEITA,        			ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLELightOn,     		"RX_LIGA_LAMPADA",     	  	RX_LIGA_LAMPADA,     		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLELightOff,     		"RX_DESLIGA_LAMPADA",    	RX_DESLIGA_LAMPADA,  		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLESetaLampada,     	"RX_LIMITE_LAMPADA",    	RX_LIMITE_LAMPADA,  		ComandoBasico);
	BluetoothAddComp(&bluetooth, &BLECancelaProcesso,  	"RX_CANCELA_PROCESSO",    	RX_CANCELA_PROCESSO,  		ComandoBasico);
}

void txBluetooth(void){
	unsigned char	Buffer		[BLUETOOTH_MAX_BUFF_LEN];//todo cogitar colocar na classe
	osEvent  evttx;
	evttx = osMessageGet(FilaTXBluetoothHandle, 0);
	if (evttx.status == osEventMessage) {
		switch ((unsigned int)evttx.value.p) {
		case TX_REALTIME_DATA:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x16;									// FUNÇÃO -
			Buffer[2] 	= PrimitiveStates.Erro.byte;							// Conjunto de erros
			Buffer[3] 	= PrimitiveStates.stateMaquina;			// State da maquina
			Buffer[4] 	= (uint16_t)PrimitiveStates.RealtimeTeto 	>>8;
			Buffer[5] 	= (uint16_t)PrimitiveStates.RealtimeTeto 	& 0x00FF;
			Buffer[6] 	= (uint16_t)PrimitiveStates.SetPointTeto 	>>8;
			Buffer[7] 	= (uint16_t)PrimitiveStates.SetPointTeto 	& 0x00FF;
			Buffer[8] 	= (uint16_t)PrimitiveStates.RealtimeLastro 	>>8;
			Buffer[9] 	= (uint16_t)PrimitiveStates.RealtimeLastro 	& 0x00FF;
			Buffer[10] 	= (uint16_t)PrimitiveStates.SetPointLastro 	>>8;
			Buffer[11] 	= (uint16_t)PrimitiveStates.SetPointLastro 	& 0x00FF;
			BluetoothEnviaComando(Buffer, 11);
			osDelay(10);
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA2, 0);
			break;
		case TX_REALTIME_DATA2:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x17;									// FUNÇÃO -
			Buffer[2] 	= PrimitiveStates.RTTimerMinutos;
			Buffer[3] 	= PrimitiveStates.RTTimerSegundos;
			Buffer[4] 	= PrimitiveStates.SPTimerMinutos;
			Buffer[5] 	= PrimitiveStates.SPTimerSegundos;
			Buffer[6] 	= (uint8_t)horimetroHoras.valor >> 8;
			Buffer[7] 	= (uint8_t)horimetroHoras.valor & 0x00FF;
			Buffer[8] 	= (uint8_t)horimetroMinutos.valor;
			Buffer[9]	= PrimitiveStates.Lampada;
			BluetoothEnviaComando(Buffer, 9);
			break;
		case TX_SINCRONIA:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x18;									// FUNÇÃO -
			Buffer[2] 	= 0x01;									// Modelo
			Buffer[3] 	= (uint8_t)tempoDelayLuz.valor;
			Buffer[4] 	= (uint8_t)LimiteTemperatura.valor	>> 8 ;
			Buffer[5] 	= (uint8_t)LimiteTemperatura.valor	& 0x00ff ;
			Buffer[6] 	= (uint8_t)instalacaoDia.valor;
			Buffer[7] 	= (uint8_t)instalacaoMes.valor;
			Buffer[8] 	= (uint8_t)instalacaoAno.valor;
			Buffer[9]	= VERSAO;
			BluetoothEnviaComando(Buffer, 9);
			break;
		case TX_CHAVE:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x51;									// FUNÇÃO -
			Buffer[2] 	= 0x51;									// FUNÇÃO -
			Buffer[3] 	= 0x01;
			Buffer[4] 	= bluetooth.chave >> 8 		;
			Buffer[5] 	= bluetooth.chave & 0x00ff	;
			BluetoothEnviaComando(Buffer, 5);
			break;
		case TX_CHAVE_ERRO:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x51;									// FUNÇÃO -
			Buffer[2] 	= 0x51;									// FUNÇÃO -
			Buffer[3] 	= 0x00;
			Buffer[4] 	= 0x00;
			Buffer[5] 	= 0x00;
			BluetoothEnviaComando(Buffer, 5);

			BluetoothDescon(&bluetooth);
			break;
		case TX_RESULTADO_CHAVE_OK:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x52;									// FUNÇÃO -
			Buffer[2] 	= 0x52;									// FUNÇÃO -
			Buffer[3] 	= 0x01;									//resultado ok
			BluetoothEnviaComando(Buffer, 3);
			break;
		case TX_RESULTADO_CHAVE_ERRO:
			Buffer[0] 	= 0x01;									// ENDEREÇO
			Buffer[1] 	= 0x52;									// FUNÇÃO -
			Buffer[2] 	= 0x52;									// FUNÇÃO -
			Buffer[3] 	= 0x00;									//resultado ok
			BluetoothEnviaComando(Buffer, 3);

			HAL_Delay(30);
			Envia_texto_UART("AT",50);//DESCONECTA
			break;
		}
	}
}

void rxBluetooth(void){
	osEvent  evtrx;
	evtrx = osMessageGet(FilaRXBluetoothHandle, 0);
	if (evtrx.status == osEventMessage) {
		//			switch ((unsigned int)evtrx.value.p) {
		switch (bluetooth._RxDataArr[1]) {
		case RX_SOLICITA_REALTIME:
			osMessagePut(FilaTXBluetoothHandle, TX_REALTIME_DATA, 0);
			break;
		case RX_SOLICITA_SINCRONIA:
			osMessagePut(FilaTXBluetoothHandle, TX_SINCRONIA, 0);
			break;
		case RX_ATUALIZA_HORA:
			//todo revisar
			//				sDate.WeekDay 	= bluetooth._RxDataArr[2]; //Dia da semana p/atualizar
			//				sDate.Date 		= bluetooth._RxDataArr[3]; //Dia do mes p/atualizar
			//				sDate.Month 	= bluetooth._RxDataArr[4]; //mes p/atualizar
			//				sDate.Year 		= bluetooth._RxDataArr[5]; //ano p/atualizar
			//				sTime.Hours 	= bluetooth._RxDataArr[6]; //hora p/atualizar
			//				sTime.Minutes 	= bluetooth._RxDataArr[7]; //minuto p/atualizar
			//				sTime.Seconds 	= bluetooth._RxDataArr[8]; //segundos p/atualizar
			osMessagePut(FilaEepromHandle, CEepromAtualizaHora, 0);

			MACRO_ENVIA_AKNOLADGE_(bluetooth._RxDataArr[1])
			break;
		case RX_RESTAURA:
			//				//---------ENDEREÇO | 0x10 | 0x10  | CRC | CRC
			//				Maquina.Maquina_eeprom = EEPROM_HARD_RESET;
			//				MACRO_ENVIA_AKNOLADGE_(RX_RESTAURA)
			break;
		case RX_SP_TEMP_TETO:
			//---------ENDEREÇO | 0x21 | SP_Teto.high | SP_Teto.low | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			uint16_t aux;
			aux = (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];
			PrimitiveStates.SetPointTeto = (double)aux;
			MACRO_ENVIA_AKNOLADGE_(RX_SP_TEMP_TETO)
			break;
		case RX_SP_TEMP_LASTRO:
			//---------ENDEREÇO | 0x22 | SP_Lastro.high | SP_Lastro.low | CRC | CRC
			MACRO_ANULA_INATIVIDADE

			PrimitiveStates.SetPointLastro = (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];

			MACRO_ENVIA_AKNOLADGE_(RX_SP_TEMP_LASTRO)
			break;
		case RX_SP_TEMPO:
		{	//---------ENDEREÇO | 0x23 | TimerMinutos | TimerSegundos |CRC | CRC
			MACRO_ANULA_INATIVIDADE

			PrimitiveStates.SPTimerMinutos 	= bluetooth._RxDataArr[2];
			PrimitiveStates.SPTimerSegundos = bluetooth._RxDataArr[3];

			PrimitiveStates.RTTimerMinutos = PrimitiveStates.SPTimerMinutos;
			PrimitiveStates.RTTimerSegundos = PrimitiveStates.SPTimerSegundos;

			switch (PrimitiveStates.stateMaquina) {
			case inicial:
			case aquecendo:
				//nao executa nada
				__NOP();
				break;
			case aquecido:
			case decrementando:
			case pausado:
				if(PrimitiveStates.MaquinaAquecimento == mantendoTemp){
					PrimitiveStates.stateMaquina 	= decrementando;
				}else{
					PrimitiveStates.stateMaquina = aquecendo;
				}
				break;
			}

			MACRO_ENVIA_AKNOLADGE_(RX_SP_TEMPO)
		}
		break;
		case RX_TOGGLE_TEMPO:
		{
			//---------ENDEREÇO | 0x24 | 0x24 | TimerSegundos |CRC | CRC
			MACRO_ANULA_INATIVIDADE

			switch (PrimitiveStates.stateMaquina) {
			case inicial:
			case aquecendo:
				//nao executa nada
				__NOP();
				break;
			case aquecido:
				PrimitiveStates.stateMaquina = decrementando;
				break;
			case decrementando:
				PrimitiveStates.stateMaquina = pausado;
				break;
			case pausado:
				if(PrimitiveStates.MaquinaAquecimento == mantendoTemp){
					PrimitiveStates.stateMaquina = decrementando;
				}else{
					PrimitiveStates.stateMaquina = aquecendo;
				}
				break;
			}

			MACRO_ENVIA_AKNOLADGE_(RX_TOGGLE_TEMPO)
		}
		break;
		case RX_RECEITA:
		{
			MACRO_ANULA_INATIVIDADE
			//---------ENDEREÇO | 0x25 | TemperaturaTeto.hi~.lo | TemperaturaLastro.hi~.lo | Minutos | Segundos | CRC | CRC
			PrimitiveStates.SetPointTeto = (bluetooth._RxDataArr[2]<< 8) | bluetooth._RxDataArr[3];
			PrimitiveStates.SetPointLastro = (bluetooth._RxDataArr[4]<< 8) | bluetooth._RxDataArr[5];
			PrimitiveStates.SPTimerMinutos 	= bluetooth._RxDataArr[6];
			PrimitiveStates.SPTimerSegundos = bluetooth._RxDataArr[7];

			//atribuicao para disparar decremento
			PrimitiveStates.RTTimerMinutos = PrimitiveStates.SPTimerMinutos;
			PrimitiveStates.RTTimerSegundos = PrimitiveStates.SPTimerSegundos;


			switch (PrimitiveStates.stateMaquina) {
			case inicial:
			case aquecendo:
				//nao executa nada
				__NOP();
				break;
			case aquecido:
			case decrementando:
			case pausado:
				if(PrimitiveStates.MaquinaAquecimento == mantendoTemp){
					PrimitiveStates.stateMaquina 	= decrementando;
				}else{
					PrimitiveStates.stateMaquina = aquecendo;
				}
				break;
			}
			MACRO_ENVIA_AKNOLADGE_(RX_RECEITA)
		}
		break;
		case RX_LIMITE_TEMPERATURA:
			//---------ENDEREÇO | 0x26 | TemperaturaTeto.hi~.lo | TemperaturaLastro.hi~.lo | CRC | CRC
			MACRO_ENVIA_AKNOLADGE_(RX_LIMITE_TEMPERATURA)
			break;
		case RX_LIGA_LAMPADA:
			//---------ENDEREÇO | 0x27 | 0x27 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			PrimitiveStates.RTLampada=PrimitiveStates.SPLampada;
			LAMPADA_ON
			MACRO_ENVIA_AKNOLADGE_(RX_LIGA_LAMPADA)
			break;
		case RX_DESLIGA_LAMPADA:
			//---------ENDEREÇO | 0x28 | 0x28 | CRC | CRC
			MACRO_ANULA_INATIVIDADE
			LAMPADA_OFF
			MACRO_ENVIA_AKNOLADGE_(RX_DESLIGA_LAMPADA)
			break;
		case RX_LIMITE_LAMPADA:
			//---------ENDEREÇO | 0x30 | 0x30 | SPLampada | CRC | CRC
			//todo sequencia salvar limite lampada eeprom
			PrimitiveStates.SPLampada = bluetooth._RxDataArr[2];
			MACRO_ENVIA_AKNOLADGE_(RX_LIMITE_LAMPADA)
			break;
		case RX_CANCELA_PROCESSO:
			//---------ENDEREÇO | 0x29 | 0x29 | CRC | CRC
			desligaForno();
			MACRO_ENVIA_AKNOLADGE_(RX_CANCELA_PROCESSO)
			break;
		case RX_PEDE_SENHA:
			if(bluetooth.JanelaConexao > 0)
				osMessagePut(FilaTXBluetoothHandle, TX_CHAVE, 0);
			else{
				osMessagePut(FilaTXBluetoothHandle, TX_CHAVE_ERRO, 0);
			}
			break;
		case RX_RECEBEU_SENHA:
			if(		bluetooth._RxDataArr[3] == (bluetooth.chave >> 8) &&
					bluetooth._RxDataArr[4] == (bluetooth.chave & 0x00ff) ){
				//--->	CHAVE CORRETA
				bluetooth.MaquinaConexao	= RX_VALIDADO;
				osMessagePut(FilaTXBluetoothHandle, TX_RESULTADO_CHAVE_OK, 0);
				break;
			}else{
				//--->	CHAVE ERRADA
				osMessagePut(FilaTXBluetoothHandle, TX_RESULTADO_CHAVE_ERRO, 0);
			}
			break;
		}
	}
}

