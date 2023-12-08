/*
 * Bluetooth.c
 *
 *  Created on: Jul 24, 2023
 *      Author: lucas
 */

#include "Bluetooth.h"

//VAR PARA CAPTURA DO ADDR ---> endereco mac
uint8_t 	addr8Bits	[12];	//VAR QUE ARMAZENA ENDERE�O DO DISPOSITIVO
uint32_t 	addr32Bits	[3];	//VAR QUE ARMAZENA ENDERE�O DO DISPOSITIVO
extern uint8_t stepBle; // todo apagar
//HANDLES GLOBAIS
UART_HandleTypeDef 	*UARTHandle;
DMA_HandleTypeDef 	*UARTDMAHandle;
CRC_short 			CRCReceive;

extern osThreadId TaskBluetoothHandle;

BleComando BLEPedeSenha,BLERecebeuSenha;
uint8_t BluetoothInit(Bluetooth *ble, UART_HandleTypeDef *bluetoothUARTHandle, DMA_HandleTypeDef *bluetoothUARTDMAHandle, osMessageQId *filaRX, osMessageQId *filaTX){
	//Pass the used UART handle to the struct
	ble->UARTHandle 	= bluetoothUARTHandle;
	ble->UARTDMAHandle 	= bluetoothUARTDMAHandle;

	UARTHandle 			= ble->UARTHandle;
	UARTDMAHandle 		= ble->UARTDMAHandle;

	//Pass the used queue to the struct
	ble->filaComandosRX = filaRX;
	ble->filaComandosTX = filaTX;

	//Start the component count variable from zero
	ble->_BleCommCount  = 0;

	ble->JanelaConexao = 120;//120 segundos

	//inicializacao do hardware
	iniciaBleHm10(ble);

	BluetoothAddComp(ble, &BLEPedeSenha,   		"RX_PEDE_SENHA",  			RX_PEDE_SENHA,   			ComandoConexao);
	BluetoothAddComp(ble, &BLERecebeuSenha,     "RX_RECEBEU_SENHA",        	RX_RECEBEU_SENHA,          	ComandoConexao);

	//Return OK
	return 0;
}
uint8_t BluetoothAddComp(Bluetooth* ble, BleComando* _blecomm, char* objectname, uint8_t __comando, TypeComandoBle __tipo){
	//Make space before passing the object name to the nexcomp struct
	_blecomm->objname = (char *) malloc((strlen(objectname)*sizeof(char)) + 1);
	//Pass the object name to the struct
	strcpy(_blecomm->objname, objectname);

	//Pass the corresponding data from component to component struct
	_blecomm->_comando = __comando;
	_blecomm->_tipo = __tipo;

	//Add the component struct to the list on the Nextion Struct
	ble->_BleCommArr[ble->_BleCommCount] = _blecomm;
	ble->_BleCommCount++;

	//Return OK
	return 0;
}
void BluetoothPutFila(Bluetooth* ble){

	//Varregura pelos comandos -----------------------------
	for(uint8_t i = 0; i < ble->_BleCommCount; i++)	{

		//Detecta por comando--------------------------------
		if( ble->_RxDataArr[1] == (ble->_BleCommArr[i]->_comando)){

			//Validacao de CRC-----------------------------------
			CRCReceive.hilo = CRC16(ble->_RxDataArr,ble->RxSize-2);
			if( 	(CRCReceive.byte.hi	!= ble->_RxDataArr[ble->RxSize-2])||//COMPARACAO CRC_LOCAL - CRC_RECEBIDO
					(CRCReceive.byte.lo != ble->_RxDataArr[ble->RxSize-1]))	{
				BluetoothErroCRC();//erro de CRC
				return; //ENCERRA
			}

			//Comandos de conexao--------------------------------
			if(ble->_BleCommArr[i]->_tipo == ComandoConexao ){

				cancelaAntecipacao(ble);

				switch (ble->_RxDataArr[1]) {
				case RX_PEDE_SENHA:
					solicitacaoSenhaBluetooh(ble);
					break;
				case RX_RECEBEU_SENHA:
					avaliaSenhaRecebidaBluetooh(ble);
					break;
				}
			}else if(ble->_BleCommArr[i]->_tipo == ComandoBasico && ble->MaquinaConexao == RX_VALIDADO ){

				cancelaAntecipacao(ble);

				//Comandos normais (EXIGE SENHA E VALIDACAO-----------------
				osMessagePut(*ble->filaComandosRX, ble->_BleCommArr[i]->_comando, 0);
			}
			else{
				BluetoothDescon(ble);
			}
		}
	}
}
void solicitacaoSenhaBluetooh(Bluetooth* ble){
	unsigned char	Buffer		[20];

	if(ble->JanelaConexao > 0){

		/*----DENTRO DO TEMPO, ENTAO RESPONDE----*/
		Buffer[0] 	= 0x01;									// ENDEREÇO
		Buffer[1] 	= 0x51;									// FUNÇÃO -
		Buffer[2] 	= 0x51;									// FUNÇÃO -
		Buffer[3] 	= 0x01;
		Buffer[4] 	= ble->chave >> 8 		;
		Buffer[5] 	= ble->chave & 0x00ff	;
		BluetoothEnviaComando(Buffer, 5);

	}else{
		/*----FORA DO TEMPO DE RESPOSTA ----*/
		Buffer[0] 	= 0x01;									// ENDEREÇO
		Buffer[1] 	= 0x51;									// FUNÇÃO -
		Buffer[2] 	= 0x51;									// FUNÇÃO -
		Buffer[3] 	= 0x00;
		Buffer[4] 	= 0x00;
		Buffer[5] 	= 0x00;
		BluetoothEnviaComando(Buffer, 5);
	}
}
void avaliaSenhaRecebidaBluetooh(Bluetooth* ble){
	unsigned char	Buffer		[20];

	if(		ble->_RxDataArr[3] == (ble->chave >> 8) &&
			ble->_RxDataArr[4] == (ble->chave & 0x00ff) ){
		//--->	CHAVE CORRETA
		ble->MaquinaConexao	= RX_VALIDADO;
		Buffer[0] 	= 0x01;									// ENDEREÇO
		Buffer[1] 	= 0x52;									// FUNÇÃO -
		Buffer[2] 	= 0x52;									// FUNÇÃO -
		Buffer[3] 	= 0x01;									//resultado ok
		BluetoothEnviaComando(Buffer, 3);

	}else{
		//--->	CHAVE ERRADA
		Buffer[0] 	= 0x01;									// ENDEREÇO
		Buffer[1] 	= 0x52;									// FUNÇÃO -
		Buffer[2] 	= 0x52;									// FUNÇÃO -
		Buffer[3] 	= 0x00;									//resultado ok
		BluetoothEnviaComando(Buffer, 3);

		BluetoothDescon(ble);
	}
}
void BLEUSART_IrqHandler(Bluetooth *ble)
{
	if (UARTHandle->Instance->SR & UART_FLAG_IDLE) {    /* if Idle flag is set */
		__IO uint32_t __attribute__((unused))tmp;      	/* Must be volatile to prevent optimizations */

		tmp = UARTHandle->Instance->SR;                 /* Read status register */
		tmp = UARTHandle->Instance->DR;                 /* Read data register */
		__HAL_DMA_DISABLE (UARTDMAHandle);       		/* Disabling DMA will force transfer complete interrupt if enabled */

		__HAL_UART_ENABLE_IT 	(UARTHandle, UART_IT_IDLE);		// HABILITA idle line INTERRUPT
		__HAL_DMA_ENABLE_IT 	(UARTDMAHandle, DMA_IT_TC);		// HABILITA O DMA Tx cplt INTERRUPT

		BLEDMA_IrqHandler (ble);
	}
}
void BLEDMA_IrqHandler (Bluetooth *ble)
{
	if(__HAL_DMA_GET_IT_SOURCE(UARTDMAHandle, DMA_IT_TC) != RESET){   // if the source is TC

		/* Clear the transfer complete flag */
		__HAL_DMA_CLEAR_FLAG(UARTDMAHandle, __HAL_DMA_GET_TC_FLAG_INDEX(UARTDMAHandle));

		//zera contador de inatividade
		ble->msIdle=0;

		//calculo do tamanho da string recebida
		ble->RxSize 		= DMA_RX_BUFFER_SIZE - UARTDMAHandle->Instance->CNDTR;

		//verifica se nao caiu conexao
		sprintf(ble->StringRecebida,"%s",ble->_RxDataArr);

		//zera contador de mensagem, pois recebeu mensagem
		ble->msDesconectado = 0;

		//-------------------------MAQUINA CONEXAO--------------------------
		switch(ble->MaquinaConexao){
		case RX_DESCONECTADO:

			/*---   C O N E C T O U  ---*/
			ble->StatusSenha = false;//chave de validacao
			ble->ss = NULL;
			ble->ss = strstr(ble->StringRecebida, "OK+CONN");
			if (((ble->ss != NULL  && ble->RxSize == 7) || MACRO_LE_BT_STATUS) && ble->SistemaInit){

				ble->MaquinaConexao = RX_CONECTADO;
				ble->StatusConexao 	= true;
			}

			/*---   A D D R  ---*/
			ble->ss = NULL;
			ble->ss = strstr(ble->StringRecebida, "OK+ADDR:");
			if (ble->ss != NULL){
				ble->PontoExato = ble->ss - ble->StringRecebida;
				for (int i = 0; i < 12; i++) {
					addr8Bits[i] = ble->_RxDataArr[i+ble->PontoExato+8];
				}

				addr32Bits[0] = (addr8Bits[0]<<24)+(addr8Bits[1]<<16)+(addr8Bits[2]<<8)+(addr8Bits[3]);
				addr32Bits[1] = (addr8Bits[4]<<24)+(addr8Bits[5]<<16)+(addr8Bits[6]<<8)+(addr8Bits[7]);
				addr32Bits[2] = (addr8Bits[8]<<24)+(addr8Bits[9]<<16)+(addr8Bits[10]<<8)+(addr8Bits[11]);

				ble->chave = CRC16(addr8Bits,12);

				osSignalSet(TaskBluetoothHandle, CHEGOU_ADDR_BLE);
			}
			break;
		case RX_VALIDADO:
		case RX_CONECTADO:
			/*---   D E S C O N E C T O U  ---*/
			ble->ss = NULL;
			ble->ss = strstr(ble->StringRecebida, "LOST");
			if (ble->ss != NULL || !MACRO_LE_BT_STATUS){
				ble->StatusSenha 		= false;//chave de validacao
				ble->MaquinaConexao 	= RX_DESCONECTADO;
				ble->StatusConexao 		= false;
			}else if(ble->SistemaInit){
				BluetoothPutFila(ble);
			}

			break;
		}

		/* Prepare DMA for next transfer */
		/* Important! DMA stream won't start if all flags are not cleared first */
		UARTDMAHandle->Instance->CMAR = (uint32_t)ble->_RxDataArr;   /* Set memory address for DMA again */
		UARTDMAHandle->Instance->CNDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
		UARTDMAHandle->Instance->CCR |= DMA_CCR_EN;            /* Start DMA transfer */
	}
}
void BluetoothEnviaComando(unsigned char _out[], int size)
{
	uint8_t	TX_Buffer		[size+3];
	unsigned short CRCVar;

	//varredura para local
	for (int i = 0; i <= size; ++i) {
		TX_Buffer[i]=_out[i];
	}

	//calculo e atribuicao do crc
	CRCVar = CRC16(_out,size+1);
	TX_Buffer[size+2] = (unsigned char) (CRCVar >> 8);
	TX_Buffer[size+1] = (unsigned char) (CRCVar & 0x00FF);

	Envia_bytes_UART((uint8_t *)TX_Buffer,size+3);
}
void Envia_bytes_UART(unsigned char _out[], uint8_t size){
	HAL_UART_Transmit(UARTHandle, (uint8_t *)_out, size,50);
}
void Envia_texto_UART(char _out[], uint16_t delay){
	HAL_UART_Transmit(UARTHandle, (uint8_t *) _out, strlen(_out),100);
	if(delay != 0){
		osDelay(delay);
	}
}
unsigned short CRC16 (unsigned char *puchMsg, unsigned short usDataLen)
{
	static unsigned char auchCRCHi[] = {
			0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
			0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
			0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
			0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
			0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
			0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
			0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
			0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
			0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
			0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
			0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
			0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
			0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
			0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
			0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
			0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
			0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
			0x40
	} ;
	/* Table of CRC values for low�order byte */
	static char auchCRCLo[] = {
			0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
			0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
			0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
			0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
			0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
			0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
			0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
			0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
			0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
			0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
			0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
			0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
			0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
			0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
			0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
			0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
			0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
			0x40
	};

	unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	unsigned uIndex ; /* will index into CRC lookup table */
	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCLo ^ *puchMsg++ ; /* calculate the CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
		uchCRCHi = auchCRCLo[uIndex] ;
	}
	return (uchCRCHi << 8 | uchCRCLo) ;
}//---END---//
void iniciaBleHm10(Bluetooth* ble){
#define M_BLE_RESET Envia_texto_UART("AT+RESET",400);//RESETA
#define SETUP_UART(baud_rate) \
		HAL_UART_Abort_IT(UARTHandle);\
		HAL_UART_DeInit(UARTHandle);\
		osDelay(50);\
		UARTHandle->Init.BaudRate = baud_rate;\
		HAL_UART_Init(UARTHandle);\
		osDelay(50);


	while(ble->sequenciaBLE !=final || ble->sequenciaBLE!=erro){
		switch (ble->sequenciaBLE) {
		case inicio:
			stepBle=1;

			MACRO_RESET_BLE		//HARDRESET NO BLE_HM10
			Envia_texto_UART("AT+ADTY3",100);	//BLOQUEIA CONEXAO
			Envia_texto_UART("AT+ADTY3",100);	//BLOQUEIA CONEXAO
			SETUP_UART(115200)
			stepBle=2;
			BluetoothDescon(ble);
			MACRO_DEFINE_INTERRUPT

			//			ble->sequenciaBLE = verificaNome;
			ble->sequenciaBLE = redefineBle;

			break;
		case verificaNome:

			static uint8_t tryingName=0;
			const uint8_t max_attempts = 15;
			const uint32_t delay_between_attempts_ms = 1000;
			stepBle=3;
			while (tryingName < max_attempts) {
				Envia_texto_UART("AT+NAME?", 100);
				MACRO_DEFINE_INTERRUPT
				osDelay(delay_between_attempts_ms);
				ble->ss = NULL;
				ble->ss = strstr(ble->StringRecebida, "NAME");

				if (ble->ss != NULL){
					ble->ss = strstr(ble->StringRecebida, BLE_DEVICE_NAME);
					if (ble->ss != NULL){
						ble->sequenciaBLE = capturaAddr;
						break;
					} else {
						ble->sequenciaBLE = redefineBle;
						break;
					}
				} else {
					tryingName++;
				}
			}
			stepBle=4;
			if(tryingName >= max_attempts)
				ble->sequenciaBLE = redefineBle;//extrapolou as tentativas

			break;
		case redefineBle:

			//seta em 115200
			SETUP_UART(115200)
			Envia_texto_UART("AT",100);	//
			Envia_texto_UART("AT",100);	//
			Envia_texto_UART("AT+RENEW",1000);	//RESTAURA PADRAO FABRICA
			//seta em 9600
			SETUP_UART(9600)
			Envia_texto_UART("AT+RENEW",1000);	//RESTAURA PADRAO FABRICA

			Envia_texto_UART("AT",100);	//
			Envia_texto_UART("AT",100);	//
			Envia_texto_UART("AT+ADTY3",300);	//BLOQUEIA CONEXAO
			Envia_texto_UART("AT+BAUD4",300);	//COLOCA BAUD EM 115200
			//seta em 115200
			SETUP_UART(115200)
			//	M_BLE_RESET
			MACRO_RESET_BLE

			//CONFIGURA CENTRAL
			Envia_texto_UART("AT",100);	//
			Envia_texto_UART("AT",100);	//
			Envia_texto_UART("AT+POWE3",300);	//POTENCIA MAXIMA
			Envia_texto_UART("AT+SHOW3",300);	//MOSTRA O NOME e rssi
			Envia_texto_UART("AT+GAIN1",300);	//INSERE GANHO
			Envia_texto_UART("AT+NOTI1",300);	//NOTIFICA QUE CONECTOU
			Envia_texto_UART("AT+PIO11",300);	//1 - CONECT = 1  \  DISC = 0
			char comando[COMANDO_BUFFER_SIZE]; // Buffer para o comando AT
			snprintf(comando, sizeof(comando), "AT+NAME%s", BLE_DEVICE_NAME);
			Envia_texto_UART(comando, 400); // Configura o nome no dispositivo

			M_BLE_RESET

			ble->sequenciaBLE = capturaAddr;

			break;
		case capturaAddr:
			static uint8_t tryingAddr=0;
			stepBle = 8;
			while (tryingAddr < max_attempts) {

				stepBle = 9;
				Envia_texto_UART("AT+ADDR?",300);//pede addr
				MACRO_DEFINE_INTERRUPT
				//				osDelay(800);

				if (ble->chave != 0){
					ble->sequenciaBLE = final;
					MACRO_DEFINE_INTERRUPT
					tryingAddr=0;
					break;
				} else {
					tryingAddr++;
					break;
				}
			}

			if(tryingAddr >= max_attempts){
				ble->sequenciaBLE = erro;//extrapolou as tentativas
			}
			break;
		case final:
			stepBle  =10;
			Envia_texto_UART("AT+ADTY0",300);	//DESBLOQUEIA CONEXA
			MACRO_RESET_BLE
			//			osDelay(50);
			/*---HABILITA INTERRUPÇÃO---*/
			__HAL_UART_ENABLE_IT 	(UARTHandle, UART_IT_IDLE);						// HABILITA idle line INTERRUPT
			__HAL_DMA_ENABLE_IT 	(UARTDMAHandle, DMA_IT_TC);					// HABILITA O DMA Tx cplt INTERRUPT
			HAL_UART_Receive_DMA 	(UARTHandle, ble->_RxDataArr, DMA_RX_BUFFER_SIZE);	// STARTA O UART1 EM DMA MODE
			ble->SistemaInit = 1;

			return;
			break;
		case erro:
			NVIC_SystemReset();
			break;
		default:
			break;
		}
	}
}
void BluetoothErroCRC(void)
{
	unsigned char	TXCRC[3];
	TXCRC[0] = 0x01;\
	TXCRC[1] = 0xEE;\
	TXCRC[2] = 0xEE;\
	Envia_bytes_UART(TXCRC,3);
}
void BluetoothDescon(Bluetooth* ble){
	unsigned char	Buffer		[20];

	Envia_texto_UART("AT",50);//DESCONECTA
	Envia_texto_UART("AT",50);//DESCONECTA

	/* Prepare DMA for next transfer */
	/* Important! DMA stream won't start if all flags are not cleared first */
	UARTDMAHandle->Instance->CMAR = (uint32_t)ble->_RxDataArr;   /* Set memory address for DMA again */
	UARTDMAHandle->Instance->CNDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
	UARTDMAHandle->Instance->CCR |= DMA_CCR_EN;            /* Start DMA transfer */
}
void bluetooth10ms(Bluetooth* ble){

	/*INCREMENTO DE INATIVIDADE-------------------*/
	(ble->msIdle<=DEF_TEMPO_MAX_S_MSG_LOW)?ble->msIdle++:0;

	/*MONITOR INATIVIDADE-------------------------*/
	if(ble->JanelaConexao>0){
		if(ble->msIdle > DEF_TEMPO_MAX_S_MSG_HIGH)	{
			BluetoothDescon(ble);
		}
	}
	else{
		__NOP();
	}

	if(ble->msIdle > DEF_TEMPO_MAX_S_MSG_LOW)	{
		BluetoothDescon(ble);
	}
}
void bluetooth1000ms(Bluetooth* ble){
	if(ble->JanelaConexao>0)
		ble->JanelaConexao--;
}
void cancelaAntecipacao(Bluetooth* ble){
	if(!ble->SistemaInit)
		BluetoothDescon(ble);
}
