/*
 * Bluetooth.h
 *
 *  Created on: Jul 24, 2023
 *      Author: lucas
 */
/*
 * configurar:
 * 		-task de manipulação de dados
 * 		-callback de DMA no stm32f1xx_it.c
 * 		-callback 10ms do contador
 * 		-callback 1000ms do contador
 *
 *
 *
 *
 *
 * */
#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

//Include HAL Library from main header file :/
#include "main.h"

//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

//---DEFINICOES---TEMPOS DE CONEXAO
#define DEF_TEMPO_MAX_S_MSG_LOW			200	//x*10ms
#define DEF_TEMPO_MAX_S_MSG_HIGH		200	//x*10ms

#define BLUETOOTH_MAX_BUFF_LEN 32
#define BLUETOOTH_TEXT_BUFF_LEN 32
#define BLUETOOTH_MAX_COMANDOS_COUNT 45

//---MACROS---BLUETOOTH----------------------------
#define MACRO_LE_BT_STATUS	HAL_GPIO_ReadPin 	(BLE_STATUS_GPIO_Port,	BLE_STATUS_Pin)
#define MACRO_RESET_BLE			HAL_GPIO_WritePin		(BLE_RESET_GPIO_Port,		BLE_RESET_Pin	,GPIO_PIN_RESET);\
		HAL_Delay(200);\
		HAL_GPIO_WritePin		(BLE_RESET_GPIO_Port,		BLE_RESET_Pin	,GPIO_PIN_SET);
#define MACRO_ENVIA_AKNOLADGE 		bluetooth.TXBuffer[0] = 0x01;\
		bluetooth.TXBuffer[1] = 0xFF;\
		bluetooth.TXBuffer[2] = 0xFF;\
		Envia_bytes_UART(bluetooth.TXBuffer,3);

#define MACRO_ENVIA_AKNOLADGE_(val) 		bluetooth.TXBuffer[0] = 0x01;\
		bluetooth.TXBuffer[1] = 0xFF;\
		bluetooth.TXBuffer[2] = val;\
		Envia_bytes_UART(bluetooth.TXBuffer, 3);



//---MAQUINA CONEXAO
typedef enum{
	RX_DESCONECTADO = 0x00,
	RX_CONECTADO,
	RX_VALIDADO
}	TypeMaquinaConexao;

/*
 * Estrutura CRC calculation
 */
typedef union CRC_short
{
	struct
	{
		unsigned char hi;
		unsigned char lo;
	}byte;
	short hilo;
}CRC_short;

/*
 * Tipos de comando
 */
typedef enum
{   ComandoCritico,
	ComandoBasico,
	ComandoConexao,
} TypeComandoBle;

//---Comandos Ble
typedef enum
{
	RX_ATUALIZA_HORA 		= 0x03,
	RX_RESTAURA 			= 0x10,
	RX_SOLICITA_REALTIME 	= 0x15,
	RX_SOLICITA_SINCRONIA 	= 0x17,
	RX_SP_TEMP_TETO			= 0x21,
	RX_SP_TEMP_LASTRO		= 0x22,
	RX_SP_TEMPO				= 0x23,
	RX_TOGGLE_TEMPO			= 0x24,
	RX_RECEITA			 	= 0x25,
	RX_LIMITE_TEMPERATURA 	= 0x26,
	RX_LIGA_LAMPADA		 	= 0x27,
	RX_DESLIGA_LAMPADA	 	= 0x28,
	RX_CANCELA_PROCESSO	 	= 0x29,
	RX_LIMITE_LAMPADA	 	= 0x30,
	RX_RECEBEU_SENHA		= 0x40,
	RX_PEDE_SENHA			= 0x42,
} ComandosBleRX;

//---Comandos Ble
typedef enum
{
	TX_ULTIMO_CODIGO 		= 0x01,
	TX_REALTIME_DATA 		= 0x16,
	TX_REALTIME_DATA2		= 0x17,
	TX_SINCRONIA 			= 0x18,
	TX_DATA_MEIA 			= 0x21,
	TX_DATA_CHEIA 			= 0x22,
	TX_COMANDO_NEGADO 		= 0x89,
	TX_DESCONECTA			,
	TX_CHAVE				,
	TX_CHAVE_ERRO			,
	TX_RESULTADO_CHAVE_OK	,
	TX_RESULTADO_CHAVE_ERRO	,
} ComandosBleTX;

/*
 * BleComando Struct
 */
typedef struct
{
	//Variavel que vai armazenar o comando e o tipo do comando
	uint8_t _comando;
	TypeComandoBle _tipo;
	//Variable for storing object name
	char *objname;

}BleComando;

/*
 * Bluetooth Struct
 */
typedef struct
{
	//Handles globais
	UART_HandleTypeDef 	*UARTHandle;
	DMA_HandleTypeDef 	*UARTDMAHandle;

	//bufer's
	unsigned char	TXBuffer	[BLUETOOTH_MAX_BUFF_LEN]		;	//BUFFER AUXILIAR DO "DMA_CIRCULAR.C" de uint8 -> uchar

	//Handle da fila de comandos
	osMessageQId	*filaComandosRX;//Handle da fila de comandos
	osMessageQId	*filaComandosTX;

	//Variables for parsing the received data
	uint8_t _RxDataArr[BLUETOOTH_MAX_BUFF_LEN], _RxData,RxSize;

	//maquina de estados para conexap
	TypeMaquinaConexao MaquinaConexao;

	//contadores
	uint8_t 	JanelaConexao		;	//contador decrescente de janela de conexao
	uint8_t     msDesconectado    	;    //Contador crescente de tempo de desconectado em milisegundos
	uint8_t     msIdle    			;    //Contador crescente sem troca mensagens em milisegundos
		//flags
	bool 		StatusSenha			;	//FLAG QUE ARMAZENA VALIDACAO DE CHAVE DE ACESSO
	bool 		StatusConexao		;	//depois de conectado e sincronizado
	bool        SistemaEmErro       ;    //depois de conectado

	//endereco ja codificado em chave
	uint16_t chave;

	//variaveis de comparacao de texto
	char	StringRecebida[BLUETOOTH_MAX_BUFF_LEN];
	long 	PontoExato;
	char 	*ss;
	char 	*tt;

	//Variavel para receber lista de comandos
	BleComando* _BleCommArr[BLUETOOTH_MAX_COMANDOS_COUNT];
	uint8_t _BleCommCount;

}Bluetooth;

uint8_t BluetoothInit(Bluetooth *ble, UART_HandleTypeDef *bluetoothUARTHandle, DMA_HandleTypeDef *bluetoothUARTDMAHandle, osMessageQId *filaRX, osMessageQId *filaTX);
uint8_t BluetoothAddComp(Bluetooth* ble, BleComando* _blecomm, char* objectname, uint8_t __comando, TypeComandoBle __tipo);
void BluetoothPutFila(Bluetooth* ble);
void BLEUSART_IrqHandler(Bluetooth *ble);
void BLEDMA_IrqHandler (Bluetooth *ble);
void BluetoothEnviaComando(unsigned char _out[], int size);
void Envia_bytes_UART(unsigned char _out[], uint8_t size);
void Envia_texto_UART(char _out[], uint16_t delay);
unsigned short CRC16 (unsigned char *puchMsg, unsigned short usDataLen);
void Inicia_HM10(Bluetooth* ble);
void iniciaBleHm10(Bluetooth* ble);
void redefineBluetooth(Bluetooth* ble);
void BluetoothErroCRC(void);
void BluetoothDescon(Bluetooth* ble);
void bluetooth10ms(Bluetooth* ble);
void bluetooth1000ms(Bluetooth* ble);

#endif /* INC_BLUETOOTH_H_ */
