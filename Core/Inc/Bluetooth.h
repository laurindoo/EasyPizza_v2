/*
 * Bluetooth.h
 *
 *  Created on: Jul 24, 2023
 *      Author: lucas
 */
/*
 * configurar:
 * 		-definir nome 				-> Bluetooth.h
 * 		-criar task de manipulação de dados ->TaskBluetooth.c
 * 		-criar fila de rx e tx 		-> confgiRtos
 * 		-callback de DMA 			-> stm32f1xx_it.c
 * 		-callback 10ms do contador 	-> main.c
 * 		-callback 1000ms do contador-> main.c
 *
 *
 * */
#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

//Include HAL Library from main header file :/
#include "main.h"

//Include libraries //todo revisar todas as bibliotecas
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"



#include "errorcodes.h"

#define BLE_DEVICE_NAME "EasyPizza" // Substitua por um nome apropriado
#define COMANDO_BUFFER_SIZE (50) // Escolha um tamanho que seja suficiente

//---DEFINICOES---TEMPOS DE CONEXAO
#define DEF_TEMPO_MAX_S_MSG_LOW			120	//x*10ms
#define DEF_TEMPO_MAX_S_MSG_HIGH		300	//x*10ms


#define CHEGOU_ADDR_BLE 0xbf
#define DMA_RX_BUFFER_SIZE      64
#define BLUETOOTH_MAX_BUFF_LEN 	32
#define BLUETOOTH_TEXT_BUFF_LEN 32
#define BLUETOOTH_MAX_COMANDOS_COUNT 45

//---MACROS---BLUETOOTH----------------------------
#define MACRO_LE_BT_STATUS	HAL_GPIO_ReadPin 	(BLE_STATUS_GPIO_Port,	BLE_STATUS_Pin)
#define MACRO_RESET_BLE			HAL_GPIO_WritePin		(BLE_RESET_GPIO_Port,		BLE_RESET_Pin	,GPIO_PIN_RESET);\
		osDelay(200);\
		HAL_GPIO_WritePin		(BLE_RESET_GPIO_Port,		BLE_RESET_Pin	,GPIO_PIN_SET);
#define MACRO_ENVIA_AKNOLADGE 		bluetooth.TXBuffer[0] = 0x01;\
		bluetooth.TXBuffer[1] = 0xFF;\
		bluetooth.TXBuffer[2] = 0xFF;\
		Envia_bytes_UART(bluetooth.TXBuffer,3);

#define MACRO_ENVIA_AKNOLADGE_(val) 		bluetooth.TXBuffer[0] = 0x01;\
		bluetooth.TXBuffer[1] = 0xFF;\
		bluetooth.TXBuffer[2] = val;\
		Envia_bytes_UART(&bluetooth,bluetooth.TXBuffer, 3);
#define MACRO_DEFINE_INTERRUPT \
		__HAL_UART_ENABLE_IT 	(ble->UARTHandle, UART_IT_IDLE);\
		__HAL_DMA_ENABLE_IT 	(ble->UARTDMAHandle, DMA_IT_TC);	\
		HAL_UART_Receive_DMA 	(ble->UARTHandle, ble->_RxDataArr, DMA_RX_BUFFER_SIZE);	// STARTA O UART1 EM DMA MODE

typedef enum
{   inicio = 0,
	verificaNome=1,
	redefineBle=2,
	capturaAddr=3,
	final=4,
	erro=5,
} sequenciaBle;

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
	RX_RECEBEU_SENHA		= 0x40,
	RX_PEDE_SENHA			= 0x42,
} ConexaoBleRX;

//---Comandos Ble
typedef enum
{
	COMANDO_ULTIMO_CODIGO 		= 0x01,
	COMANDO_COMANDO_NEGADO 		= 0x89,
	COMANDO_SOLICITACAO_SENHA	,
	COMANDO_AVALIACAO_CHAVE		,
} ConexaoBleTX;


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
 * Bluetooth Struct -- classe do objeto
 */
typedef struct
{
	//Handles globais
	UART_HandleTypeDef 	*UARTHandle;
	DMA_HandleTypeDef 	*UARTDMAHandle;

	//bufer's
	unsigned char	TXBuffer	[BLUETOOTH_MAX_BUFF_LEN]		;	//BUFFER AUXILIAR DO "DMA_CIRCULAR.C" de uint8 -> uchar

	//Handle da filas
	osMessageQId	*filaComandosRX;//Handle da fila de comandos
	osMessageQId	*filaComandosTX;
	osMessageQId	*filaComandoInternoTX;

	//Variables for parsing the received data
	uint8_t _RxDataArr[BLUETOOTH_MAX_BUFF_LEN], _RxData,RxSize;

	//maquina de estados para conexap
	TypeMaquinaConexao MaquinaConexao;

	//contadores
	uint8_t 	JanelaConexao		;	//contador decrescente de janela de conexao
	uint8_t     msDesconectado    	;    //Contador crescente de tempo de desconectado em milisegundos
	uint16_t     msIdle    			;    //Contador crescente sem troca mensagens em milisegundos
	//flags
	bool 		StatusSenha			;	//FLAG QUE ARMAZENA VALIDACAO DE CHAVE DE ACESSO
	bool 		StatusConexao		;	//depois de conectado e sincronizado
	bool 		SistemaInit			;	//indica que sistema ja esta inicializado
	bool        SistemaEmErro       ;    //depois de conectado

	//endereco ja codificado em chave
	uint16_t chave;

	//variaveis de comparacao de texto
	char	StringRecebida[BLUETOOTH_MAX_BUFF_LEN];
	long 	PontoExato;
	char 	*ss;
	char 	*tt;

	//maquina de inicializacao
	sequenciaBle sequenciaBLE;

	//Objetos de conexao
	BleComando BLEPedeSenha,BLERecebeuSenha;

	//Variavel para receber lista de comandos
	BleComando* _BleCommArr[BLUETOOTH_MAX_COMANDOS_COUNT];
	uint8_t _BleCommCount;

}Bluetooth;


uint8_t bleConstrutora(Bluetooth *ble,  UART_HandleTypeDef *UARTHandle, DMA_HandleTypeDef *UARTDMAHandle, osMessageQId *_filaRX, osMessageQId *_filaTX, osMessageQId *_filaComandoInternoTX);
uint8_t bleAddComp(Bluetooth* ble, uint8_t _comando, TypeComandoBle _tipo);



uint8_t BluetoothInit(Bluetooth *ble, UART_HandleTypeDef *bluetoothUARTHandle, DMA_HandleTypeDef *bluetoothUARTDMAHandle, osMessageQId *filaRX, osMessageQId *filaTX, osMessageQId *filaComandoInternoTX);
uint8_t BluetoothAddComp(Bluetooth* ble, BleComando* _blecomm, char* objectname, uint8_t __comando, TypeComandoBle __tipo);
void BluetoothPutFila(Bluetooth* ble);
void BLEUSART_IrqHandler(Bluetooth *ble);
void txBleComando(Bluetooth *ble);
void BLEDMA_IrqHandler (Bluetooth *ble);
void BluetoothEnviaComando(Bluetooth *ble, unsigned char _out[], int size);
void Envia_bytes_UART(Bluetooth *ble, unsigned char _out[], uint8_t size);
void Envia_texto_UART(Bluetooth *ble, char _out[], uint16_t delay);
unsigned short CRC16 (unsigned char *puchMsg, unsigned short usDataLen);
uint8_t iniciaBleHm10(Bluetooth* ble);
void redefineBluetooth(Bluetooth* ble);
void BluetoothErroCRC(Bluetooth* ble);
void BluetoothDescon(Bluetooth* ble);
void bluetooth10ms(Bluetooth* ble);
void bluetooth1000ms(Bluetooth* ble);
void cancelaAntecipacao(Bluetooth* ble);

#endif /* INC_BLUETOOTH_H_ */
