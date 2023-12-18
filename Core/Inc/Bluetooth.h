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

//todo subistituir envio de ALNOLADGE por função




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
#define DMA_RX_BUFFER_SIZE      24
#define BLUETOOTH_MAX_BUFF_LEN 	24
#define BLUETOOTH_TEXT_BUFF_LEN 24
#define BLUETOOTH_MAX_COMANDOS_COUNT 30

#define TAMANHO_MENSAGEM_CONECTOU 7

//---MACROS---BLUETOOTH----------------------------
#define MACRO_LE_BT_STATUS	HAL_GPIO_ReadPin 	(BLE_STATUS_GPIO_Port,	BLE_STATUS_Pin)
#define MACRO_RESET_BLE			HAL_GPIO_WritePin		(BLE_RESET_GPIO_Port,		BLE_RESET_Pin	,GPIO_PIN_RESET);\
		osDelay(200);\
		HAL_GPIO_WritePin		(BLE_RESET_GPIO_Port,		BLE_RESET_Pin	,GPIO_PIN_SET);


#define MACRO_DEFINE_INTERRUPT \
		__HAL_UART_ENABLE_IT 	(ble->UARTHandle, UART_IT_IDLE);\
		__HAL_DMA_ENABLE_IT 	(ble->UARTDMAHandle, DMA_IT_TC);	\
		HAL_UART_Receive_DMA 	(ble->UARTHandle, ble->_RxDataArr, DMA_RX_BUFFER_SIZE);	// STARTA O UART1 EM DMA MODE


//ERROS CLASSE BLUETOOTH
typedef enum {
    BLE_SUCCESS 		  = 0x00,
    BLE_OBJETO_NULO 			,
    BLE_EXTRAPOLOU_TRY 			,
    BLE_CRC_INCORRETO 			,
    BLE_COMANDO_NAO_ENCONTRADO 	,
    BLE_COMANDO_NAO_PERMITIDO 	,
    BLE_OBJETO_NAO_CRIADO 		,
    BLE_COMANDO_NAO_CRIADO 		,
    BLE_COMANDOCON_NAO_CRIADO 	,
    BLE_NEW_DEVICE_NEGADO 		,
    BLE_SENHA_ERRADA			,
    BLE_HARDWARE_NOINIT			,
} BLE_ErrorCode;

typedef enum
{   inicio = 0,
	redefineBle,
	capturaAddr,
	final,
	erro,
} sequenciaBle;

//---MAQUINA CONEXAO
typedef enum{
	RX_HARDWARE_INIT= 0x00,
	RX_DESCONECTADO,
	RX_CONECTADO,
	RX_VALIDADO
}	TypeMaquinaConexao;
/*
 * Tipos de comando
 */
typedef enum
{   ComandoBasico,
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

// Declaracao estrutura Bluetooth antecipadamente
typedef struct Bluetooth Bluetooth;
/*
 * BleComando Struct
 */
typedef struct
{
	//Variavel que vai armazenar o comando e o tipo do comando
	uint8_t _comando;
	TypeComandoBle _tipo;
}BleComando;

/*
 * Bluetooth Struct -- classe do objeto
 */
struct Bluetooth
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
	bool        SistemaEmErro       ;    //depois de conectado

	//endereco ja codificado em chave
	CRC_short chave;

	//maquina de inicializacao
	sequenciaBle sequenciaBLE;

	//comando atual
	BleComando ComandoAtual;

	//Objetos de conexao
	BleComando BLEPedeSenha,BLERecebeuSenha;

	//Variavel para receber lista de comandos
	BleComando* _BleCommArr[BLUETOOTH_MAX_COMANDOS_COUNT];
	uint8_t contComandos;

	void (*aknowladge)(Bluetooth* ble,uint8_t cmd);

};


BLE_ErrorCode bleConstrutora(Bluetooth *ble,  UART_HandleTypeDef *UARTHandle, DMA_HandleTypeDef *UARTDMAHandle, osMessageQId *_filaRX, osMessageQId *_filaTX, osMessageQId *_filaComandoInternoTX);
BLE_ErrorCode bleAddComp(Bluetooth* ble, BleComando* _blecomm, uint8_t __comando);
BLE_ErrorCode bleAddCompConexao(Bluetooth* ble, BleComando* _blecomm, uint8_t __comando);


BLE_ErrorCode readComando(Bluetooth* ble, TypeComandoBle tipo);
BLE_ErrorCode bluetoothPutFila(Bluetooth* ble, TypeComandoBle tipo);
void BLEUSART_IrqHandler(Bluetooth *ble);
BLE_ErrorCode txBleComando(Bluetooth *ble);
void BLEDMA_IrqHandler (Bluetooth *ble);
HAL_StatusTypeDef bluetoothEnviaComando(Bluetooth *ble, unsigned char _out[], int size);
void comandHM10(Bluetooth *ble, char _out[], uint16_t delay);
BLE_ErrorCode iniciaBleHm10(Bluetooth* ble);
void redefineBluetooth(Bluetooth* ble);
BLE_ErrorCode bluetoothErroCRC(Bluetooth* ble);
void bluetoothDescon(Bluetooth* ble);
void bluetooth10ms(Bluetooth* ble);
void bluetooth1000ms(Bluetooth* ble);
void sendAknowladge(Bluetooth* ble,uint8_t Cmd);

#endif /* INC_BLUETOOTH_H_ */
