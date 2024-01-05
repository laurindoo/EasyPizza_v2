/*
 * error_codes.h
 *
 *  Created on: Jan 4, 2024
 *      Author: lucas
 */

#ifndef INC_ERROR_CODES_H_
#define INC_ERROR_CODES_H_

#include <stdint.h> // Necessário para uint8_t

// breve descricao dos erros
typedef enum {
	NONE_ERROR 			  = 0x00,
    EEPROM_SUCCESS 		  = 0x00, 	// deu tudo certo (no reseting).
	SEQUENCIA_EEPROM	  = 40	,	// ** reinicia contagem da sequencia do typedef enum.
    EEPROM_OBJETO_NULO 			,	// recebeu um handler nulo para o objeto q deveria ter sido precriado.
    EEPROM_TIPO_ERRADO 			,	// tipo de manipulacao nao autorizado.
    EEPROM_ERRO_CADASTRO		,	// nao foi possivel cadastrar variavel na eeprom.
    EEPROM_LISTA_CHEIA			,	// lista cheia, consultar header.
    EEPROM_I2C_ERROR			,	// eeprom busy ou erro nao identificado.
    EEPROM_ERRO_ESCRITA			,	// nao foi possivel escrever na eeprom I2C.
    EEPROM_ERRO_LEITURA			,	// nao foi possivel ler na eeprom I2C.
    EEPROM_ERRO_ENDERECO_OBJ	,	// endereco do objeto nao criado.
    EEPROM_QUEBRA_ENDERECO_OBJ	,	// variavel quebrou a paginacao.
	EEPROM_TIPO_DESCONHECIDO	,	// tipo desconhecido pela classe.
	EEPROM_BUSY					,	// eeprom ocupada I2C.
	EEPROM_ERROR_FULL			,	// limite de variaveis atingido, consulte header.
	EEPROM_ERROR_EXISTS			,	//
    BLE_SUCCESS 		  = 0x00,	// deu tudo certo (no reseting).
	SEQUENCIA_BLE		  = 2	,	// ** reinicia sequencia dei typedef enum.
    BLE_OBJETO_NULO 			,	// recebeu um handler nulo para o objeto q deveria ter sido precriado.
    BLE_EXTRAPOLOU_TRY 			,	// extrapolou o numero de tentativas de obter MAC addres do hm-10.
    BLE_CRC_INCORRETO 			,	// CRC incorreto (no reseting).
    BLE_COMANDO_NAO_ENCONTRADO 	,	// comando nao encontrado na lista (no reseting).
    BLE_COMANDO_NAO_PERMITIDO 	,	// comando nao pertmitido no meu momento (no reseting).
    BLE_OBJETO_NAO_CRIADO 		,	// objeto bluetooth nao criado.
    BLE_COMANDO_NAO_CRIADO 		,	// comando nao criado.
    BLE_COMANDOCON_NAO_CRIADO 	,	// comando de conexao do HM10 nao criado (no reseting).
    BLE_NEW_DEVICE_NEGADO 		,	// novo dispositivo negado (no reseting).
    BLE_SENHA_ERRADA			,	// senha enviada por app errada (no reseting).
    BLE_HARDWARE_NOINIT			,	// hardware com erro de inicializacao.
    BLE_FILA_CHEIA				,	// fila de comandos cheia (no reseting).
    BLE_ENVIO_COMANDO			,	// erro de envio de comando (no reseting).
}ErrorCode;


#endif /* INC_ERROR_CODES_H_ */
