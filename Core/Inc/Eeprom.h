/*
 * Eeprom.h
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

//Include libraries
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"





/*
--------------->>M24C64
 	 	 	 -total de memoria 64KBit = 8KBytes
 	 	 	 -Cada página do M24C64 possui 32 bytes de dados (0x20).

 	 	 	 datas serao armazenadas cada uma em uma página, sendo apartir da página 10

 	 	 	 	 **eeprom shift será feito por página, trazendo todas as paginas(10 até a 24) da PAGINA*N -> PAGINA*(N-1)

//TAMANHO DA PAGINA
#define PAGE_SIZE 0d32

*/
//TAMANHO DA PAGINA
#define PAGE_SIZE 32

#define EEPROM_MAX_COMP_COUNT 45

//ENDEREÇO EEPROM  1010 E2 E1 E0 RW
#define EEPROM_READ_ADDR 	0b10100011
#define EEPROM_WRITE_ADDR 	0b10100010

//VARIAVEIS DE CONFIGURACAO    pagina 1
#define addrTEMPO_LUZ			0x0004	// 8-bits
#define addrHORIMETRO			0x0005	// 8-bits
#define addrMINUTIMETRO			0x0006	// 8-bits
#define addrINST_DIA			0x000c	// 8-bits
#define addrINST_MES			0x000d	// 8-bits
#define addrINST_ANO			0x000e	// 8-bits
#define addrTOTAL_GERAL			0x000f	// 16-bits
#define addrLIMITE_TEMP			0x0011	// 16-bits
#define addrCONT_MAX_TETO		0x0013	// 16-bits
#define addrCONT_MAX_LASTRO		0x0015	// 16-bits


//---MACROS---EEPROM----------------------------
#define M_EEPROM_SHIFT	Maquina.Maquina_eeprom = EEPROM_SHIFITING;
#define LIBERA_EEPROM  		HAL_GPIO_WritePin		(EEPROM_EN_GPIO_Port,	EEPROM_EN_Pin	,GPIO_PIN_RESET);\
		HAL_Delay(5);
#define TRAVA_EEPROM		HAL_Delay(5);\
	HAL_GPIO_WritePin		(EEPROM_EN_GPIO_Port,	EEPROM_EN_Pin	,GPIO_PIN_SET);
#define I2C_READ_MEMORY_1B(OFFSET,VARIAVEL) 	HAL_I2C_Mem_Read(&hi2c1	, EEPROM_READ_ADDR	, OFFSET, I2C_MEMADD_SIZE_16BIT,(uint8_t *) 	&VARIAVEL	, 1, 100);
#define I2C_WRITE_MEMORY_1BB(OFFSET,VARIAVEL) 	HAL_I2C_Mem_Write(&hi2c1, EEPROM_WRITE_ADDR	, OFFSET, I2C_MEMADD_SIZE_16BIT,(uint8_t *) 	&VARIAVEL	, 1, 100);
#define I2C_READ_MEMORY_2B(OFFSET ,VAR) 		HAL_I2C_Mem_Read(&hi2c1	, EEPROM_READ_ADDR	, OFFSET, I2C_MEMADD_SIZE_16BIT,(uint8_t *)  buffer		, 2, 100);\
	VAR = ( (buffer[0] << 8) | buffer[1]);


//---tamanhos
typedef enum{
	DATA8BITS  = 1,
	DATA16BITS = 2,
	DATA32BITS = 4,
}	TypeTamData;

/*
 * EepromVariaveis Struct
 */
typedef struct
{
	//Variables Proprio endereco e endereco na eeprom
	uint16_t _addrEprom;

	uint32_t valor;
	uint32_t defaultValue; //valor de default
	uint32_t minValue;
	uint32_t maxValue;
	//Variable for storing object name
	char *objname;

	TypeTamData _sizeType;

	uint8_t *ptr8;
	uint16_t *ptr16;
	uint32_t *ptr32;

}EepromVariaveis;


/*
 * Eeprom Struct
 */
typedef struct
{
	//Handle para o i2c consultar a eeprom
	I2C_HandleTypeDef *i2cHandle;

	//Handle da fila de comandos
	osMessageQId	*filaComandos;

	//Variables for component list
	EepromVariaveis* _EepromVarArr[EEPROM_MAX_COMP_COUNT];
	uint8_t _EepromVarCount;


	//todo:devo importar a fila de eeprom
	//todo estudar como importar também a estrutura de comandos possiveis para a fila importada a cima

}Eeprom;

//prototipos das funcoes da eeprom
uint8_t EepromInit(Eeprom *eeprom, I2C_HandleTypeDef *i2c, osMessageQId *fila);
void EepromUpdateMes(Eeprom *eeprom, uint16_t __addMes, uint16_t __addItem, uint32_t valor, TypeTamData _sizeType);
uint8_t EepromAddVar(Eeprom *eeprom, EepromVariaveis* _eepromvar, char* objectname, uint8_t __addreeprom,TypeTamData tamanho,uint32_t minimo,uint32_t padrao,uint32_t maximo,uint32_t *addrVar);
__IO bool EepromSetVar(Eeprom *eeprom, EepromVariaveis *eepromvar,uint32_t valor);
void EepromDownloadValores(Eeprom *eeprom);
void RestauraPadraoTudo(Eeprom *eeprom);
void EepromReadVal(Eeprom *eeprom, uint16_t addr, uint8_t *var, uint16_t size);
void Write_1_byte(Eeprom *eeprom, uint16_t addr, uint8_t * ptr);



#endif /* INC_EEPROM_H_ */
