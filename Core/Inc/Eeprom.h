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
#define STD_REF_MEM	0x25
#define PAGE_SIZE 32

#define EEPROM_MAX_COMP_COUNT 30

//ENDEREÇO EEPROM  1010 E2 E1 E0 RW
#define EEPROM_READ_ADDR 	0b10100011
#define EEPROM_WRITE_ADDR 	0b10100010

//VARIAVEIS DE CONFIGURACAO    pagina 1
#define addrTEMPO_LUZ			1	// 16-bits
#define addrHORIMETRO			3	// 16-bits
#define addrMINUTIMETRO			5	// 8-bits
#define addrINST_DIA			6	// 8-bits
#define addrINST_MES			7	// 8-bits
#define addrINST_ANO			8	// 8-bits
#define addrTOTAL_GERAL			9	// 16-bits
#define addrCONT_MAX_TETO		11	// 16-bits
#define addrCONT_MAX_LASTRO		13	// 16-bits

#define addrTETO_KP				15	// 64-bits
#define addrTETO_KI				22	// 64-bits
#define addrTETO_KD				33	// 64-bits
#define addrTETO_HIST			41	// 16-bits
#define addrTETO_LIMIT			43	// 16-bits

#define addrLASTRO_KP			45	// 64-bits
#define addrLASTRO_KI			53	// 64-bits
#define addrBUZZER				61	// 8-bits
#define addrLASTRO_KD			65	// 64-bits
#define addrLASTRO_HIST			73	// 16-bits
#define addrLASTRO_LIMIT		75	// 16-bits

#define addrREF_MEM_FLAG		77 // 8-bits



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
}	TypeData;

typedef enum{
	DATAFLOAT 	= 32,
	DATADOUBLE 	= 64,
}	TypeDataFloating;

typedef enum{
	softReset 	= 0,
	hardReset 	= 1,
}	TypeRestauracao;

/*
 * EepromVariaveis Struct
 */
typedef struct
{
	//sinal que bloqueia softreset
	bool flagResetavel;

	//Variable for storing object name
	char *objname;

	//endereco na eeprom
	uint16_t _addrEprom;

	uint32_t valor;
	uint32_t defaultValue; //valor de default
	uint32_t minValue;
	uint32_t maxValue;

	//tamanho_tipo
	TypeData _sizeType;

	//ponteiros digitalTwins
	uint8_t  *ptr8;
	uint16_t *ptr16;
	uint32_t *ptr32;

}EepromVariaveis;
typedef struct
{
	//sinal que bloqueia softreset
	bool flagResetavel;

	//Variable for storing object name
	char *objname;

	//endereco na eeprom
	uint16_t _addrEprom;

	double  valorDouble;
	float   valorFloat;
	double  defaultValue; //valor de default
	double 	minValue;
	double  maxValue;

	//tamanho_tipo
	TypeDataFloating _sizeType;

	//ponteiros digitalTwins
	float 	 *ptrFloat;
	double 	 *ptrDouble;

}EepromVarFloating;


/*
 * Eeprom Struct
 */
typedef struct
{
	//flag de referencia
	EepromVariaveis RefFlag;

	//Handle para o i2c consultar a eeprom
	I2C_HandleTypeDef *i2cHandle;

	//Handle da fila de comandos
	osMessageQId	*filaComandos;

	//Variables for component list
	EepromVariaveis* _EepromVarArr[EEPROM_MAX_COMP_COUNT];
	EepromVarFloating* _EepromVarFloatingArr[EEPROM_MAX_COMP_COUNT];

	uint8_t _EepromVarCount;
	uint8_t _EepromVarFloatingCount;


	//todo:devo importar a fila de eeprom
	//todo estudar como importar também a estrutura de comandos possiveis para a fila importada a cima

}Eeprom;

//prototipos das funcoes da eeprom
uint8_t EepromInit	(Eeprom *eeprom, I2C_HandleTypeDef *i2c, osMessageQId *fila);
void EepromUpdateMes(Eeprom *eeprom, uint16_t addMes, uint16_t addItem, uint32_t valor, TypeData _sizeType);
uint8_t EepromAddVar(Eeprom *eeprom			, bool resetavel, EepromVariaveis* _var, char* _name, uint16_t addr, TypeData tipo, uint32_t minimo, uint32_t padrao,uint32_t maximo, void *_addrVar);
uint8_t EepromAddVarFloating(Eeprom *eeprom	, bool resetavel, EepromVarFloating* _eepromvar, char* _name,uint16_t addr,TypeDataFloating tipo,double minimo,double padrao,double maximo, void *_addrVar);
bool EepromSetVar	(Eeprom *eeprom, EepromVariaveis *_var, uint32_t valor);
bool EepromSetVarFloating	(Eeprom *eeprom, EepromVarFloating *_var, double valor);
void EepromDownloadValores	(Eeprom *eeprom);
void RestauraEeprom			(Eeprom *eeprom ,TypeRestauracao tipo);
void EepromReadVal			(Eeprom *eeprom, uint16_t addr, uint8_t *_var, uint16_t size);
void Write_1_byte			(Eeprom *eeprom, uint16_t addr, uint8_t * _ptr);



#endif /* INC_EEPROM_H_ */
