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
#include "cmsis_os.h"

//lista de erros
#include "error_codes.h"


/*
 * classe é capaz de controlar maximos e minimos que forem cadastrados nela atraves de ponteiros
 *
 * */

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
#define STD_REF_MEM	 STD_REF_EEPROM
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
#define addrTETO_KI				23	// 64-bits
#define addrTETO_KD				33	// 64-bits
#define addrTETO_HIST			41	// 16-bits
#define addrTETO_LIMIT			43	// 16-bits

#define addrLASTRO_KP			45	// 64-bits
#define addrLASTRO_KI			53	// 64-bits
#define addrBUZZER				61	// 8-bits
#define addrLASTRO_KD			65	// 64-bits
#define addrLASTRO_HIST			73	// 16-bits
#define addrLASTRO_LIMIT		75	// 16-bits

#define addrREF_MEM_FLAG		77 	// 8-bits

//PAGINA PARA SALVAR ERROS pagina 10.
#define addrINIT_ERR			320
//maximo numero de erros admitidos.
#define MAX_ERRORS 10



//---MACROS---EEPROM----------------------------
#define M_EEPROM_SHIFT	Maquina.Maquina_eeprom = EEPROM_SHIFITING;
#define LIBERA_EEPROM  		HAL_GPIO_WritePin		(EEPROM_EN_GPIO_Port,	EEPROM_EN_Pin	,GPIO_PIN_RESET);\
		osDelay(5);
#define TRAVA_EEPROM		osDelay(5);\
		HAL_GPIO_WritePin		(EEPROM_EN_GPIO_Port,	EEPROM_EN_Pin	,GPIO_PIN_SET);
#define I2C_READ_MEMORY_1B(OFFSET,VARIAVEL) 	HAL_I2C_Mem_Read(&hi2c1	, EEPROM_READ_ADDR	, OFFSET, I2C_MEMADD_SIZE_16BIT,(uint8_t *) 	&VARIAVEL	, 1, 100);
#define I2C_WRITE_MEMORY_1BB(OFFSET,VARIAVEL) 	HAL_I2C_Mem_Write(&hi2c1, EEPROM_WRITE_ADDR	, OFFSET, I2C_MEMADD_SIZE_16BIT,(uint8_t *) 	&VARIAVEL	, 1, 100);
#define I2C_READ_MEMORY_2B(OFFSET ,VAR) 		HAL_I2C_Mem_Read(&hi2c1	, EEPROM_READ_ADDR	, OFFSET, I2C_MEMADD_SIZE_16BIT,(uint8_t *)  buffer		, 2, 100);\
		VAR = ( (buffer[0] << 8) | buffer[1]);

typedef struct eepromVarArr eepromVarArr;
typedef struct Eeprom Eeprom;

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
	SOFT_RESET 	= 0,
	HARD_RESET 	= 1,
}	TypeRestauracao;

typedef enum{
	stdMAX 	= 1	,
	stdDEFAULT 	,
	stdMIN 		,
}	TypeStdValue;

typedef enum {
    DATA_8BITS,
    DATA_16BITS,
    DATA_32BITS,
    DATA_FLOAT,
    DATA_DOUBLE
} DataType;

typedef struct {
	ErrorCode errors[MAX_ERRORS];
    uint8_t errorCount;
} ErrorBuffer;

/*
 * Eeprom Struct (cointainer)
 */
 struct Eeprom
{
	uint8_t flagResetavel;

	//Handle para o i2c consultar a eeprom
	I2C_HandleTypeDef *i2cHandle;

	//Handle da fila de comandos
	osMessageQId	*filaComandos;

	eepromVarArr* arrVar[EEPROM_MAX_COMP_COUNT];
	uint8_t 	  arrCount;

	//buffer de erros
	ErrorBuffer errorBuffer;

	/*---METODOS---*/
	// metodo incluir var na lista.
	void (*M_AddOnArr)(Eeprom*, eepromVarArr* );
    // download de todos os valores na memoria para todos os objetos cadastrados.
    void (*M_downloadAllVar)(Eeprom*);
    // reseta os valores na eeprom e nos ponteiros
    void (*M_resetAllVar)(Eeprom*,TypeRestauracao);
    // escreve uint8_t direto em endereco
    void (*write)(Eeprom*, uint16_t, uint16_t, uint8_t*);
    // read uint8_t direto em endereco
    void (*read)(Eeprom*,uint16_t, uint16_t, uint8_t*);

};

 /*
  * eepromVarArr Struct (objetos)
  */
 struct eepromVarArr{
 	TypeRestauracao typeReset;  // sinal que bloqueia softreset
     uint16_t _addrEprom;     	// endereco na eeprom
     DataType typeVar;        	// Tipo de dado (inteiro ou flutuante)

     // Valor atual, valor padrão, mínimo e máximo representados em forma unificada
     union {
         uint32_t intValue;   	// Para inteiros (usar quando type for DATA8BITS, DATA16BITS ou DATA32BITS)
         float    floatValue; 	// Para float (usar quando type for DATAFLOAT)
         double   doubleValue;	// Para double (usar quando type for DATADOUBLE)
     } value, defaultValue, minValue, maxValue;

     // Ponteiros para os valores gêmeos digitais
     union {
         uint8_t  *ptr8;
         uint16_t *ptr16;
         uint32_t *ptr32;
         float    *ptrFloat;
         double   *ptrDouble;
     };

     Eeprom *parentEeprom;  // Ponteiro para o 'pai'

     /*---METODOS---*/
     // valores limites e padrão fabrica
     void (*M_setStdValues8bits)(eepromVarArr* , uint8_t , uint8_t , uint8_t );
     void (*M_setStdValues16bits)(eepromVarArr* , uint16_t , uint16_t , uint16_t );
     void (*M_setStdValues32bits)(eepromVarArr* , uint32_t , uint32_t , uint32_t );
     void (*M_setStdValuesFloat)(eepromVarArr* , float, float, float );
     void (*M_setStdValuesDouble)(eepromVarArr* , double, double, double );
     // update de valor na eeprom com referencia no ponteiro.
     void (*M_update_eepromValue)(eepromVarArr*);
 };








void defaultValuesArrVar(eepromVarArr* self, TypeStdValue tipo, void *val) ;
ErrorCode eepromVarArr_deinit(eepromVarArr *self);
ErrorCode eeprom_AddVarOnArr(Eeprom* eeprom, eepromVarArr* self) ;

void addVarOnContainerEeprom(Eeprom* self, eepromVarArr* var);
ErrorCode eepromVarArr_deinit(eepromVarArr *self);

ErrorCode 	objArrEeprom_init	(eepromVarArr* self, TypeRestauracao typeReset, uint16_t addr, DataType type, void *_addrVar);
void 			 	init_objArrEeprom	(eepromVarArr* self, TypeRestauracao typeReset, uint16_t addr, DataType type, void *_addrVar);
void 				set_StdValues8bits	(eepromVarArr* self, uint8_t min, 	uint8_t def, 	uint8_t max);
void 				set_StdValues16bits	(eepromVarArr* self, uint16_t min, 	uint16_t def, 	uint16_t max);
void 				set_StdValues32bits	(eepromVarArr* self, uint32_t min, 	uint32_t def, 	uint32_t max);
void 				set_StdValuesFloat	(eepromVarArr* self, float min, 	float def, 		float max);
void 				set_StdValuesDouble (eepromVarArr* self, double min, 	double def, 	double max);

ErrorCode 	containerEeprom_init	(Eeprom *self, I2C_HandleTypeDef *i2c, osMessageQId *fila);
void 				init_containerEeprom	(Eeprom *self, I2C_HandleTypeDef *i2c, osMessageQId *fila);

ErrorCode 	eepromObjArr_update(eepromVarArr* obj);
void 				update_eepromObjArr(eepromVarArr* obj);

ErrorCode 	containerEeprom_download	(Eeprom *eeprom);
void 				download_containerEeprom	(Eeprom *eeprom);

ErrorCode 	containerEeprom_reset(Eeprom *eeprom, TypeRestauracao resetType);
void			 	reset_containerEeprom(Eeprom *eeprom, TypeRestauracao resetType);

ErrorCode 	eepromAddr_write(Eeprom *eeprom, uint16_t addr, uint16_t size, uint8_t *data);
void 				write_eepromAddr(Eeprom *eeprom, uint16_t addr, uint16_t size, uint8_t *data);

ErrorCode 	eepromAddr_read(Eeprom *eeprom, uint16_t _addrEprom, uint16_t size, uint8_t *value);
void 				read_eepromAddr(Eeprom *eeprom, uint16_t _addrEprom, uint16_t size, uint8_t *value);

// gerenciamento de erro
void 				eepromError_Handler(Eeprom *eeprom, ErrorCode erro);
void 				ErrorBuffer_init(ErrorBuffer* ebuffer);
ErrorCode 	ErrorBuffer_add(Eeprom* eeprom, uint8_t errorCode);
ErrorCode 	ErrorBuffer_read(Eeprom *eeprom);
void			 	ErrorBuffer_clear(Eeprom* eeprom);





#endif /* INC_EEPROM_H_ */
