/*
 * Eeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#include "Eeprom.h"
#include "Conversoes.h"

//variaveis globais da EEprom.
#ifndef INC_EEPROM_C_
#define INC_EEPROM_C_


ErrorCode 	containerEeprom_init(Eeprom *self, I2C_HandleTypeDef *i2c, osMessageQId *fila){
	if (self == NULL || i2c == NULL || fila == NULL)
		return EEPROM_OBJETO_NULO;

	// limpe a memória da estrutura.
	memset(self, 0, sizeof(*self));

	// popula objeto.
	self->i2cHandle 			= i2c;	// handler de I2C.
	self->filaComandos 			= fila; // fila de salvamento.

	// inicializa o error buffer.
	ErrorBuffer_init(&self->errorBuffer);

	// le os erros salvos na eeprom.
	ErrorBuffer_read(self);

	/*---METODOS---*/
	// metodo incluir var na lista.
	self->M_AddOnArr = addVarOnContainerEeprom;
	// download de todos os valores na memoria para todos os objetos cadastrados.
	self->M_downloadAllVar = download_containerEeprom;
	// reseta os valores na eeprom e nos ponteiros
	self->M_resetAllVar = reset_containerEeprom;
	// ecreve 8 bits em endereco
	self->write = write_eepromAddr;
	// le 8 bits em endereco
	self->read = read_eepromAddr;

	return EEPROM_SUCCESS;
}
void 				init_containerEeprom(Eeprom *self, I2C_HandleTypeDef *i2c, osMessageQId *fila) {
	//funcao que implementa selfContainer_init
	ErrorCode errCode = containerEeprom_init(self, i2c, fila);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(self, errCode);
	}
}

ErrorCode 	objArrEeprom_init(eepromVarArr* 	self, TypeRestauracao typeReset, uint16_t addr, DataType type, void *_addrVar){

	if (self == NULL || _addrVar == NULL)
		return EEPROM_OBJETO_NULO;
	if (addr == 0)
		return EEPROM_ERRO_ENDERECO_OBJ;

	// Limpe a memória da estrutura.
	memset(self, 0, sizeof(*self));

	// popula objeto.
	self->typeVar 		= type;
	self->typeReset 	= typeReset;
	self->_addrEprom 	= addr;

	switch (self->typeVar) {
	case DATA_8BITS:	self->ptr8 		= (uint8_t *)_addrVar;	break;
	case DATA_16BITS:	self->ptr16 	= (uint16_t *)_addrVar;	break;
	case DATA_32BITS:	self->ptr32 	= (uint32_t *)_addrVar;	break;
	case DATA_FLOAT:	self->ptrFloat 	= (float *)_addrVar;	break;
	case DATA_DOUBLE:	self->ptrDouble	= (double *)_addrVar;	break;
	default:
		return EEPROM_TIPO_ERRADO;
	}

	/*---cadastro de metodos---*/
	// metodos valores standart e limites.
	self->M_setStdValues8bits		= set_StdValues8bits;
	self->M_setStdValues16bits		= set_StdValues16bits;
	self->M_setStdValues32bits		= set_StdValues32bits;
	self->M_setStdValuesFloat		= set_StdValuesFloat;
	self->M_setStdValuesDouble		= set_StdValuesDouble;
	// metodo de update de eeprom vindo do ponteiro.
	self->M_update_eepromValue 		= update_eepromObjArr;

	return EEPROM_SUCCESS;
}
void 				init_objArrEeprom(eepromVarArr* 	self, TypeRestauracao typeReset, uint16_t addr, DataType type, void *_addrVar) {
	ErrorCode errCode = objArrEeprom_init(self, typeReset, addr, type,_addrVar);
	if (errCode != EEPROM_SUCCESS)
		eepromError_Handler(self->parentEeprom, errCode);
}
void 				set_StdValues8bits(eepromVarArr* 	self, uint8_t 	min, uint8_t 	def, uint8_t 	max) {
	self->minValue.intValue 	= min;
	self->defaultValue.intValue = def;
	self->maxValue.intValue 	= max;
}
void 				set_StdValues16bits(eepromVarArr* 	self, uint16_t 	min, uint16_t 	def, uint16_t 	max) {
	self->minValue.intValue 	= min;
	self->defaultValue.intValue = def;
	self->maxValue.intValue 	= max;
}
void 				set_StdValues32bits(eepromVarArr* 	self, uint32_t 	min, uint32_t 	def, uint32_t 	max) {
	self->minValue.intValue 	= min;
	self->defaultValue.intValue = def;
	self->maxValue.intValue 	= max;
}
void 				set_StdValuesFloat(eepromVarArr* 	self, float 	min, float 		def, float 		max) {
	self->minValue.floatValue 		= min;
	self->defaultValue.floatValue 	= def;
	self->maxValue.floatValue 		= max;
}
void 				set_StdValuesDouble(eepromVarArr* 	self, double 	min, double 	def, double 	max) {
	self->minValue.doubleValue 		= min;
	self->defaultValue.doubleValue 	= def;
	self->maxValue.doubleValue 		= max;
}
ErrorCode 	eepromVarArr_deinit(eepromVarArr*	self){
	__NOP();
	return EEPROM_SUCCESS;
}

ErrorCode 	eeprom_AddVarOnArr(Eeprom* eeprom, eepromVarArr* self) {

	if (eeprom->arrCount >= EEPROM_MAX_COMP_COUNT)
		return EEPROM_LISTA_CHEIA;
	if (self->_addrEprom == 0)
		return EEPROM_ERRO_ENDERECO_OBJ;

	// Calcula o início da próxima página
	uint16_t inicioProximaPagina = ((self->_addrEprom / PAGE_SIZE) + 1) * PAGE_SIZE;

	// Verifica se o endereço inicial + tamanho da variável ultrapassa o início da próxima página
	uint16_t fimVar;

	switch (self->typeVar) {
	case DATA_8BITS:
		fimVar = self->_addrEprom + sizeof(uint8_t);
		break;
	case DATA_16BITS:
		fimVar = self->_addrEprom + sizeof(uint16_t);
		break;
	case DATA_32BITS:
		fimVar = self->_addrEprom + sizeof(uint32_t);
		break;
	case DATA_FLOAT:
		fimVar = self->_addrEprom + sizeof(float);
		break;
	case DATA_DOUBLE:
		fimVar = self->_addrEprom + sizeof(double);
		break;
	default:
		return EEPROM_TIPO_DESCONHECIDO;
	}

	// Se a próxima varável começa no início de uma nova página, então essa variável cruzou a fronteira de uma página
	if (fimVar > inicioProximaPagina)
		return EEPROM_QUEBRA_ENDERECO_OBJ;

	// Configuração do ponteiro parent
	self->parentEeprom = eeprom;
	// Adicionar a variável ao próximo slot livre
	eeprom->arrVar[eeprom->arrCount] = self;
	eeprom->arrCount++;

	return EEPROM_SUCCESS;
}
void 				addVarOnContainerEeprom(Eeprom* self, eepromVarArr* var){
	ErrorCode errCode = eeprom_AddVarOnArr(self, var);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(self, errCode);
	}
}

ErrorCode 	eepromObjArr_update(eepromVarArr* obj) {

	HAL_StatusTypeDef result;
	uint8_t tries = 0, MAX_TRIES = 5;
	uint8_t buffer[sizeof(double)] __attribute__((aligned(sizeof(double)))); // Buffer pode conter até 64 bits (para DATADOUBLE)

	LIBERA_EEPROM

	//verifica disponibilidade da eeprom
	while (tries++ < MAX_TRIES) {
		result = HAL_I2C_IsDeviceReady(obj->parentEeprom->i2cHandle,EEPROM_WRITE_ADDR, 50, HAL_MAX_DELAY);
		if (result == HAL_OK)	break;
		if (tries == MAX_TRIES)	return EEPROM_I2C_ERROR;
	}

	switch (obj->typeVar) {
	case DATA_8BITS:
		obj->value.intValue 	= *obj->ptr8;
		*((uint8_t*)&buffer) 	= *obj->ptr8;
		// limita as tentativas de escrita.
		while (tries++ < MAX_TRIES) {
			result = HAL_I2C_Mem_Write(obj->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, obj->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 1, 200);
			if (result == HAL_OK)	break;
			if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
		}
		tries=0;
		break;
	case DATA_16BITS:
		obj->value.intValue 	= *obj->ptr16;
		*((uint16_t*)&buffer) 	= *obj->ptr16;
		// limita as tentativas de escrita.
		while (tries++ < MAX_TRIES) {
			result = HAL_I2C_Mem_Write(obj->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, obj->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 2, 200);
			if (result == HAL_OK)	break;
			if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
		}
		tries=0;
		break;
	case DATA_32BITS:
		obj->value.intValue 	= *obj->ptr32;
		*((uint32_t*)&buffer) 	= *obj->ptr32;
		// limita as tentativas de escrita.
		while (tries++ < MAX_TRIES) {
			result = HAL_I2C_Mem_Write(obj->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, obj->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 4, 200);
			if (result == HAL_OK)	break;
			if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
		}
		tries=0;
		break;
	case DATA_FLOAT:
		obj->value.floatValue 	= *obj->ptrFloat;
		*((float*)&buffer) 		= *obj->ptrFloat;
		// limita as tentativas de escrita.
		while (tries++ < MAX_TRIES) {
			result = HAL_I2C_Mem_Write(obj->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, obj->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 4, 200);
			if (result == HAL_OK)	break;
			if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
		}
		tries=0;
		break;
	case DATA_DOUBLE:
		obj->value.doubleValue 	= *obj->ptrDouble;
		*((double*)&buffer) 	= (double)obj->value.doubleValue;
		// limita as tentativas de escrita.
		while (tries++ < MAX_TRIES) {
			result = HAL_I2C_Mem_Write(obj->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, obj->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 8, 200);
			if (result == HAL_OK)	break;
			if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
		}
		tries=0;
		break;
	}
	osDelay(40);
	TRAVA_EEPROM
	return EEPROM_SUCCESS;
}
void 				update_eepromObjArr(eepromVarArr* obj) {
	ErrorCode errCode = eepromObjArr_update(obj);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(obj->parentEeprom, errCode);
	}
}

ErrorCode 	containerEeprom_download	(Eeprom *eeprom){
	HAL_StatusTypeDef 	result;
	uint8_t tries = 0, MAX_TRIES = 5;
	uint8_t 		firstStart,	buffer[sizeof(double)] __attribute__((aligned(sizeof(double)))); // Buffer pode conter até 64 bits (para DATADOUBLE)
	eepromVarArr		*var;

	//verifica disponibilidade da eeprom
	while (tries++ < MAX_TRIES) {
		result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle,EEPROM_WRITE_ADDR, 50, HAL_MAX_DELAY);
		if (result == HAL_OK)	break;
		if (tries == MAX_TRIES)	return EEPROM_I2C_ERROR;
	}
	tries=0;

	// verifica flag primeiro start.
	eeprom->read(eeprom, addrREF_MEM_FLAG, sizeof(uint8_t), &firstStart);
	if (firstStart != STD_REF_MEM) {
		// salvar na eeprom o primeiro start.
		eeprom->M_resetAllVar(eeprom, HARD_RESET);
		firstStart = STD_REF_MEM;
		eeprom->write(eeprom, addrREF_MEM_FLAG, sizeof(uint8_t), &firstStart);
	}

	// Varredura por todos os elementos
	for (uint8_t i = 0; i < eeprom->arrCount; i++) {
		// Pega a variável atual para facilitar o acesso
		var = eeprom->arrVar[i];

		//testa se o endereco da eeprom esta ok
		if (var->_addrEprom == 0) {
			return EEPROM_ERRO_ENDERECO_OBJ;
		}

		// Determine o tamanho do dado a ser lido com base no tipo.
		uint8_t dataSize = 0;
		switch (var->typeVar) {
		case DATA_8BITS: 	dataSize = 1; break;
		case DATA_16BITS: 	dataSize = 2; break;
		case DATA_32BITS: 	dataSize = 4; break;
		case DATA_FLOAT: 	dataSize = 4; break;
		case DATA_DOUBLE: 	dataSize = 8; break;
		}

		// limpe o buffer,
		memset(&buffer, 0, sizeof(uint8_t[8]));

		// Realiza leitura da EEPROM.
		while (tries++ < MAX_TRIES) {
			result = HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, var->_addrEprom, I2C_MEMADD_SIZE_16BIT,buffer, dataSize, 200);
			if (result == HAL_OK)	break;
			if (tries == MAX_TRIES)	return EEPROM_ERRO_LEITURA;
		}
		tries=0;

		// verifica e atribui o valor baseado no tipo.
		// certifica de que o valor está dentro dos limites.
		// envia para o ponteiro.
		switch (var->typeVar) {
		case DATA_8BITS:
			var->value.intValue = *((uint8_t*)&buffer);
			if (var->value.intValue > var->maxValue.intValue || var->value.intValue < var->minValue.intValue) {
				var->value.intValue = var->defaultValue.intValue;
			}
			*var->ptr8 = var->value.intValue; 	// envio para ponteiro.
			break;
		case DATA_16BITS:
			var->value.intValue = *((uint16_t*)&buffer);
			if (var->value.intValue > var->maxValue.intValue || var->value.intValue < var->minValue.intValue) {
				var->value.intValue = var->defaultValue.intValue;
			}
			*var->ptr16 = var->value.intValue; 	// envio para ponteiro
			break;
		case DATA_32BITS:
			var->value.intValue = *((uint32_t*)&buffer);
			if (var->value.intValue > var->maxValue.intValue || var->value.intValue < var->minValue.intValue) {
				var->value.intValue = var->defaultValue.intValue;
			}
			*var->ptr32 = var->value.intValue;
			break;
		case DATA_FLOAT:
			var->value.floatValue = *((float*)&buffer);
			if (var->value.floatValue > var->maxValue.floatValue || var->value.floatValue < var->minValue.floatValue) {
				var->value.floatValue = var->defaultValue.floatValue;
			}
			*var->ptrFloat = var->value.floatValue;
			break;
		case DATA_DOUBLE:
			var->value.doubleValue = *((double*)&buffer);
			if (var->value.doubleValue > var->maxValue.doubleValue || var->value.doubleValue < var->minValue.doubleValue) {
				var->value.doubleValue = var->defaultValue.doubleValue;
			}
			*var->ptrDouble = var->value.doubleValue;
			break;
		}
	}
	// testa se valor referencia é diferente do valor da eeprom
	return EEPROM_SUCCESS;
}
void 				download_containerEeprom	(Eeprom *eeprom){

	//funcao que implementa containerEeprom_download
	ErrorCode errCode = containerEeprom_download(eeprom);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(eeprom, errCode);
	}
}

ErrorCode 	containerEeprom_reset(Eeprom *eeprom, TypeRestauracao resetType) {

	HAL_StatusTypeDef 	result;
	uint8_t tries = 0, MAX_TRIES = 5;
	uint8_t 			buffer[sizeof(double)] __attribute__((aligned(sizeof(double)))); // Buffer pode conter até 64 bits (para DATADOUBLE)
	eepromVarArr		*var;
	if (!eeprom) {
		return EEPROM_OBJETO_NULO;
	}

	LIBERA_EEPROM

	//verifica disponibilidade da eeprom
	while (tries++ < MAX_TRIES) {
		result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle,EEPROM_WRITE_ADDR, 50, HAL_MAX_DELAY);
		if (result == HAL_OK)	break;
		if (tries == MAX_TRIES)	return EEPROM_I2C_ERROR;
	}
	tries=0;

	for (uint8_t i = 0; i < eeprom->arrCount; i++) {
		// Pega a variável atual para facilitar o acesso.
		var = eeprom->arrVar[i];

		// limpe o buffer
		memset(&buffer, 0, sizeof(uint8_t[8]));

		// Se for um soft reset, verifique se a variável pode ser resetada
		if (resetType == HARD_RESET || var->typeReset == SOFT_RESET) {
			// Reseta a variável para o defaultValue ou zero se não estiver definido
			// atualiza o ponteiro e o digital twin para esse valor.
			// escreve na eeprom.
			// *ps.: tenta escrever 5 vezes.
			switch (var->typeVar) {
			case DATA_8BITS:
				*var->ptr8 = var->defaultValue.intValue ? var->defaultValue.intValue : 0;
				var->value.intValue		= *var->ptr8;
				*((uint8_t*)&buffer) 	= var->value.intValue;

				//tentativas de escrita
				while (tries++ < MAX_TRIES) {
					result = HAL_I2C_Mem_Write(var->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, var->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 1, 200);
					if (result == HAL_OK)	break;
					if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
				}
				tries=0;
				break;
			case DATA_16BITS:
				*var->ptr16 = (var->defaultValue.intValue ? var->defaultValue.intValue : 0);
				var->value.intValue 	= *var->ptr16;
				*((uint16_t*)&buffer) 	= var->value.intValue;

				//tentativas de escrita
				while (tries++ < MAX_TRIES) {
					result = HAL_I2C_Mem_Write(var->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, var->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 2, 200);
					if (result == HAL_OK)	break;
					if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
				}
				tries=0;
				break;
			case DATA_32BITS:
				*var->ptr32 = var->defaultValue.intValue ? var->defaultValue.intValue : 0;
				var->value.intValue 	= *var->ptr32;
				*((uint32_t*)&buffer) 	= var->value.intValue;

				//tentativas de escrita
				while (tries++ < MAX_TRIES) {
					result = HAL_I2C_Mem_Write(var->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, var->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 4, 200);
					if (result == HAL_OK)	break;
					if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
				}
				tries=0;
				break;
			case DATA_FLOAT:
				*var->ptrFloat = var->defaultValue.floatValue ? var->defaultValue.floatValue : 0.0;
				var->value.floatValue 	= *var->ptrFloat;
				*((float*)&buffer) 		= var->value.floatValue;

				//tentativas de escrita
				while (tries++ < MAX_TRIES) {
					result = HAL_I2C_Mem_Write(var->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, var->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 4, 200);
					if (result == HAL_OK)	break;
					if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
				}
				tries=0;
				break;
			case DATA_DOUBLE:
				*var->ptrDouble = var->defaultValue.doubleValue ? var->defaultValue.doubleValue : 0.0;
				var->value.doubleValue 	= *var->ptrDouble;
				*((double*)&buffer) 	= (double)*var->ptrDouble;

				//tentativas de escrita
				while (tries++ < MAX_TRIES) {
					result = HAL_I2C_Mem_Write(var->parentEeprom->i2cHandle, EEPROM_WRITE_ADDR, var->_addrEprom, I2C_MEMADD_SIZE_16BIT, buffer, 8, 200);
					if (result == HAL_OK)	break;
					if (tries == MAX_TRIES)	return EEPROM_ERRO_ESCRITA;
				}
				tries=0;
				break;
			default:
				return EEPROM_TIPO_ERRADO; // Retorna código de erro apropriado para tipo desconhecido
				break;
			}
		}
		osDelay(20);
	}
	ErrorBuffer_clear(eeprom); // Limpa o buffer de erros)
	TRAVA_EEPROM
	return EEPROM_SUCCESS; // Retorna sucesso se todos os valores foram resetados
}
void 				reset_containerEeprom(Eeprom *eeprom, TypeRestauracao resetType) {

	//funcao que implementa containerEeprom_reset
	ErrorCode errCode = containerEeprom_reset(eeprom, resetType);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(eeprom, errCode);
	}
}

ErrorCode eepromAddr_write(Eeprom *eeprom, uint16_t addr, uint16_t size, uint8_t *data) {
	if (eeprom == NULL || data == NULL) {
		return EEPROM_OBJETO_NULO;
	}

	HAL_StatusTypeDef result;
	uint8_t tries = 0,MAX_TRIES = 5;;

	LIBERA_EEPROM

	// Verifica se a EEPROM está pronta para operações de escrita
	while (tries < MAX_TRIES) {
		result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR, 1, HAL_MAX_DELAY);
		if (result == HAL_OK) {
			break;
		}
		tries++;
	}

	if (tries >= MAX_TRIES) {
		TRAVA_EEPROM
		return EEPROM_I2C_ERROR; // EEPROM não está pronta após várias tentativas
	}

	// Escrita dos dados na EEPROM utilizando a função HAL_I2C_Mem_Write
	tries = 0; // Reinicializa o contador de tentativas
	while (tries < MAX_TRIES) {
		result = HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, addr, I2C_MEMADD_SIZE_16BIT, data, size, HAL_MAX_DELAY);
		if (result == HAL_OK) {
			break; // Dados gravados com sucesso
		}
		tries++;
	}

	TRAVA_EEPROM
	if (tries >= MAX_TRIES) {
		return EEPROM_ERRO_ESCRITA; // Falha ao escrever após várias tentativas
	}

	osDelay(5); // Pequeno atraso para garantir que a escrita tenha tempo para se estabelecer
	return EEPROM_SUCCESS; // Dados gravados com sucesso
}

void 				write_eepromAddr(Eeprom *eeprom, uint16_t addr, uint16_t size, uint8_t *data) {
	ErrorCode errCode = eepromAddr_write(eeprom,addr,size,data);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(eeprom, errCode);
	}
}

ErrorCode 	eepromAddr_read(Eeprom *eeprom, uint16_t _addrEprom, uint16_t size, uint8_t *value) {

	HAL_StatusTypeDef result;
	uint8_t tries = 0;
	const uint8_t MAX_TRIES = 5;

	LIBERA_EEPROM

	// verifica disponibilidade da eeprom.
	while (tries++ < MAX_TRIES) {
		result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR, 50, HAL_MAX_DELAY);
		if (result == HAL_OK)    break;
		if (tries == MAX_TRIES)  return EEPROM_I2C_ERROR;
	}

	// limita as tentativas de leitura.
	tries = 0;
	while (tries++ < MAX_TRIES) {
		result = HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, _addrEprom, I2C_MEMADD_SIZE_16BIT, value, size, 200);
		if (result == HAL_OK)    break;
		if (tries == MAX_TRIES)  return EEPROM_ERRO_LEITURA;
	}

	osDelay(40);
	TRAVA_EEPROM
	return EEPROM_SUCCESS;
}
void 				read_eepromAddr(Eeprom *eeprom, uint16_t _addrEprom, uint16_t size, uint8_t *value) {
	ErrorCode errCode = eepromAddr_read(eeprom, _addrEprom,size, value);
	if (errCode != EEPROM_SUCCESS) {
		eepromError_Handler(eeprom, errCode);
	}
}
// gerenciamento de erros.
void 			eepromError_Handler(Eeprom *eeprom, ErrorCode erro){
//	__disable_irq();
	while (1)
	{
		ErrorBuffer_add(eeprom, erro);
		NVIC_SystemReset();
	}
}
void ErrorBuffer_init(ErrorBuffer* ebuffer) {
	memset(ebuffer->errors, 0, sizeof(ebuffer->errors)); // Preenche com 0xFFFF que é valor padrão para EEPROM limpa
	ebuffer->errorCount = 0;
}

ErrorCode ErrorBuffer_add(Eeprom* eeprom, uint8_t errorCode) {
	// Verifica se o errorCode já está no buffer
	for (uint8_t i = 0; i < eeprom->errorBuffer.errorCount; i++) {
		if (eeprom->errorBuffer.errors[i] == errorCode) {
			return EEPROM_ERROR_EXISTS; // Erro já foi adicionado anteriormente
		}
	}

	// Adiciona o erro se houver espaço no buffer
	if (eeprom->errorBuffer.errorCount < MAX_ERRORS) {
		eeprom->errorBuffer.errors[eeprom->errorBuffer.errorCount] = errorCode;
		eeprom->write(eeprom, addrINIT_ERR + eeprom->errorBuffer.errorCount * sizeof(uint16_t), sizeof(uint8_t), &errorCode); // Grava novo erro na EEPROM
		eeprom->errorBuffer.errorCount++;
		return EEPROM_SUCCESS;
	}

	return EEPROM_ERROR_FULL; // Buffer de erros cheio
}
ErrorCode ErrorBuffer_read(Eeprom *eeprom) {
	if (eeprom == NULL ) {
		return EEPROM_OBJETO_NULO;
	}

	HAL_StatusTypeDef result;
	uint8_t buffer[MAX_ERRORS];  // Buffer para ler os erros da EEPROM (2 bytes por erro)
	int i;

	// Lê os erros da EEPROM
	result = HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, addrINIT_ERR,I2C_MEMADD_SIZE_16BIT, buffer, MAX_ERRORS, HAL_MAX_DELAY);
	if (result != HAL_OK) {
		return EEPROM_ERRO_LEITURA;
	}

	// Armazena os erros lidos no ErrorBuffer
	for (i = 0; i < MAX_ERRORS; i++) {
		if (buffer[i] != 0) {
			eeprom->errorBuffer.errors[eeprom->errorBuffer.errorCount++] = buffer[i];
		}
	}

	return EEPROM_SUCCESS;
}
void ErrorBuffer_clear(Eeprom* eeprom) {
	memset(eeprom->errorBuffer.errors, 0, sizeof(eeprom->errorBuffer.errors)); // Zera o buffer de erro
	eeprom->write(eeprom, addrINIT_ERR, sizeof(eeprom->errorBuffer.errors), eeprom->errorBuffer.errors); // Zera o registro na EEPROM
	eeprom->errorBuffer.errorCount = 0;
}

#endif /* SRC_EEPROM_H_ */











