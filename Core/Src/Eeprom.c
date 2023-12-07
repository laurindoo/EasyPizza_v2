/*
 * Eeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#include "Eeprom.h"
#include "Conversoes.h"



//variaveis globais da EEprom
#ifndef INC_EEPROM_C_
#define INC_EEPROM_C_

uint8_t EepromInit	(Eeprom *eeprom, I2C_HandleTypeDef *i2c, osMessageQId *fila){
	//Pass the used I2C handle to the struct
	eeprom->i2cHandle = i2c;

	//Pass the used queue to the struct
	eeprom->filaComandos = fila;

	//Start the component count variable from zero
	eeprom->_EepromVarCount  = 0;
	eeprom->_EepromVarFloatingCount  = 0;

	EepromAddVar(eeprom, 0, &eeprom->RefFlag,	"addrREF_MEM_FLAG",	addrREF_MEM_FLAG,	DATA8BITS,	0,	STD_REF_MEM	,254	,0);

	//Return OK
	return 0;
}
void EepromUpdateMes(Eeprom *eeprom, uint16_t addMes, uint16_t addItem, uint32_t valor, TypeData _sizeType){
	HAL_StatusTypeDef result;
	uint8_t buffer0[1];
	uint8_t buffer1[2];
	uint8_t buffer2[4];

	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,30,HAL_MAX_DELAY);
	if (result==HAL_OK)
	{
		switch (_sizeType) {
		case DATA8BITS:
			buffer0[0] = (uint8_t)valor;
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,addMes+addItem, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer0, 1, 200);
			break;
		case DATA16BITS:
			buffer1[0] = (uint8_t)(valor >> 8);
			buffer1[1] = (uint8_t)(valor & 0xFF);
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, addMes+addItem, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer1, 2, 200);
			break;
		case DATA32BITS:
			buffer2[0] = (uint8_t)(valor >> 24);
			buffer2[1] = (uint8_t)(valor >> 16);
			buffer2[2] = (uint8_t)(valor >> 8);
			buffer2[3] = (uint8_t)(valor & 0xFF);
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, addMes+addItem, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2, 4, 200);
			break;
		default:
			break;
		}
	}
}
uint8_t EepromAddVar(Eeprom *eeprom, bool resetavel, EepromVariaveis* _var, char* _name, uint16_t addr, TypeData tipo, uint32_t minimo, uint32_t padrao,uint32_t maximo, void *_addrVar){
	//todo implementar erros e tratar de alguma forma
	//Make space before passing the object name to the nexcomp struct
	_var->objname = (char *) malloc((strlen(_name)*sizeof(char)) + 1);

	//Nome do objeto
	strcpy(_var->objname, _name);

	//Endereco
	_var->_addrEprom = addr;

	//passando o tamnho da variavel
	_var->_sizeType = tipo;

	//torna nao resetavel ao softreset
	_var->flagResetavel = resetavel;


	switch (tipo) {
	case DATA8BITS:
		//definindo minimos maximos e default
		_var->minValue 		= (uint8_t)minimo;
		_var->defaultValue 	= (uint8_t)padrao;
		_var->maxValue 		= (uint8_t)maximo;
		_var->ptr8			= (uint8_t *)_addrVar;
		break;
	case DATA16BITS:
		//definindo minimos maximos e default
		_var->minValue 		= (uint16_t)minimo;
		_var->defaultValue 	= (uint16_t)padrao;
		_var->maxValue 		= (uint16_t)maximo;
		_var->ptr16			= (uint16_t *)_addrVar;
		break;
	case DATA32BITS:
		//definindo minimos maximos e default
		_var->minValue 		= (uint32_t)minimo;
		_var->defaultValue 	= (uint32_t)padrao;
		_var->maxValue 		= (uint32_t)maximo;
		_var->ptr32			= (uint32_t *)_addrVar;
		break;
	default:
		//erro, parametro errado
		break;
	}

	//Adiciona o componente na respectiva estrutura
	eeprom->_EepromVarArr[eeprom->_EepromVarCount] = _var;
	eeprom->_EepromVarCount++;

	return 0;
}
uint8_t EepromAddVarFloating(Eeprom *eeprom, bool resetavel, EepromVarFloating* _var, char* _name,uint16_t addr,TypeDataFloating tipo,double minimo,double padrao,double maximo, void *_addrVar)
{
	//todo implementar erros e tratar de alguma forma
	//Make space before passing the object name to the nexcomp struct
	_var->objname = (char *) malloc((strlen(_name)*sizeof(char)) + 1);

	//Nome do objeto
	strcpy(_var->objname, _name);

	//Endereco
	_var->_addrEprom = addr;

	//passando o tamnho da variavel
	_var->_sizeType = tipo;

	//torna nao resetavel ao softreset
	_var->flagResetavel = resetavel;
	switch (tipo) {
	case DATAFLOAT:
		//definindo minimos maximos e default
		_var->minValue 		= (float)minimo;
		_var->defaultValue 	= (float)padrao;
		_var->maxValue 		= (float)maximo;
		_var->ptrFloat		= (float *)_addrVar;
		break;
	case DATADOUBLE:
		//definindo minimos maximos e default
		_var->minValue 		= (double)minimo;
		_var->defaultValue 	= (double)padrao;
		_var->maxValue 		= (double)maximo;
		_var->ptrDouble		= (double *)_addrVar;
		break;
	}

	//Adiciona o componente na respectiva estrutura
	eeprom->_EepromVarFloatingArr[eeprom->_EepromVarFloatingCount] = _var;
	eeprom->_EepromVarFloatingCount++;

	return 0;
}
bool EepromSetVar	(Eeprom *eeprom, EepromVariaveis *_var, uint32_t valor){
	//retomar leitura direto da variavel interna no objeto
	HAL_StatusTypeDef result;
	uint8_t 		buffer1b[1];
	shortAsBytes 	buffer2b;
	uint32AsBytes	buffer4b;

	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,50,HAL_MAX_DELAY);
	if (result==HAL_OK)
	{
		switch (_var->_sizeType) {
		case DATA8BITS:
			_var->valor = *_var->ptr8;

			//grava na memoria
			buffer1b[0] 		= (uint8_t)_var->valor;
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer1b, 1, 200);

			break;
		case DATA16BITS:
			_var->valor = *_var->ptr16;

			//grava na memoria
			buffer2b.value 		= (uint16_t)_var->valor;
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2b.bytes, 2, 200);

			break;
		case DATA32BITS:
			_var->valor = *_var->ptr32;

			//grava na memoria
			buffer4b.value 		= (uint32_t)_var->valor;
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer4b.bytes, 4, 200);

			break;
		}
	}
	osDelay(20);
	TRAVA_EEPROM
	return 1;
}
bool EepromSetVarFloating	(Eeprom *eeprom, EepromVarFloating *_var, double valor){
	//retomar leitura direto da variavel interna no objeto
	__IO HAL_StatusTypeDef result;
	__IO floatAsBytes	floatBuff;
	__IO doubleAsBytes 	doubleBuff;

	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,50,HAL_MAX_DELAY);
	if (result==HAL_OK)
	{
		switch (_var->_sizeType) {

		case DATAFLOAT:
			_var->valorFloat = *_var->ptrFloat;

			//grava na memoria
			floatBuff.value = (float)_var->valorFloat;
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)floatBuff.bytes, 4, 200);

			break;
		case DATADOUBLE:
			_var->valorDouble = *_var->ptrDouble;

			//grava na memoria
			doubleBuff.value = (double)_var->valorDouble;
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)doubleBuff.bytes, 8, 200);

			break;
		}
	}
	osDelay(30);
	TRAVA_EEPROM
	return 1;
}
void EepromDownloadValores	(Eeprom *eeprom){
	bytetAsBytes 	buffer1b;
	shortAsBytes 	buffer2b;
	uint32AsBytes	buffer4b;

	doubleAsBytes 	doubleBuff;
	floatAsBytes	floatBuff;

	//for para variaveis normais
	for(uint8_t i = 0; i < eeprom->_EepromVarCount; i++){

		if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,50,HAL_MAX_DELAY)==HAL_OK){

			switch (eeprom->_EepromVarArr[i]->_sizeType) {
			case DATA8BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,eeprom->_EepromVarArr[i]->_addrEprom,I2C_MEMADD_SIZE_16BIT,buffer1b.bytes, 1, 200);
				eeprom->_EepromVarArr[i]->valor = buffer1b.value;
				if(eeprom->_EepromVarArr[i]->ptr8 !=0){
					*eeprom->_EepromVarArr[i]->ptr8 = eeprom->_EepromVarArr[i]->valor;
				}
				break;
			case DATA16BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,eeprom->_EepromVarArr[i]->_addrEprom,I2C_MEMADD_SIZE_16BIT,buffer2b.bytes, 2, 200);//tentar usar o & todo
				eeprom->_EepromVarArr[i]->valor = buffer2b.value;
				if(eeprom->_EepromVarArr[i]->ptr16 !=0){
					*eeprom->_EepromVarArr[i]->ptr16 = eeprom->_EepromVarArr[i]->valor;
				}
				break;
			case DATA32BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,eeprom->_EepromVarArr[i]->_addrEprom,I2C_MEMADD_SIZE_16BIT,buffer4b.bytes, 4, 200);
				eeprom->_EepromVarArr[i]->valor =  buffer4b.value;
				if(eeprom->_EepromVarArr[i]->ptr32 !=0){
					*eeprom->_EepromVarArr[i]->ptr32 = eeprom->_EepromVarArr[i]->valor;
				}
				break;
			}

			//caso a variavel esteja zerada, atribui o valor padrao
			if(eeprom->_EepromVarArr[i]->defaultValue>0 && (eeprom->_EepromVarArr[i]->valor == 0 || eeprom->_EepromVarArr[i]->valor > eeprom->_EepromVarArr[i]->maxValue || eeprom->_EepromVarArr[i]->valor < eeprom->_EepromVarArr[i]->minValue))
				EepromSetVar(eeprom, eeprom->_EepromVarArr[i], eeprom->_EepromVarArr[i]->defaultValue);
		}
		//for para variaveis floating
		for(uint8_t y = 0; y < eeprom->_EepromVarFloatingCount; y++){

			if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,30,HAL_MAX_DELAY)==HAL_OK){

				switch (eeprom->_EepromVarFloatingArr[y]->_sizeType) {

				case DATAFLOAT:
					HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, eeprom->_EepromVarFloatingArr[y]->_addrEprom, I2C_MEMADD_SIZE_16BIT, floatBuff.bytes, 4, 200);
					eeprom->_EepromVarFloatingArr[y]->valorFloat =  floatBuff.value;
					if (eeprom->_EepromVarFloatingArr[y]->ptrFloat != 0) {
						*eeprom->_EepromVarFloatingArr[y]->ptrFloat = eeprom->_EepromVarFloatingArr[y]->valorFloat;
					}

					//caso a variavel esteja zerada, atribui o valor padrao
					if(eeprom->_EepromVarFloatingArr[y]->defaultValue>0 && (eeprom->_EepromVarFloatingArr[y]->valorFloat == 0 || eeprom->_EepromVarFloatingArr[y]->valorFloat > eeprom->_EepromVarFloatingArr[y]->maxValue || eeprom->_EepromVarFloatingArr[y]->valorFloat < eeprom->_EepromVarFloatingArr[y]->minValue))
						EepromSetVarFloating(eeprom, eeprom->_EepromVarFloatingArr[y], (float)eeprom->_EepromVarFloatingArr[y]->defaultValue);
					break;
				case DATADOUBLE:
					HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, eeprom->_EepromVarFloatingArr[y]->_addrEprom, I2C_MEMADD_SIZE_16BIT, doubleBuff.bytes, 8, 200);
					eeprom->_EepromVarFloatingArr[y]->valorDouble =  doubleBuff.value;
					if (eeprom->_EepromVarFloatingArr[y]->ptrDouble != 0) {
						*eeprom->_EepromVarFloatingArr[y]->ptrDouble = eeprom->_EepromVarFloatingArr[y]->valorDouble;
					}

					//caso a variavel esteja zerada, atribui o valor padrao
					if(eeprom->_EepromVarFloatingArr[y]->defaultValue>0 && (eeprom->_EepromVarFloatingArr[y]->valorDouble == 0 || eeprom->_EepromVarFloatingArr[y]->valorDouble > eeprom->_EepromVarFloatingArr[y]->maxValue || eeprom->_EepromVarFloatingArr[y]->valorDouble < eeprom->_EepromVarFloatingArr[y]->minValue))
						EepromSetVarFloating(eeprom, eeprom->_EepromVarFloatingArr[y], (double)eeprom->_EepromVarFloatingArr[y]->defaultValue);
					break;
				}
			}
		}

		//Leitura referencia
		if(eeprom->RefFlag.valor != STD_REF_MEM)
		{
			//executa reset de eeprom HARDRESET
			RestauraEeprom(eeprom, hardReset);
			return;
		}
	}
}
void RestauraEeprom			(Eeprom *eeprom ,TypeRestauracao tipo){
	HAL_StatusTypeDef result;

	__IO bytetAsBytes	buffer1b;
	__IO shortAsBytes 	buffer2b;
	__IO uint32AsBytes	buffer4b;
	__IO floatAsBytes	floatBuff;
	__IO doubleAsBytes 	doubleBuff;

	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,30,HAL_MAX_DELAY);
	if (result==HAL_OK){
		//redefine para default variaveis normais
		for(uint8_t i = 0; i < eeprom->_EepromVarCount; i++){
			//verifica se item é resetavel
			if(!eeprom->_EepromVarArr[i]->flagResetavel || tipo == hardReset){
				switch (eeprom->_EepromVarArr[i]->_sizeType) {
				case DATA8BITS:
					buffer1b.value 					= (uint8_t)eeprom->_EepromVarArr[i]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarArr[i]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer1b.bytes, 1, 200);
					break;
				case DATA16BITS:
					buffer2b.value 					= (uint16_t)eeprom->_EepromVarArr[i]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarArr[i]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2b.bytes, 2, 200);
					break;
				case DATA32BITS:
					buffer4b.value 					= (uint32_t)eeprom->_EepromVarArr[i]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarArr[i]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer4b.bytes, 4, 200);
					break;
				}
			}
			osDelay(20);
		}
		//redefine para default variaveis floating
		for(uint8_t k = 0; k < eeprom->_EepromVarFloatingCount; k++){
			//verifica se item é resetavel
			if(!eeprom->_EepromVarFloatingArr[k]->flagResetavel || tipo == hardReset){
				switch (eeprom->_EepromVarFloatingArr[k]->_sizeType) {
				case DATAFLOAT:
					floatBuff.value 							= (float)eeprom->_EepromVarFloatingArr[k]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarFloatingArr[k]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)floatBuff.bytes, 4, 200);
					break;
				case DATADOUBLE:
					doubleBuff.value 							= (double)eeprom->_EepromVarFloatingArr[k]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarFloatingArr[k]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)doubleBuff.bytes, 8, 200);
					break;
				}
			}
			osDelay(40);
		}

		//grava valor padrao standart
		buffer1b.value					= (uint8_t)STD_REF_MEM;
		eeprom->RefFlag.valor			= buffer1b.value;
		HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->RefFlag._addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer1b.bytes, 1, 200);
		osDelay(40);
	}
	TRAVA_EEPROM
}
void EepromReadVal			(Eeprom *eeprom, uint16_t addr, uint8_t *_var, uint16_t size){
	if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,30,HAL_MAX_DELAY)==HAL_OK){
		HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,addr,I2C_MEMADD_SIZE_16BIT,(uint8_t *)&_var, size, 200);
	}
}
void Write_1_byte			(Eeprom *eeprom, uint16_t addr, uint8_t * _ptr){

	uint8_t *buffer = 0;
	buffer = (uint8_t *)(_ptr);

	LIBERA_EEPROM
	//envio para memoria pagina 1
	HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,addr, I2C_MEMADD_SIZE_16BIT,(uint8_t *) buffer, 1, 100);
	TRAVA_EEPROM

}//---END---//

#endif /* SRC_EEPROM_H_ */











