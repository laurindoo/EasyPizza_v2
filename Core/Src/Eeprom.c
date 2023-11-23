/*
 * Eeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#include "Eeprom.h"


//variaveis globais da EEprom
#ifndef INC_EEPROM_C_
#define INC_EEPROM_C_

uint8_t EepromInit(Eeprom *eeprom, I2C_HandleTypeDef *i2c, osMessageQId *fila)
{
	//Pass the used I2C handle to the struct
	eeprom->i2cHandle = i2c;

	//Pass the used queue to the struct
	eeprom->filaComandos = fila;

	//Start the component count variable from zero
	eeprom->_EepromVarCount  = 0;

	//Return OK
	return 0;
}

void EepromUpdateMes(Eeprom *eeprom, uint16_t __addMes, uint16_t __addItem, uint32_t valor, TypeTamData _sizeType)
{
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
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,__addMes+__addItem, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer0, 1, 200);
			break;
		case DATA16BITS:
			buffer1[0] = (uint8_t)(valor >> 8);
			buffer1[1] = (uint8_t)(valor & 0xFF);
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, __addMes+__addItem, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer1, 2, 200);
			break;
		case DATA32BITS:
			buffer2[0] = (uint8_t)(valor >> 24);
			buffer2[1] = (uint8_t)(valor >> 16);
			buffer2[2] = (uint8_t)(valor >> 8);
			buffer2[3] = (uint8_t)(valor & 0xFF);
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, __addMes+__addItem, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2, 4, 200);
			break;
		default:
			break;
		}
	}
}

//variaveis de eeprom que serao manipuladas
uint8_t EepromAddVar(Eeprom *eeprom, EepromVariaveis* _eepromvar, char* objectname,uint8_t __addreeprom,TypeTamData tamanho,uint32_t minimo,uint32_t padrao,uint32_t maximo, void *addrVar)
{
	//Make space before passing the object name to the nexcomp struct
	_eepromvar->objname = (char *) malloc((strlen(objectname)*sizeof(char)) + 1);

	//Pass the object name to the struct
	strcpy(_eepromvar->objname, objectname);

	//Pass the corresponding data from component to component struct
	_eepromvar->_addrEprom = __addreeprom;

	//definindo minimos maximos e default
	_eepromvar->minValue 		= minimo;
	_eepromvar->defaultValue 	= padrao;
	_eepromvar->maxValue 		= maximo;

	//passando o tamnho da variavel
	_eepromvar->_sizeType = tamanho;

	//Add the component struct to the list on the Nextion Struct
	eeprom->_EepromVarArr[eeprom->_EepromVarCount] = _eepromvar;
	eeprom->_EepromVarCount++;


	if(!addrVar)
		return 0;
	switch (tamanho) {
	case DATA8BITS:
		_eepromvar->ptr8=(uint8_t *)addrVar;
		break;
	case DATA16BITS:
		_eepromvar->ptr16=(uint16_t *)addrVar;
		break;
	case DATA32BITS:
		_eepromvar->ptr32=(uint32_t *)addrVar;
		break;
	case DATADOUBLE:
		_eepromvar->ptrDouble=(double *)addrVar;
		break;
	}
	return 0;
}

bool EepromSetVar(Eeprom *eeprom, EepromVariaveis *eepromvar, uint32_t valor)
{
	//retomar leitura direto da variavel interna no objeto
	HAL_StatusTypeDef result;
	uint8_t buffer0[1];
	uint8_t buffer1[2];
	uint8_t buffer2[4];
	uint8_t buffer3[8];
	//envio para memoria pagina 1
	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,30,HAL_MAX_DELAY);
	if (result==HAL_OK)
	{
		if(valor!=0){ //valor recebido
			switch (eepromvar->_sizeType) {
			case DATA8BITS:
				eepromvar->valor = (uint8_t)valor;
				buffer0[0] = (uint8_t)valor;
				HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer0, 1, 200);

				if(!eepromvar->ptr8)
					break;
				*eepromvar->ptr8 = (uint8_t)valor;

				break;
			case DATA16BITS:
				eepromvar->valor = (uint16_t)valor;
				buffer1[0] = (uint8_t)(valor >> 8);
				buffer1[1] = (uint8_t)(valor & 0xFF);
				HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer1, 2, 200);

				if(!eepromvar->ptr16)
					break;
				*eepromvar->ptr16 = (uint16_t)valor;
				break;
			case DATA32BITS:
				eepromvar->valor = (uint32_t)valor;
				buffer2[0] = (uint8_t)(valor >> 24);
				buffer2[1] = (uint8_t)(valor >> 16);
				buffer2[2] = (uint8_t)(valor >> 8);
				buffer2[3] = (uint8_t)(valor & 0xFF);
				HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2, 4, 200);

				if(!eepromvar->ptr32)
					break;
				*eepromvar->ptr32 = (uint32_t)valor;
				break;
			case DATADOUBLE:
				    eepromvar->valor = (double)valor;
				    union {
				        double asDouble;
				        uint64_t asUInt;
				    } doubleToUInt;

				    doubleToUInt.asDouble = eepromvar->valor;
				    uint64_t valAsUInt = doubleToUInt.asUInt;

	//			    uint64_t valAsUInt = *(uint64_t*)&(eepromvar->valor);
				    for (int i = 0; i < 8; i++) {
				        buffer3[i] = (uint8_t)(valAsUInt >> (56 - 8*i));
				    }
				    HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer3, 8, 200);

				    if (!eepromvar->ptrDouble)
				        break;
				    *eepromvar->ptrDouble = (double)valor;
				    break;
			}
		}else{ //usar valor da var local
			switch (eepromvar->_sizeType) {
			case DATA8BITS:
				if(!eepromvar->ptr8)
					break;
				eepromvar->valor = *eepromvar->ptr8;
				buffer0[0] = *eepromvar->ptr8;

				HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer0, 1, 200);

				break;
			case DATA16BITS:
				if(!eepromvar->ptr16)
					break;
				eepromvar->valor = *eepromvar->ptr16;
				buffer1[0] = (uint8_t)(*eepromvar->ptr16 >> 8);
				buffer1[1] = (uint8_t)(*eepromvar->ptr16 & 0xFF);

				HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer1, 2, 200);
				break;
			case DATA32BITS:
				if(!eepromvar->ptr32)
					break;
				eepromvar->valor = *eepromvar->ptr32;
				buffer2[0] = (uint8_t)(*eepromvar->ptr32 >> 24);
				buffer2[1] = (uint8_t)(*eepromvar->ptr32 >> 16);
				buffer2[2] = (uint8_t)(*eepromvar->ptr32 >> 8);
				buffer2[3] = (uint8_t)(*eepromvar->ptr32 & 0xFF);

				HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2, 4, 200);
				break;
			case DATADOUBLE:
			    if (!eepromvar->ptrDouble)
			        break;
			    union {
			        double asDouble;
			        uint64_t asUInt;
			    } doubleToUInt2;
			    doubleToUInt2.asDouble = *eepromvar->ptrDouble;
			    uint64_t valAsUInt2 = doubleToUInt2.asUInt;
			    for (int i = 0; i < 8; i++) {
			        buffer3[i] = (uint8_t)(valAsUInt2 >> (56 - 8*i));
			    }
			    HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer3, 8, 200);
			    break;
			}
		}
	}
	//
	//	//envio para memoria pagina 1
	//	LIBERA_EEPROM
	//	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,30,HAL_MAX_DELAY);
	//	if (result==HAL_OK)
	//	{
	//		switch (eepromvar->_sizeType) {
	//		case DATA8BITS:
	//			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer0, 1, 200);
	//			break;
	//		case DATA16BITS:
	//			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buffer1, 2, 200);
	//			break;
	//		case DATA32BITS:
	//			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR, eepromvar->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2, 4, 200);
	//			break;
	//		default:
	//			break;
	//		}
	//	}
	//
	//	return result;
	TRAVA_EEPROM
	return 1;
}

void EepromDownloadValores(Eeprom *eeprom)
{
	uint8_t buffer1[2];
	uint8_t buffer2[4];
	uint8_t buffer3[8];

//	return (uchCRCHi << 8 | uchCRCLo)
	for(uint8_t i = 0; i < eeprom->_EepromVarCount; i++){

		if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,30,HAL_MAX_DELAY)==HAL_OK){

			switch (eeprom->_EepromVarArr[i]->_sizeType) {
			case DATA8BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,eeprom->_EepromVarArr[i]->_addrEprom,I2C_MEMADD_SIZE_16BIT,(uint8_t *)&eeprom->_EepromVarArr[i]->valor, DATA8BITS, 200);
				if(eeprom->_EepromVarArr[i]->ptr8 !=0){
					*eeprom->_EepromVarArr[i]->ptr8 = eeprom->_EepromVarArr[i]->valor;
				}
				break;
			case DATA16BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,eeprom->_EepromVarArr[i]->_addrEprom,I2C_MEMADD_SIZE_16BIT,(uint8_t *) buffer1, DATA16BITS, 200);
				eeprom->_EepromVarArr[i]->valor =  ( buffer1[0] << 8 | buffer1[1]);
				if(eeprom->_EepromVarArr[i]->ptr16 !=0){
					*eeprom->_EepromVarArr[i]->ptr16 = eeprom->_EepromVarArr[i]->valor;
				}
				break;
			case DATA32BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,eeprom->_EepromVarArr[i]->_addrEprom,I2C_MEMADD_SIZE_16BIT,(uint8_t *) buffer2, DATA32BITS, 200);
				eeprom->_EepromVarArr[i]->valor =  ( buffer2[0] << 24 | buffer2[1] << 16 | buffer2[2] << 8 | buffer2[3]);
				if(eeprom->_EepromVarArr[i]->ptr32 !=0){
					*eeprom->_EepromVarArr[i]->ptr32 = eeprom->_EepromVarArr[i]->valor;
				}
				break;
			case DATADOUBLE:
			    HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, eeprom->_EepromVarArr[i]->_addrEprom, I2C_MEMADD_SIZE_16BIT, (uint8_t *) buffer3, 8, 200);
			    uint64_t valAsUInt = ((uint64_t)buffer3[0] << 56) |
			                         ((uint64_t)buffer3[1] << 48) |
			                         ((uint64_t)buffer3[2] << 40) |
			                         ((uint64_t)buffer3[3] << 32) |
			                         ((uint64_t)buffer3[4] << 24) |
			                         ((uint64_t)buffer3[5] << 16) |
			                         ((uint64_t)buffer3[6] << 8)  |
			                         (uint64_t)buffer3[7];
			    union {
			        uint64_t asUInt;
			        double asDouble;
			    } uintToDouble;
			    uintToDouble.asUInt = valAsUInt;
			    double val = uintToDouble.asDouble;
			    if (eeprom->_EepromVarArr[i]->ptrDouble != 0) {
			        *eeprom->_EepromVarArr[i]->ptrDouble = val;
			    }
			    break;
			}
		}

		//caso a variavel esteja zerada, atribui o valor padrao
		if(eeprom->_EepromVarArr[i]->defaultValue>0 && eeprom->_EepromVarArr[i]->valor == 0)
			EepromSetVar(eeprom, eeprom->_EepromVarArr[i], eeprom->_EepromVarArr[i]->defaultValue);

		HAL_Delay(10);
	}
}

void RestauraPadraoTudo(Eeprom *eeprom)
{
	//redefine para default
	for(uint8_t i = 0; i < eeprom->_EepromVarCount; i++){
		EepromSetVar(eeprom, eeprom->_EepromVarArr[i], eeprom->_EepromVarArr[i]->defaultValue);
		HAL_Delay(10);
	}
}



void EepromReadVal(Eeprom *eeprom, uint16_t addr, uint8_t *var, uint16_t size)
{
	if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,30,HAL_MAX_DELAY)==HAL_OK){
		HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,addr,I2C_MEMADD_SIZE_16BIT,(uint8_t *)&var, size, 200);
	}
}

void Write_1_byte(Eeprom *eeprom, uint16_t addr, uint8_t * ptr)
{

	uint8_t *buffer = 0;
	buffer = (uint8_t *)(ptr);

	LIBERA_EEPROM
	//envio para memoria pagina 1
	HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,addr, I2C_MEMADD_SIZE_16BIT,(uint8_t *) buffer, 1, 100);
	TRAVA_EEPROM

}//---END---//



#endif /* SRC_EEPROM_H_ */











