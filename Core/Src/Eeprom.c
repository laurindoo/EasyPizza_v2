/*
 * Eeprom.c
 *
 *  Created on: Jul 25, 2023
 *      Author: lucas
 */

#include "Eeprom.h"
#include "Conversoes.h"

//todo revisar logica de restauracao


//variaveis globais da EEprom.
#ifndef INC_EEPROM_C_
#define INC_EEPROM_C_

EEPROM_ErrorCode EepromInit	(Eeprom *eeprom, I2C_HandleTypeDef *i2c, osMessageQId *fila){

	eeprom->i2cHandle 				= i2c;	// handler de I2C.
	eeprom->filaComandos 			= fila; // fila de salvamento.
	eeprom->_EepromVarCount  		= 0; 	// contador de elementos unsigned.
	eeprom->_EepromVarFloatingCount = 0; 	// contador de elementos flutuantes.

	// variavel de referencia para autoreset na primeira vez ao ligar.
	EepromAddVar(eeprom, 0, &eeprom->RefFlag,	"addrREF_MEM_FLAG",	addrREF_MEM_FLAG,	DATA8BITS,	0,	STD_REF_MEM	,254	,0);

	return EEPROM_SUCCESS;
}
EEPROM_ErrorCode EepromAddVar(Eeprom *eeprom, bool resetavel, EepromVariaveis* _var, char* _name, uint16_t addr, TypeData tipo, uint32_t minimo, uint32_t padrao,uint32_t maximo, void *_addrVar){

	_var->objname = (char *) malloc((strlen(_name)*sizeof(char)) + 1);
	strcpy(_var->objname, _name);		// nome do objeto.
	_var->_addrEprom 	= addr;			// endereço eemprom.
	_var->_sizeType 	= tipo;			// tipo da variavel.
	_var->flagResetavel = resetavel;	// softreset ou hardreset.

	//definindo minimos maximos e default.
	switch (tipo) {
	case DATA8BITS:
		_var->minValue 		= (uint8_t)minimo;
		_var->defaultValue 	= (uint8_t)padrao;
		_var->maxValue 		= (uint8_t)maximo;
		_var->ptr8			= (uint8_t *)_addrVar;
		break;
	case DATA16BITS:
		_var->minValue 		= (uint16_t)minimo;
		_var->defaultValue 	= (uint16_t)padrao;
		_var->maxValue 		= (uint16_t)maximo;
		_var->ptr16			= (uint16_t *)_addrVar;
		break;
	case DATA32BITS:
		_var->minValue 		= (uint32_t)minimo;
		_var->defaultValue 	= (uint32_t)padrao;
		_var->maxValue 		= (uint32_t)maximo;
		_var->ptr32			= (uint32_t *)_addrVar;
		break;
	default:
		eepromError_Handler(EEPROM_TIPO_ERRADO);
		return EEPROM_TIPO_ERRADO;
		break;
	}

	//Adiciona o componente na lista unsigned.
	eeprom->_EepromVarArr[eeprom->_EepromVarCount] = _var;
	eeprom->_EepromVarCount++;

	return EEPROM_SUCCESS;
}
uint8_t EepromAddVarFloating(Eeprom *eeprom, bool resetavel, EepromVarFloating* _var, char* _name,uint16_t addr,TypeDataFloating tipo,double minimo,double padrao,double maximo, void *_addrVar){

	_var->objname = (char *) malloc((strlen(_name)*sizeof(char)) + 1);
	strcpy(_var->objname, _name);		// nome do objeto.
	_var->_addrEprom 	= addr;         // endereço eemprom.
	_var->_sizeType 	= tipo;         // tipo da variavel.
	_var->flagResetavel = resetavel;    // softreset ou hardreset.

	//definindo minimos maximos e default.
	switch (tipo) {
	case DATAFLOAT:
		_var->minValue 		= (float)minimo;
		_var->defaultValue 	= (float)padrao;
		_var->maxValue 		= (float)maximo;
		_var->ptrFloat		= (float *)_addrVar;
		break;
	case DATADOUBLE:
		_var->minValue 		= (double)minimo;
		_var->defaultValue 	= (double)padrao;
		_var->maxValue 		= (double)maximo;
		_var->ptrDouble		= (double *)_addrVar;
		break;
	default:
		eepromError_Handler(EEPROM_TIPO_ERRADO);
		return EEPROM_TIPO_ERRADO;
		break;
	}

	//Adiciona o componente na lista float.
	eeprom->_EepromVarFloatingArr[eeprom->_EepromVarFloatingCount] = _var;
	eeprom->_EepromVarFloatingCount++;

	return EEPROM_SUCCESS;
}
bool EepromSetVar	(Eeprom *eeprom, EepromVariaveis *_var){
	//retomar leitura direto da variavel interna no objeto
	HAL_StatusTypeDef result;
	uint8_t 		buffer1b[1];
	shortAsBytes 	buffer2b;
	uint32AsBytes	buffer4b;

	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,50,HAL_MAX_DELAY);//todo tratar se nao tiver disponivel?
	if (result==HAL_OK)	{
		switch (_var->_sizeType) {
		case DATA8BITS:
			_var->valor 		= *_var->ptr8; 				// valor apontado.
			buffer1b[0] 		= (uint8_t)_var->valor;		// vetoriza.
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer1b, 1, 200);// grava na eeprom.
			break;
		case DATA16BITS:
			_var->valor 		= *_var->ptr16;				// valor apontado.
			buffer2b.value 		= (uint16_t)_var->valor;	// vetoriza.
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2b.bytes, 2, 200);// grava na eeprom.
			break;
		case DATA32BITS:
			_var->valor 		= *_var->ptr32;				// valor apontado.
			buffer4b.value 		= (uint32_t)_var->valor;	// vetoriza.
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer4b.bytes, 4, 200);// grava na eeprom.
			break;
		}
	}
	osDelay(20);
	TRAVA_EEPROM
	return 1;
}
bool EepromSetVarFloating	(Eeprom *eeprom, EepromVarFloating *_var){
	//retomar leitura direto da variavel interna no objeto
	HAL_StatusTypeDef result;
	floatAsBytes	floatBuff;
	doubleAsBytes 	doubleBuff;

	LIBERA_EEPROM
	result = HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_WRITE_ADDR,50,HAL_MAX_DELAY);//todo tratar se nao tiver disponivel?
	if (result==HAL_OK)	{
		switch (_var->_sizeType) {
		case DATAFLOAT:
			_var->valorFloat 	= *_var->ptrFloat;         	// valor apontado.
			floatBuff.value 	= (float)_var->valorFloat;  // vetoriza.
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)floatBuff.bytes, 4, 200); // grava na eeprom.
			break;
		case DATADOUBLE:
			_var->valorDouble 	= *_var->ptrDouble;         // valor apontado.
			doubleBuff.value 	= (double)_var->valorDouble;// vetoriza.
			HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,_var->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)doubleBuff.bytes, 8, 200); // grava na eeprom
			break;
		}
	}
	osDelay(30);
	TRAVA_EEPROM
	return 1;
}
void EepromDownloadValores	(Eeprom *eeprom){

	/*--- leitura da lista unsigned ---*/
	// buffers auxiliares na leitura
	bytetAsBytes 	buffer1b; 	//leitura de 1 byte (uint8_t).
	shortAsBytes 	buffer2b; 	//leitura de 2 bytes (uint16_t).
	uint32AsBytes	buffer4b; 	//leitura de 4 bytes (uint32_t).
	EepromVariaveis *Var; 		//Ponteiro auxiliar direto para a variavela ser manipulada.

	// varredura por todos os elementos unsigned.
	for(uint8_t i = 0; i < eeprom->_EepromVarCount; i++){
		// leitura de buffers para facilitar leitura.
		Var = eeprom->_EepromVarArr[i];

		if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,50,HAL_MAX_DELAY)==HAL_OK){

			switch (Var->_sizeType) {
			case DATA8BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,Var->_addrEprom,I2C_MEMADD_SIZE_16BIT,buffer1b.bytes, 1, 200);
				Var->valor = buffer1b.value;	// digital twin
				if(Var->ptr8 !=0){
					*Var->ptr8 = Var->valor; 	// envio para ponteiro
				}
				break;
			case DATA16BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,Var->_addrEprom,I2C_MEMADD_SIZE_16BIT,buffer2b.bytes, 2, 200);
				Var->valor = buffer2b.value;	// digital twin
				if(Var->ptr16 !=0){
					*Var->ptr16 = Var->valor; 	// envio para ponteiro
				}
				break;
			case DATA32BITS:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,Var->_addrEprom,I2C_MEMADD_SIZE_16BIT,buffer4b.bytes, 4, 200);
				Var->valor =  buffer4b.value;	// digital twin
				if(Var->ptr32 !=0){
					*Var->ptr32 = Var->valor; 	// envio para ponteiro
				}
				break;
			}
			//caso a variavel esteja zerada, atribui o valor padrao todo revisar pois nao esta puxando valor padrao
			if(Var->defaultValue > 0 && (Var->valor == 0 || Var->valor > Var->maxValue || Var->valor < Var->minValue))
				EepromSetVar(eeprom, Var);
		}
	}

	/*--- leitura da lista signed ---*/
	// buffers auxiliares na leitura
	floatAsBytes		floatBuff;	//leitura de 4 bytes (float).
	doubleAsBytes 		doubleBuff;	//leitura de 8 bytes (double).
	EepromVarFloating 	*VarU; 		//Ponteiro auxiliar direto para a variavela ser manipulada.

	// varredura por todos os elementos signed.
	for(uint8_t y = 0; y < eeprom->_EepromVarFloatingCount; y++){
		// leitura de buffers para facilitar leitura.
		VarU = eeprom->_EepromVarFloatingArr[y];

		if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,30,HAL_MAX_DELAY)==HAL_OK){

			switch (VarU->_sizeType) {

			case DATAFLOAT:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, VarU->_addrEprom, I2C_MEMADD_SIZE_16BIT, floatBuff.bytes, 4, 200);
				VarU->valorFloat =  floatBuff.value;
				if (VarU->ptrFloat != 0) {
					*VarU->ptrFloat = VarU->valorFloat;
				}

				//caso a variavel esteja zerada, atribui o valor padrao
				if(VarU->defaultValue>0 && (VarU->valorFloat == 0 || VarU->valorFloat > VarU->maxValue || VarU->valorFloat < VarU->minValue))
					EepromSetVarFloating(eeprom, VarU);
				break;
			case DATADOUBLE:
				HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR, VarU->_addrEprom, I2C_MEMADD_SIZE_16BIT, doubleBuff.bytes, 8, 200);
				VarU->valorDouble =  doubleBuff.value;
				if (VarU->ptrDouble != 0) {
					*VarU->ptrDouble = VarU->valorDouble;
				}

				//caso a variavel esteja zerada, atribui o valor padrao todo revisar pois nao esta puxando valor padrao
				if(VarU->defaultValue > 0 && (VarU->valorDouble == 0 || VarU->valorDouble > VarU->maxValue || VarU->valorDouble < VarU->minValue))
					EepromSetVarFloating(eeprom, VarU);
				break;
			}
		}
	}// end for para variaveis floatin

	//Leitura referencia
	if(eeprom->RefFlag.valor != STD_REF_MEM)
	{
		//executa reset de eeprom HARDRESET
		RestauraEeprom(eeprom, hardReset);
		return;
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
					buffer1b.value					=0x00;
					buffer1b.value 					= (uint8_t)eeprom->_EepromVarArr[i]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarArr[i]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer1b.bytes, 1, 200);
					break;
				case DATA16BITS:
					buffer2b.value					=0x0000;
					buffer2b.value 					= (uint16_t)eeprom->_EepromVarArr[i]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarArr[i]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)buffer2b.bytes, 2, 200);
					break;
				case DATA32BITS:
					buffer4b.value					=0x00000000;
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
					floatBuff.value 							= 0x0000;
					floatBuff.value 							= (float)eeprom->_EepromVarFloatingArr[k]->defaultValue;
					HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,eeprom->_EepromVarFloatingArr[k]->_addrEprom, I2C_MEMADD_SIZE_16BIT,(uint8_t *)floatBuff.bytes, 4, 200);
					break;
				case DATADOUBLE:
					doubleBuff.value 							= 0x00000000;
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
	//todo condicionar caso nao possivel ler, tomar açao
	if (HAL_I2C_IsDeviceReady(eeprom->i2cHandle, EEPROM_READ_ADDR,30,HAL_MAX_DELAY)==HAL_OK){
		HAL_I2C_Mem_Read(eeprom->i2cHandle, EEPROM_READ_ADDR,addr,I2C_MEMADD_SIZE_16BIT,(uint8_t *)&_var, size, 200);
	}
}
void Write_1_byte			(Eeprom *eeprom, uint16_t addr, uint8_t * _ptr){
	//todo confirmar uso dessa funcao
	uint8_t *buffer = 0;
	buffer = (uint8_t *)(_ptr);

	LIBERA_EEPROM
	//envio para memoria pagina 1
	HAL_I2C_Mem_Write(eeprom->i2cHandle, EEPROM_WRITE_ADDR,addr, I2C_MEMADD_SIZE_16BIT,(uint8_t *) buffer, 1, 100);
	TRAVA_EEPROM

}//---END---//
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
void 			eepromError_Handler(EEPROM_ErrorCode erro)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#endif /* SRC_EEPROM_H_ */











