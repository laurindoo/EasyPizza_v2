/*
 * Crc.h
 *
 *  Created on: Dec 18, 2023
 *      Author: lucas
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

// Defina o tamanho máximo do seu log
#define MAX_LOG_SIZE 28

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
 * Estrutura Para armazenagem do log
 */
typedef struct {
    uint8_t logs[MAX_LOG_SIZE]; // Array de bytes para salvar os logs
    size_t currentPos;          // Posição atual no array
} LogSystem;




CRC_short CRC16 (volatile unsigned char *puchMsg, unsigned short usDataLen);
uint8_t validaCRC(CRC_short referencia, CRC_short chave);




#endif /* INC_CRC_H_ */
