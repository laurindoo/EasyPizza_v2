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

CRC_short CRC16 (unsigned char *puchMsg, unsigned short usDataLen);
uint8_t validaCRC(CRC_short referencia, CRC_short chave);

#endif /* INC_CRC_H_ */
