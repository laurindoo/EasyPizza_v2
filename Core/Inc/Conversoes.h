/*
 * Conversoes.h
 *
 *  Created on: Dec 6, 2023
 *      Author: lucas
 */

#ifndef INC_CONVERSOES_H_
#define INC_CONVERSOES_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"


typedef union {
    double value; // Valor de ponto flutuante
    uint8_t bytes[8]; // Representação em bytes do ponto flutuante
} doubleAsBytes;

typedef union {
    float value; // Valor de ponto flutuante
    uint8_t bytes[4]; // Representação em bytes do ponto flutuante
} floatAsBytes;

typedef union {
    uint16_t value;
    uint8_t bytes[2];
} shortAsBytes;

typedef union {
    uint8_t value;
    uint8_t bytes[1];
} bytetAsBytes;

typedef union {
    uint32_t value;
    uint8_t bytes[4];
} uint32AsBytes;


void vetor4b_TO_Double(double *destino, volatile uint8_t *vetor, uint8_t inicio);
void vetor2b_TO_uint16(uint16_t *destino, volatile uint8_t *vetor, uint8_t inicio);
void vetor2b_TO_Double(double *destino, volatile uint8_t *vetor, uint8_t inicio);
void vetor8b_TO_Double(double *destino, uint8_t *vetor, uint8_t inicio);
void float_TO_vetor4b(double valor, uint8_t *vetor, uint8_t inicio);
void double_TO_vetor8b(double valor, uint8_t *vetor, uint8_t inicio);

#endif /* INC_CONVERSOES_H_ */
