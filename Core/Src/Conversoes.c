/*
 * Conversoes.c
 *
 *  Created on: Dec 6, 2023
 *      Author: lucas
 */
#include "Conversoes.h"

void vetor4b_TO_Double(double *destino, uint8_t *vetor, uint8_t inicio) {
	// Verificar se o destino e o vetor não são nulos
	floatAsBytes myValue;

	// Copia os bytes do double para o vetor, um por vez, começando no índice 'inicio'
	for (int i = 0; i < 4; ++i) {
		myValue.bytes[i] = vetor[inicio + i] ;
		printf("%d \n",myValue.bytes[i]);
	}

	*destino = (double)myValue.value;
}
void vetor2b_TO_uint16(uint16_t *destino, uint8_t *vetor, uint8_t inicio) {

	// Verificar se o destino e o vetor não são nulos
	if (destino == NULL || vetor == NULL) {
		printf("Parâmetros inválidos para a função extraiFloatComoDoubleDoVetor.\n");
		return;
	}

	*destino = vetor[inicio] << 8 | vetor[inicio+1];
}
void vetor2b_TO_Double(double *destino, uint8_t *vetor, uint8_t inicio) {

	// Verificar se o destino e o vetor não são nulos
	if (destino == NULL || vetor == NULL) {
		printf("Parâmetros inválidos para a função extraiFloatComoDoubleDoVetor.\n");
		return;
	}

	*destino = vetor[inicio] << 8 | vetor[inicio+1];
}
void vetor8b_TO_Double(double *destino, uint8_t *vetor, uint8_t inicio) {

	// Verificar se o destino e o vetor não são nulos
	if (destino == NULL || vetor == NULL) {
		printf("Parâmetros inválidos para a função extraiFloatComoDoubleDoVetor.\n");
		return;
	}

	// Cria uma união e armazena o valor double parts nela
	doubleAsBytes myValue;

	// Copia os bytes do float para o vetor, um por vez, começando no índice 'inicio'
	for (int i = 0; i < sizeof(myValue.value); ++i) {
		myValue.bytes[i] = vetor[inicio + i];
	}

	*destino = myValue.value;
}
void float_TO_vetor4b(double valor, uint8_t *vetor, uint8_t inicio) {

	// Cria uma união e armazena o valor float nela
	floatAsBytes myValue;
	myValue.value = (float)valor;

	// Copia os bytes do float para o vetor, um por vez, começando no índice 'inicio'
	for (int i = 0; i < sizeof(myValue.value); ++i) {
		vetor[inicio + i] = myValue.bytes[i];
	}
}
void double_TO_vetor8b(double valor, uint8_t *vetor, uint8_t inicio) {

	// Cria uma união e armazena o valor float nela
	floatAsBytes myValue;
	myValue.value = (double)valor;

	// Copia os bytes do float para o vetor, um por vez, começando no índice 'inicio'
	for (int i = 0; i < sizeof(myValue.value); ++i) {
		vetor[inicio + i] = myValue.bytes[i];
	}
}

