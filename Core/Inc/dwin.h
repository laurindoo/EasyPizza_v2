///*
// * dwin.h
// *
// *  Created on: Dec 13, 2023
// *      Author: lucas
// */
//
//
//#define NEXTION_OK  0x0A
//
////Reminder: To use more components or to receive more characters increase buffer or list counts!
//#define DWIN_TIMEOUT 50
//#define DWIN_MAX_BUFF_LEN 96
//#define DWIN_TEXT_BUFF_LEN 64
//#define DWIN_MAX_COMP_COUNT 32
//#define MAX_DWIN_COMPONENTS 36
//
//#define NEX_RET_CMD_FINISHED                 (0x01)
//#define NEX_RET_EVENT_LAUNCHED               (0x88)
//#define NEX_RET_EVENT_UPGRADED               (0x89)
//#define NEX_RET_EVENT_TOUCH_HEAD             (0x65)
//#define NEX_RET_EVENT_POSITION_HEAD          (0x67)
//#define NEX_RET_EVENT_SLEEP_POSITION_HEAD    (0x68)
//#define NEX_RET_CURRENT_PAGE_ID_HEAD         (0x66)
//#define NEX_RET_STRING_HEAD                  (0x70)
//#define NEX_RET_NUMBER_HEAD                  (0x71)
//#define NEX_RET_INVALID_CMD                  (0x00)
//#define NEX_RET_INVALID_COMPONENT_ID         (0x02)
//#define NEX_RET_INVALID_PAGE_ID              (0x03)
//#define NEX_RET_INVALID_PICTURE_ID           (0x04)
//#define NEX_RET_INVALID_FONT_ID              (0x05)
//#define NEX_RET_INVALID_BAUD                 (0x11)
//#define NEX_RET_INVALID_VARIABLE             (0x1A)
//#define NEX_RET_INVALID_OPERATION            (0x1B)
//#define NEX_EVENT_ON_PRESS                   (0x01)
//#define NEX_EVENT_ON_RELEASE                 (0x00)
//
//
//
////---Estados do display
//typedef enum
//{   DispNotInit = 0,
//	DispInit = 1,
//	DispDesconected,
//} state_display;
//
//
///*
// * DwinComp Struct
// * 	trabalharemos apenas com uint16_t e se necessário, tratamos
// */
//typedef struct
//{
//	//Variables for storing ID for every component
//	uint16_t dwinId;
//
//	//Variable for storing object name
//	char *objname;
//
//	//Local Values
//	uint16_t *_varLocal;
//	uint16_t lastSend;
//
//	//Display received values
//	uint16_t dispValRX;
//
//} DwinVar;
//
//
///*
// * Dwin Struct
// */
//typedef struct
//{
//	//Handles globais
//	DMA_HandleTypeDef 	*UARTDMAHandle;
//	UART_HandleTypeDef 	*UARTHandle;
//
//	//Variables for parsing the received data
//	uint8_t _RxDataArr[DWIN_MAX_BUFF_LEN], _RxData, _arrCount, _pkgCount,RxSize;
//
//	osMessageQId	*filaComandosRX;//Handle da fila de comandos
//
//	//Variables for component list
//	NexComp* _NexCompArr[DWIN_MAX_COMP_COUNT];
//	uint8_t _NexCompCount;
//
//	//Variables for receiving strings and numbers,
//	int32_t NextNumBuff;
//	uint8_t NexTextBuff[DWIN_TEXT_BUFF_LEN], NextTextLen;
//
//	state_display state;
//
//} Nextion;
