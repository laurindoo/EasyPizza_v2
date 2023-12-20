/*
 * Bluetooth.c
 *
 *  Created on: Jul 24, 2023
 *      Author: lucas
 */

#include "Bluetooth.h"

/**
 * \brief 	Contrutor do objeto bluetooth.
 * \param 	*ble - Ponteiro para o proprio objeto pre-criado.
 * \param 	*UARTHandle - Ponteiro de handle da interrupcao uart.
 * \param 	*UARTHandle - Ponteiro de handle do DMA uart.
 * \param 	*_filaRX - Ponteiro para fila de recebimento de comandos ja tratados.
 * \param 	*_filaTX - Ponteiro para fila de comandos a serem enviados.
 * \param 	*_filaComandoInternoTX - Ponteiro para fila de comandos internos de configuracao da classe.
 * \return 	Retorna BLE_SUCCESS em caso de contrucao bem sucedida.
 *         	Retorna OBJETO_NULO caso objeto nao criado.
 *         	Retorna EXTRAPOLOU_TRY_BLE caso tentativas de inicializacao tenham falhado.
 */
BLE_ErrorCode 	bleConstrutora(Bluetooth *ble, UART_HandleTypeDef *UARTHandle, DMA_HandleTypeDef *UARTDMAHandle, osMessageQId *filaRX, osMessageQId *filaTX, osMessageQId *filaComandoInternoTX){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL || UARTHandle == NULL || UARTDMAHandle == NULL || filaRX == NULL || filaTX == NULL || filaComandoInternoTX == NULL) {
		return BLE_OBJETO_NULO;
	}

	// Inicialização das variáveis do objeto Bluetooth.
	ble->UARTHandle 	= UARTHandle;
	ble->UARTDMAHandle 	= UARTDMAHandle;
	ble->filaComandosRX = filaRX;
	ble->filaComandosTX = filaTX;
	ble->filaComandoInternoTX = filaComandoInternoTX;
	ble->contComandos 	= 0; 	// Renomeado para ter um nome mais explícito e sem _ inicial.
	ble->JanelaConexao 	= 120; 	// 120 segundos.

	// Inicialização do hardware.
	if (iniciaBleHm10(ble) == BLE_EXTRAPOLOU_TRY) {
		// Faça o log do erro ou tome uma ação apropriada aqui.
		bleError_Handler(BLE_EXTRAPOLOU_TRY);
		return BLE_EXTRAPOLOU_TRY;
	}

	// Inicialização de objetos de conexão.
	if (bleAddCompConexao(ble, &ble->BLEPedeSenha, RX_PEDE_SENHA) != BLE_SUCCESS ||
			bleAddCompConexao(ble, &ble->BLERecebeuSenha, RX_RECEBEU_SENHA) != BLE_SUCCESS) {
		bleError_Handler(BLE_OBJETO_NAO_CRIADO);
		return BLE_OBJETO_NAO_CRIADO; // Use a constante apropriada para erro.
	}

	//funcoes de respostas.
	ble->aknowladge	= sendAknowladge;

	// Se tudo correu bem, retorne sucesso.
	return BLE_SUCCESS;
}
/**
 * \brief 	Adiciona componentes do tipo ComandoBasico no objeto
 * \param 	*ble - Ponteiro para o objeto pai
 * \param 	*_blecomm - Ponteiro do comando a ser cadastrado, pré-criado
 * \param 	__comando - Comando definido em typedef geralmente no header da tarefa que usará
 * \return 	Retorna BLE_SUCCESS em caso de contrucao bem sucedida
 *         	Retorna OBJETO_NULO caso objeto nao criado.
 *         	Retorna BLE_OBJETO_NAO_CRIADO caso array esteja fora do range definido.
 */
BLE_ErrorCode 	bleAddComp(Bluetooth* ble, BleComando* _blecomm, uint8_t __comando){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL || _blecomm == NULL) {
		bleError_Handler(BLE_OBJETO_NULO);
		return BLE_OBJETO_NULO;
	}

	// Verifica tamanho.
	if(ble->contComandos>=BLUETOOTH_MAX_COMANDOS_COUNT){
		bleError_Handler(BLE_COMANDO_NAO_CRIADO);
		return BLE_COMANDO_NAO_CRIADO;
	}

	// Pass the corresponding data from component to component struct.
	_blecomm->_comando 	= __comando;
	_blecomm->_tipo 	= ComandoBasico;

	// Add the component struct to the list on the Nextion Struct.
	ble->_BleCommArr[ble->contComandos] = _blecomm;
	ble->contComandos++;

	// Return OK.
	return BLE_SUCCESS;
}
/**
 * \brief 	Adiciona componentes do tipo ComandoConexao no objeto
 * \param 	*ble - Ponteiro para o objeto pai
 * \param 	*_blecomm - Ponteiro do comando a ser cadastrado, pré-criado
 * \param 	__comando - Comando definido em typedef geralmente no header da tarefa que usará
 * \return 	Retorna BLE_SUCCESS em caso de contrucao bem sucedida
 *         	Retorna OBJETO_NULO caso objeto nao criado.
 *         	Retorna BLE_OBJETO_NAO_CRIADO caso array esteja fora do range definido.
 */
BLE_ErrorCode 	bleAddCompConexao(Bluetooth* ble, BleComando* _blecomm, uint8_t __comando){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL || _blecomm == NULL) {
		bleError_Handler(BLE_OBJETO_NULO);
		return BLE_OBJETO_NULO;
	}

	// Verifica tamanho.
	if(ble->contComandos>=BLUETOOTH_MAX_COMANDOS_COUNT){
		bleError_Handler(BLE_COMANDOCON_NAO_CRIADO);
		return BLE_COMANDOCON_NAO_CRIADO;
	}

	// Pass the corresponding data from component to component struct.
	_blecomm->_comando 	= __comando;
	_blecomm->_tipo 	= ComandoConexao;

	// Add the component struct to the list on the Nextion Struct.
	ble->_BleCommArr[ble->contComandos] = _blecomm;
	ble->contComandos++;

	// Return OK.
	return BLE_SUCCESS;
}
/**
 * \brief 	Função para manipular interrupções do DMA no módulo Bluetooth.
 * 			Esta função é chamada quando um novo conjunto de dados é recebido via DMA.
 * \param 	*ble - Ponteiro para o objeto pai
 * \return 	todo tratar retorno
 */
void 			BLEDMA_IrqHandler (Bluetooth *ble)
{
	// Armazenda endereco MAC do HM-10.
	uint8_t 	addr8Bits	[12];
	//variaveis de comparacao de texto.
	char	StringRecebida[BLUETOOTH_MAX_BUFF_LEN];
	long 	PontoExato;
	char 	*ss;

	// Certifique-se de que a fonte da interrupção é o evento de transferência completa.
	if(__HAL_DMA_GET_IT_SOURCE(ble->UARTDMAHandle, DMA_IT_TC) != RESET){

		/* Clear the transfer complete flag */
		__HAL_DMA_CLEAR_FLAG(ble->UARTDMAHandle, __HAL_DMA_GET_TC_FLAG_INDEX(ble->UARTDMAHandle));

		// Calculo do tamanho da string recebida.
		ble->RxSize 		= DMA_RX_BUFFER_SIZE - ble->UARTDMAHandle->Instance->CNDTR;

		// Copia os dados do buffer de DMA para a StringRecebida para processamento.
		sprintf(StringRecebida,"%s",ble->_RxDataArr);

		// Zera contador desconectado, pois recebeu mensagem.
		ble->msDesconectado = 0;

		// Processamento da máquina de estados da conexão.
		switch(ble->MaquinaConexao){
		case RX_HARDWARE_INIT:
			ble->msIdle=0;
			/*---   A D D R  ---*/

			// Processa o recebimento do endereço MAC do dispositivo e armazena uma chava baseada nele.
			ss = NULL;
			ss = strstr(StringRecebida, "OK+ADDR:");

			if (ss != NULL){
				PontoExato = ss - StringRecebida;
				for (int i = 0; i < 12; i++) {
					addr8Bits[i] = ble->_RxDataArr[i+PontoExato+8];
				}
				ble->chave = CRC16(addr8Bits,12);
			}
			break;
		case RX_DESCONECTADO:
			ble->msIdle=0;
			/*---   C O N E C T O U  ---*/
			ble->StatusSenha = false;
			ss = NULL;
			ss = strstr(StringRecebida, "OK+CONN");

			// Processamento de mensagem de conexao ou sinal hardware de conexao.
			if ((ss != NULL  && ble->RxSize == TAMANHO_MENSAGEM_CONECTOU) || MACRO_LE_BT_STATUS){
				ble->MaquinaConexao = RX_CONECTADO;
			}

			break;
		case RX_CONECTADO:
			/*---   D E S C O N E C T O U  ---*/
			ss = NULL;
			ss = strstr(StringRecebida, "LOST");
			if (ss != NULL || !MACRO_LE_BT_STATUS){
				ble->StatusSenha 		= false;
				ble->MaquinaConexao 	= RX_DESCONECTADO;
			}else{

				//Leitura apenas de COMANDOS DE CONEXAO.
				if(readComando(ble,ComandoConexao)!=0)	{
					__NOP();//ERRO todo tratar esse erro.
				}
			}
			break;
		case RX_VALIDADO:

			// Somente apos validado é permitido resetar tempo de inatividade em ms.
			ble->msIdle=0;
			/*---   D E S C O N E C T O U  ---*/
			ss = NULL;
			ss = strstr(StringRecebida, "LOST");

			// Processamento de mensagem de desconexao ou sinal hardware de desconexao.
			if (ss != NULL || !MACRO_LE_BT_STATUS){
				ble->StatusSenha 		= false;
				ble->MaquinaConexao 	= RX_DESCONECTADO;
			}else{

				//Leitura apenas de COMANDOS BASICO.
				if(readComando(ble,ComandoBasico)!=0)	{
					__NOP();//ERRO todo tratar esse erro.
				}
			}
			break;
		}

		// Prepara o DMA para a próxima transferência.
		/* Important! DMA stream won't start if all flags are not cleared first */
		ble->UARTDMAHandle->Instance->CMAR = (uint32_t)ble->_RxDataArr; /* Set memory address for DMA again */
		ble->UARTDMAHandle->Instance->CNDTR = DMA_RX_BUFFER_SIZE;    	/* Set number of bytes to receive */
		ble->UARTDMAHandle->Instance->CCR |= DMA_CCR_EN;            	/* Start DMA transfer */
	}
}
/**
 * \brief 	Realiza a extracao do comando do array RxDataArr e vincula ao ComandoAtual
 * 				de acordo com o tipo de comando aceitavel: ComandoBasico OU ComandoConexao.
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	tipo - tipo de comando aceitavel: ComandoBasico OU ComandoConexao.
 * \return 	Retorna BLE_SUCCESS em caso de leitura de comando com sucesso.
 *         	Retorna BLE_CRC_INCORRETO caso crc errado. Envia automaticamente na uart o comando de erro de CRC.
 *         	Retorna BLE_COMANDO_NAO_ENCONTRADO caso comando nao encontrado na lista.
 *         	Retorna BLE_COMANDO_NAO_PERMITIDO caso comando nao coerente com o parametro -tipo.
 */
BLE_ErrorCode 	readComando(Bluetooth* ble, TypeComandoBle tipo){

	BleComando comandoRX;
	uint8_t ret;

	// Validação de CRC.
	if (bluetoothErroCRC(ble)) {
		return BLE_CRC_INCORRETO;
	}

	// Varredura pelo array de comandos.
	for(uint8_t i = 0; i < ble->contComandos; i++)	{
		comandoRX = *ble->_BleCommArr[i];

		// identifica comando no array.
		if( ble->_RxDataArr[1] == comandoRX._comando){
			ble->ComandoAtual = comandoRX;
			ret = bluetoothPutFila(ble,tipo);
			return ret;
		}
	}
	//todo apagar dados lidos LIMPAR BUFFER.
	return BLE_COMANDO_NAO_ENCONTRADO;
}
/**
 * \brief 	Analisa comando lido no ble->ComandoAtual e aloca-o na fila
 * 				de comandos de conexao(filaComandoInternoTX) ou leitura de comandos(filaComandosRX)
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	tipo - tipo de comando aceitavel: ComandoBasico OU ComandoConexao.
 * \return 	Retorna BLE_SUCCESS em caso de leitura de comando com sucesso.
 *         	Retorna BLE_CRC_INCORRETO caso crc errado. Envia automaticamente na uart o comando de erro de CRC.
 *         	Retorna BLE_COMANDO_NAO_ENCONTRADO caso comando nao encontrado na lista.
 *         	Retorna BLE_COMANDO_NAO_PERMITIDO caso comando nao coerente com o parametro -tipo
 */
BLE_ErrorCode 	bluetoothPutFila(Bluetooth* ble, TypeComandoBle tipo){

	switch (tipo) {
	case ComandoBasico:
		if(ble->ComandoAtual._tipo == ComandoBasico ){
			osMessagePut(*ble->filaComandosRX, ble->ComandoAtual._comando, 0);
			return BLE_SUCCESS;
		}

		//Tipo do comando diference do autorizado.
		return BLE_COMANDO_NAO_PERMITIDO;
		break;
	case ComandoConexao:
		if(ble->ComandoAtual._tipo == ComandoConexao ){

			switch (ble->ComandoAtual._comando) {
			case RX_PEDE_SENHA:
				osMessagePut(*ble->filaComandoInternoTX, COMANDO_SOLICITACAO_SENHA, 0);
				return BLE_SUCCESS;
				break;
			case RX_RECEBEU_SENHA:
				osMessagePut(*ble->filaComandoInternoTX, COMANDO_AVALIACAO_CHAVE, 0);
				return BLE_SUCCESS;
				break;
			}
		}

		//Tipo do comando diference do autorizado.
		return BLE_COMANDO_NAO_PERMITIDO;
		break;
	}

	//Tipo de comando nao cadastrado.
	return BLE_COMANDO_NAO_ENCONTRADO;
}
/**
 * \brief 	Processa solicitacoes de senha e entradas ao sistema
 * \param 	*ble - Ponteiro para o objeto pai.
 * \return 	Retorna BLE_SUCCESS em caso de leitura de comando com sucesso.
 *         	Retorna BLE_CRC_INCORRETO caso crc errado. Envia automaticamente na uart o comando de erro de CRC.
 *         	Retorna BLE_COMANDO_NAO_ENCONTRADO caso comando nao encontrado na lista.
 *         	Retorna BLE_COMANDO_NAO_PERMITIDO caso comando nao coerente com o parametro -tipo
 */
BLE_ErrorCode 	txBleComando(Bluetooth *ble){

	// Usamos um ponteiro para uint8_t para copiar byte a byte.
	unsigned char	Buffer		[10];
	osEvent  evttx;
	evttx = osMessageGet(*ble->filaComandoInternoTX, 0);
	if (evttx.status == osEventMessage) {
		switch ((unsigned int)evttx.value.p) {
		case COMANDO_SOLICITACAO_SENHA:
			if(ble->JanelaConexao > 0){

				/*----DENTRO DO TEMPO, ENTAO RESPONDE----*/
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x51;
				Buffer[2] 	= 0x51;
				Buffer[3] 	= 0x01;
				Buffer[4] 	= ble->chave.byte.hi	;
				Buffer[5] 	= ble->chave.byte.lo	;
				bluetoothEnviaComando(ble,Buffer, 5);
				return BLE_SUCCESS;

			}else{
				/*----FORA DO TEMPO DE RESPOSTA ----*/
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x51;
				Buffer[2] 	= 0x51;
				Buffer[3] 	= 0x00;
				Buffer[4] 	= 0x00;
				Buffer[5] 	= 0x00;
				bluetoothEnviaComando(ble,Buffer, 5);
				return BLE_NEW_DEVICE_NEGADO;
				//o Aplicativo esta se encarregando dessa desconxao pós senha errada.
				//				bluetoothDescon(ble);
			}
			break;
		case COMANDO_AVALIACAO_CHAVE:
			CRC_short chaveApp;
			chaveApp.byte.hi = ble->_RxDataArr[3];
			chaveApp.byte.lo = ble->_RxDataArr[4];
			if(validaCRC(ble->chave, chaveApp)){
				//--->	CHAVE ERRADA
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x52;
				Buffer[2] 	= 0x52;
				Buffer[3] 	= 0x00;
				bluetoothEnviaComando(ble,Buffer, 3);
				return BLE_SENHA_ERRADA;

				//o Aplicativo esta se encarregando dessa desconxao pós senha errada.
				//				bluetoothDescon(ble);
			}else{
				//--->	CHAVE CORRETA
				ble->MaquinaConexao	= RX_VALIDADO;
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x52;
				Buffer[2] 	= 0x52;
				Buffer[3] 	= 0x01;
				bluetoothEnviaComando(ble,Buffer, 3);
				return BLE_SUCCESS;
			}
			break;
		default:
			return BLE_COMANDO_NAO_ENCONTRADO;
			break;

		}
	}
	return BLE_SUCCESS;
}
/**
 * \brief 	Inicializa o hardware com sequencia de comandos
 * \param 	*ble - Ponteiro para o objeto pai.
 * \return 	Retorna BLE_SUCCESS em caso de hardware inicializado com sucesso.
 *         	Retorna BLE_EXTRAPOLOU_TRY caso excedido tentativas de capturar MAC addres.
 * \info*  	Prestar atencao nas configuracoes do header bluetooth.h
 */
BLE_ErrorCode 	iniciaBleHm10(Bluetooth* ble){
#define M_BLE_RESET comandHM10(ble,"AT+RESET",400);//RESETA
#define SETUP_UART(baud_rate) \
		HAL_UART_Abort_IT(ble->UARTHandle);\
		HAL_UART_DeInit(ble->UARTHandle);\
		osDelay(50);\
		ble->UARTHandle->Init.BaudRate = baud_rate;\
		HAL_UART_Init(ble->UARTHandle);\
		osDelay(50);

	const uint8_t max_attempts = 15;

	while(ble->sequenciaBLE !=final || ble->sequenciaBLE!=erro){
		switch (ble->sequenciaBLE) {
		case inicio:

			MACRO_RESET_BLE		//HARDRESET NO BLE_HM10
			comandHM10(ble,"AT+ADTY3",100);	//BLOQUEIA CONEXAO
			comandHM10(ble,"AT+ADTY3",100);	//BLOQUEIA CONEXAO
			SETUP_UART(115200)
			bluetoothDescon(ble);
			MACRO_DEFINE_INTERRUPT

			ble->sequenciaBLE = redefineBle;

			break;
		case redefineBle:

			//seta em 115200
			SETUP_UART(115200)
			comandHM10(ble,"AT",100);	//
			comandHM10(ble,"AT",100);	//
			comandHM10(ble,"AT+RENEW",1000);	//RESTAURA PADRAO FABRICA

			//seta em 9600
			SETUP_UART(9600)
			comandHM10(ble,"AT+RENEW",1000);	//RESTAURA PADRAO FABRICA

			comandHM10(ble,"AT",100);	//
			comandHM10(ble,"AT",100);	//
			comandHM10(ble,"AT+ADTY3",300);	//BLOQUEIA CONEXAO
			comandHM10(ble,"AT+BAUD4",300);	//COLOCA BAUD EM 115200
			//seta em 115200
			SETUP_UART(115200)
			MACRO_RESET_BLE

			//CONFIGURA CENTRAL
			comandHM10(ble,"AT",100);	//
			comandHM10(ble,"AT",100);	//
			comandHM10(ble,"AT+POWE3",300);	//POTENCIA MAXIMA
			comandHM10(ble,"AT+SHOW3",300);	//MOSTRA O NOME e rssi
			comandHM10(ble,"AT+GAIN1",300);	//INSERE GANHO
			comandHM10(ble,"AT+NOTI1",300);	//NOTIFICA QUE CONECTOU
			comandHM10(ble,"AT+PIO11",300);	//1 - CONECT = 1  \  DISC = 0
			char comando[COMANDO_BUFFER_SIZE]; 		// Buffer para o comando AT
			snprintf(comando, sizeof(comando), "AT+NAME%s", BLE_DEVICE_NAME);
			comandHM10(ble,comando, 400); 	// Configura o nome no dispositivo

			M_BLE_RESET

			ble->sequenciaBLE = capturaAddr;

			break;
		case capturaAddr:
			static uint8_t tryingAddr=0;
			while (tryingAddr < max_attempts) {

				comandHM10(ble,"AT+ADDR?",300);//pede addr
				MACRO_DEFINE_INTERRUPT

				if (ble->chave.hilo != 0){
					ble->sequenciaBLE = final;
					MACRO_DEFINE_INTERRUPT
					tryingAddr=0;
					break;
				} else {
					tryingAddr++;
					break;
				}
			}

			if(tryingAddr >= max_attempts){
				ble->sequenciaBLE = erro;//extrapolou as tentativas
			}
			break;
		case final:
			comandHM10(ble,"AT+ADTY0",300);	//DESBLOQUEIA CONEXA
			MACRO_RESET_BLE

			/*---HABILITA INTERRUPÇÃO---*/
			__HAL_UART_ENABLE_IT 	(ble->UARTHandle, UART_IT_IDLE);						// HABILITA idle line INTERRUPT
			__HAL_DMA_ENABLE_IT 	(ble->UARTDMAHandle, DMA_IT_TC);					// HABILITA O DMA Tx cplt INTERRUPT
			HAL_UART_Receive_DMA 	(ble->UARTHandle, ble->_RxDataArr, DMA_RX_BUFFER_SIZE);	// STARTA O UART1 EM DMA MODE
			ble->MaquinaConexao = RX_DESCONECTADO;

			return BLE_SUCCESS;
			break;
		case erro:
			return BLE_EXTRAPOLOU_TRY;
			NVIC_SystemReset();
			break;
		}
	}
}
/**
 * \brief 	Processa informacao recebida com chave apropriada
 * \param 	*ble - Ponteiro para o objeto pai.
 * \return 	Retorna BLE_SUCCESS em caso de hardware inicializado com sucesso.
 *         	Retorna BLE_CRC_INCORRETO no caso de erro de mensagem ou crc incorreto.
 */
BLE_ErrorCode 	bluetoothErroCRC(Bluetooth* ble)
{
	//buffer de envio de msg
	unsigned char	TXCRC[3];

	//buffer calculo de crc
	CRC_short 		CRCReceive,CRCKey;

	//captura chave recebida pelo App
	CRCKey.byte.hi	= ble->_RxDataArr[ble->RxSize-2];
	CRCKey.byte.lo 	= ble->_RxDataArr[ble->RxSize-1];

	//recalcula chave em funcao da msg
	CRCReceive = CRC16(ble->_RxDataArr,ble->RxSize-2);

	if(validaCRC(CRCKey,CRCReceive)){
		//erro
		TXCRC[0] = 0x01;
		TXCRC[1] = 0xEE;
		TXCRC[2] = 0xEE;
		HAL_UART_Transmit(ble->UARTHandle, (uint8_t *)TXCRC, 3,50);
		return BLE_CRC_INCORRETO;
	}else{
		//certo
		return BLE_SUCCESS;
	}
}
/**
 * \brief 	Envia mensagens em buffer via HM-10 com CRC implementado ao fim da string.
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	_out[] - Buffer com mensagem a ser enviada.
 * \param 	size - Tamanho da mensagem original.
 * \return 	Retorna HAL status.
 */
HAL_StatusTypeDef bluetoothEnviaComando(Bluetooth *ble,unsigned char _out[], int size)
{
	uint8_t	TX_Buffer		[size+3];
	CRC_short CRCVar;

	//varredura para local.
	for (int i = 0; i <= size; ++i) {
		TX_Buffer[i]=_out[i];
	}

	//calculo e atribuicao do crc.
	CRCVar = CRC16(_out,size+1);
	TX_Buffer[size+2] = (unsigned char) CRCVar.byte.lo;
	TX_Buffer[size+1] = (unsigned char) CRCVar.byte.hi;

	return HAL_UART_Transmit(ble->UARTHandle, (uint8_t *)TX_Buffer, size+3,50);
}
/**
 * \brief 	Processa contadores internos a passo de 10 milisegundos
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			bluetooth10ms(Bluetooth* ble){

	/*INCREMENTO DE INATIVIDADE-------------------*/
	(ble->msIdle<=DEF_TEMPO_MAX_S_MSG_LOW)?ble->msIdle++:0;

	/*MONITOR INATIVIDADE-------------------------*/
	if(ble->JanelaConexao>0){
		if(ble->msIdle > DEF_TEMPO_MAX_S_MSG_HIGH)	{
			bluetoothDescon(ble);
		}
	}
	else{
		__NOP();
	}

	if(ble->msIdle > DEF_TEMPO_MAX_S_MSG_LOW)	{
		bluetoothDescon(ble);
	}
}
/**
 * \brief 	Processa contadores internos a passo de 1000 milisegundos
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			bluetooth1000ms(Bluetooth* ble){
	if(ble->JanelaConexao>0)
		ble->JanelaConexao--;
}
/**
 * \brief 	Processa interrupcoes internas
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			BLEUSART_IrqHandler(Bluetooth *ble)
{ //todo comentar melhor
	if (ble->UARTHandle->Instance->SR & UART_FLAG_IDLE) {    /* if Idle flag is set */
		__IO uint32_t __attribute__((unused))tmp;      	/* Must be volatile to prevent optimizations */

		tmp = ble->UARTHandle->Instance->SR;                 /* Read status register */
		tmp = ble->UARTHandle->Instance->DR;                 /* Read data register */
		__HAL_DMA_DISABLE (ble->UARTDMAHandle);       		/* Disabling DMA will force transfer complete interrupt if enabled */

		__HAL_UART_ENABLE_IT 	(ble->UARTHandle, UART_IT_IDLE);		// HABILITA idle line INTERRUPT
		__HAL_DMA_ENABLE_IT 	(ble->UARTDMAHandle, DMA_IT_TC);		// HABILITA O DMA Tx cplt INTERRUPT

		BLEDMA_IrqHandler (ble);
	}
}
/**
 * \brief 	Funcao exclusiva para envio de comandos ao HM10.
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	_out[] - Buffer com mensagem a ser enviada.
 * \param 	delay - Delay requirido via datasheet.
 */
void 			comandHM10(Bluetooth *ble, char _out[], uint16_t delay){
	HAL_UART_Transmit(ble->UARTHandle, (uint8_t *) _out, strlen(_out),100);
	if(delay != 0){
		osDelay(delay);
	}
}
/**
 * \brief 	Desconecta qualquer dispositivo conectado de forma apropriada
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			bluetoothDescon(Bluetooth* ble){
	comandHM10(ble,"AT",50);//DESCONECTA
	comandHM10(ble,"AT",50);//DESCONECTA
}
/**
 * \brief 	Simples envio de aknowladge indexado a comando
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	Cmd - comando que esta enviando o Aknowladge.
 */
void 			sendAknowladge(Bluetooth* ble,uint8_t Cmd){
	unsigned char	TXCRC[3];
	TXCRC[0] = 0x01;
	TXCRC[1] = 0xFF;
	TXCRC[2] = Cmd;
	HAL_UART_Transmit(ble->UARTHandle, (uint8_t *)TXCRC, 3,50);
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void 			bleError_Handler(BLE_ErrorCode erro)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

