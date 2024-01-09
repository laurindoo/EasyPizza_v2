/*
 * Bluetooth.c
 *
 *  Created on: Jul 24, 2023
 *      Author: lucas
 */

#include "Bluetooth.h"
#include "Eeprom.h"

extern Eeprom eeprom;
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
void	 		bleConstrutora(volatile Bluetooth *ble, UART_HandleTypeDef *UARTHandle, DMA_HandleTypeDef *UARTDMAHandle, osThreadId Task){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL || UARTHandle == NULL || UARTDMAHandle == NULL)
		bleError_Handler(BLE_OBJETO_NULO);

	// Inicialização das variáveis do objeto volatile Bluetooth.
	ble->UARTHandle 	= UARTHandle;
	ble->UARTDMAHandle 	= UARTDMAHandle;
	ble->contComandos 	= 0; 	// Renomeado para ter um nome mais explícito e sem _ inicial.
	ble->JanelaConexao 	= 120; 	// 120 segundos.

	//filas de dados
	ble->myQ_bleCom = Queue_create();
	ble->myQ_dataRx = Queue_create();
	ble->myQ_dataTx = Queue_create();
	ble->Task 		= Task;
	osSignalSet(ble->Task, newMessage);

	// Inicialização de objetos de conexão.
	bleAddCompConexao(ble, &ble->BLEPedeSenha, RX_PEDE_SENHA);
	bleAddCompConexao(ble, &ble->BLERecebeuSenha, RX_RECEBEU_SENHA);

	// Inicialização do hardware.
	iniciaBleHm10(ble);

	//metodo de respostas.
	ble->aknowladge	= sendAknowladge;
}

/**
 * \brief 	Adiciona componentes do tipo ComandoBasico no objeto
 * \param 	*ble - Ponteiro para o objeto pai
 * \param 	*_blecomm - Ponteiro do comando a ser cadastrado, pré-criado
 * \param 	__comando - Comando definido em typedef geralmente no header da tarefa que usará
 * \return 	Retorna BLE_SUCCESS em caso de contrucao bem sucedida
 *         	Retorna OBJETO_NULO caso objeto nao criado.
 *         	Retorna BLE_EXCEDEU_LIMITE caso array esteja fora do range definido.
 */
void 			bleAddComp(volatile Bluetooth* ble, BleComando* _blecomm, ComandosBleRX __comando){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL || _blecomm == NULL || __comando == 0) {
		bleError_Handler(BLE_OBJETO_NULO);
	}

	// Verifica tamanho.
	if(ble->contComandos>=BLUETOOTH_MAX_COMANDOS_COUNT){
		bleError_Handler(BLE_EXCEDEU_LIMITE);
	}

	// Pass the corresponding data from component to component struct.
	_blecomm->_comando 	= __comando;
	_blecomm->_tipo 	= ComandoBasico;

	// Add the component struct to the list on the Nextion Struct.
	ble->_BleCommArr[ble->contComandos] = _blecomm;
	ble->contComandos++;
}
BleComando* 	createBleComp(volatile Bluetooth* ble, ComandosBleRX __comando) {
	BleComando *me = (BleComando*)malloc(sizeof(BleComando));
	if (me == NULL) {
		return NULL;
	}
	bleAddComp(ble, me, __comando);
	return me;
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
void 			bleAddCompConexao(volatile Bluetooth* ble, volatile BleComando* _blecomm, ConexaoBleTX __comando){

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL || _blecomm == NULL || __comando == 0) {
		bleError_Handler(BLE_OBJETO_NULO);
	}

	// Verifica tamanho.
	if(ble->contComandos>=BLUETOOTH_MAX_COMANDOS_COUNT){
		bleError_Handler(BLE_COM_EXCEDEU_LIMITE);
	}

	// Pass the corresponding data from component to component struct.
	_blecomm->_comando 	= __comando;
	_blecomm->_tipo 	= ComandoConexao;

	// Add the component struct to the list on the Nextion Struct.
	ble->_BleCommArr[ble->contComandos] = _blecomm;
	ble->contComandos++;
}

/**
 * \brief 	Função para manipular interrupções do DMA no módulo volatile Bluetooth.
 * 			Esta função é chamada quando um novo conjunto de dados é recebido via DMA.
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			BLEDMA_IrqHandler (volatile Bluetooth *ble){
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
		__HAL_DMA_CLEAR_FLAG(ble->UARTDMAHandle, __HAL_DMA_GET_GI_FLAG_INDEX(ble->UARTDMAHandle));

		// Calculo do tamanho da string recebida.
		ble->RxSize 		= DMA_RX_BUFFER_SIZE - ble->UARTDMAHandle->Instance->CNDTR;

		// Copia os dados do buffer de DMA para a StringRecebida para processamento.
		sprintf(StringRecebida,"%s",ble->_RxDataArr);

		// Processamento da máquina de estados da conexão.
		switch(ble->MaquinaConexao){
		case RX_HARDWARE_INIT:
			ble->msIdle=0;

			// recebeu endereco MAC?
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

			// conectou?
			ss = NULL;
			ss = strstr(StringRecebida, "OK+CONN");
			if ((ss != NULL  && ble->RxSize == TAMANHO_MENSAGEM_CONECTOU) || MACRO_LE_BT_STATUS){
				ble->MaquinaConexao = RX_CONECTADO;
			}

			break;
		case RX_CONECTADO:

			// desconectou?
			ss = NULL;
			ss = strstr(StringRecebida, "LOST");
			if (ss != NULL || !MACRO_LE_BT_STATUS){
				ble->MaquinaConexao 	= RX_DESCONECTADO;
				break;
			}

			// entao le comando conexao.
			readComando(ble,ComandoConexao);
			break;
		case RX_VALIDADO:
			// desconectou?
			ss = NULL;
			ss = strstr(StringRecebida, "LOST");
			if (ss != NULL || !MACRO_LE_BT_STATUS){
				ble->MaquinaConexao 	= RX_DESCONECTADO;
				break;
			}

			// entao le comando basico.
			readComando(ble,ComandoBasico);
			break;
		}
		// Prepara o DMA para a próxima transferência.
		/* Important! DMA stream won't start if all flags are not cleared first */
		ble->UARTDMAHandle->Instance->CMAR 	= (uint32_t)ble->_RxDataArr; 	/* Set memory address for DMA again */
		ble->UARTDMAHandle->Instance->CNDTR = DMA_RX_BUFFER_SIZE;    		/* Set number of bytes to receive */
		ble->UARTDMAHandle->Instance->CCR |= DMA_CCR_EN;            		/* Start DMA transfer */

	}
}

/**
 * \brief 	Realiza a extracao do comando do array RxDataArr e vincula ao ComandoAtual
 * 				de acordo com o tipo de comando aceitavel: ComandoBasico OU ComandoConexao.
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	tipo - tipo de comando aceitavel: ComandoBasico OU ComandoConexao.
 * \note: 	Se o comando for encontrado, o mesmo é vinculado ao ComandoAtual.
 *             Caso contrário, é enviado um erro de CRC.
 */
void		 	readComando(volatile Bluetooth* ble, TypeComandoBle tipo){
	BleComando localComandoRX;

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL) {
		bleError_Handler(BLE_OBJETO_NULO);
	}

	// Validação de CRC.
	if(bluetoothErroCRC(ble)) {
		return;
	}

	// Varredura pelo array de comandos.
	for(uint8_t i = 0; i < ble->contComandos; i++)	{

		// captura comando.
		localComandoRX = *ble->_BleCommArr[i];

		// identifica comando no array e casa com o tipo.
		if( ble->_RxDataArr[1] == localComandoRX._comando && localComandoRX._tipo == tipo){
			ble->ComandoAtual = localComandoRX;
			switch (tipo) {
			case ComandoBasico:
				putQueueDataRx(ble, ble->ComandoAtual._comando);
				ble->msIdle=0;
				break;
			case ComandoConexao:
				putQueueComando(ble, ble->ComandoAtual._comando);
				ble->msIdle=0;
				break;
			}
			return;
		}
	}
	//se chegou aqui é porque comando nao foi encontrado. envia erro de CRC nao encontrado.
	putQueueComando(ble, TX_ERRO_CRC);
}

/**
 * \brief 	Processa solicitacoes de senha e entradas ao sistema
 * \param 	*ble - Ponteiro para o objeto pai.
 * \note:     Se o comando for encontrado, o mesmo é vinculado ao ComandoAtual.
 *            Caso contrário, é enviado um erro de CRC.
 */
void 			txBleComando(volatile Bluetooth *ble){
	unsigned char	Buffer		[10];
	int 			buffQueue;

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL)
		bleError_Handler(BLE_OBJETO_NULO);

	// Verifica se a fila está vazia.
	if (ble->myQ_bleCom->is_empty(ble->myQ_bleCom))
		return;

	// Remove o primeiro elemento da fila.
	buffQueue = ble->myQ_bleCom->remove(ble->myQ_bleCom);
	if (buffQueue > 0) {
		switch ((uint8_t)buffQueue) {
		case RX_PEDE_SENHA:
			if(ble->JanelaConexao > 0){
				/*----DENTRO DO TEMPO, ENTAO RESPONDE----*/
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x51;
				Buffer[2] 	= 0x51;
				Buffer[3] 	= 0x01;
				Buffer[4] 	= ble->chave.byte.hi	;
				Buffer[5] 	= ble->chave.byte.lo	;
				bluetoothEnviaComando(ble,Buffer, 5);
				return;
			}else{
				/*----FORA DO TEMPO DE RESPOSTA ----*/
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x51;
				Buffer[2] 	= 0x51;
				Buffer[3] 	= 0x00;
				Buffer[4] 	= 0x00;
				Buffer[5] 	= 0x00;
				bluetoothEnviaComando(ble,Buffer, 5);
			}
			break;
		case RX_RECEBEU_SENHA:
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
			}else{
				//--->	CHAVE CORRETA
				ble->MaquinaConexao	= RX_VALIDADO;
				Buffer[0] 	= 0x01;
				Buffer[1] 	= 0x52;
				Buffer[2] 	= 0x52;
				Buffer[3] 	= 0x01;
				bluetoothEnviaComando(ble,Buffer, 3);
				return;
			}
			break;
		case TX_DESCONECTA:
			comandHM10(ble,"AT",50);//DESCONECTA
			comandHM10(ble,"AT",50);//DESCONECTA
			break;
		case TX_AKNOWLADGE:
			Buffer[0] = 0x01;
			Buffer[1] = 0xFF;
			Buffer[2] = ble->ComandoAtual._comando;
			bluetoothEnviaComandoSCRC(ble,Buffer, 2);
			break;
		case TX_ERRO_CRC:
		default:
			Buffer[0] = 0x01;
			Buffer[1] = 0xEE;
			Buffer[2] = 0xEE;
			bluetoothEnviaComandoSCRC(ble,Buffer, 2);
			break;
		}
	}
	return;
}

/**
 * \brief 	Inicializa o hardware com sequencia de comandos
 * \param 	*ble - Ponteiro para o objeto pai.
 * \return 	Retorna BLE_SUCCESS em caso de hardware inicializado com sucesso.
 *         	Retorna BLE_EXTRAPOLOU_TRY caso excedido tentativas de capturar MAC addres.
 * \info*  	Prestar atencao nas configuracoes do header bluetooth.h
 */
void		 	iniciaBleHm10(volatile Bluetooth* ble){
	const uint8_t 		max_attempts = 15;
	static sequenciaBle sequenciaBLE = inicio;
	static uint8_t 		tryingAddr=0;

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL)
		bleError_Handler(BLE_OBJETO_NULO);

	while(sequenciaBLE !=final || sequenciaBLE!=erro){
		switch (sequenciaBLE) {
		case inicio:
			MACRO_RESET_BLE						// hm10 reset via hardware.
			comandHM10(ble,"AT+ADTY3"	,100);	// hm10 bloqueia conexao.
			comandHM10(ble,"AT+ADTY3"	,100);	// hm10 bloqueia conexao.
			SETUP_UART(115200)					// stm32 baudrate em 115200.
			bluetoothDescon(ble);				// stm32 desconecta hm10.
			sequenciaBLE = redefineBle;			// proxima sequencia.
			break;
		case redefineBle:
			SETUP_UART(115200)					// stm32 seta baudrate em 115200.
			comandHM10(ble,"AT"			,100);	// hm10 envia comando AT.
			comandHM10(ble,"AT"			,100);	// hm10 envia comando AT.
			comandHM10(ble,"AT+RENEW"	,1000); // hm10 restaura padroes de fabrica.
			SETUP_UART(9600)					// stm32 seta baudrate em 9600
			comandHM10(ble,"AT+RENEW"	,1000); // hm10 restaura padroes de fabrica.
			comandHM10(ble,"AT"			,100);	// hm10 envia comando AT.
			comandHM10(ble,"AT"			,100);	// hm10 envia comando AT.
			comandHM10(ble,"AT+ADTY3"	,300);	// hm10 bloqueia conexao.
			comandHM10(ble,"AT+BAUD4"	,300);	// hm10 define baudrate 115200.
			SETUP_UART(115200)					// stm32 baudrate em 115200.
			MACRO_RESET_BLE						// hm10 reset via hardware.
			//-------------------configuracao final----------------------------
			comandHM10(ble,"AT"			,100);	// hm10 envia comando AT.
			comandHM10(ble,"AT"			,100);	// hm10 envia comando AT.
			comandHM10(ble,"AT+POWE3"	,300);	// hm10 em potencia maxima.
			comandHM10(ble,"AT+SHOW3"	,300);	// hm10 mostra nome e rssi.
			comandHM10(ble,"AT+GAIN1"	,300);	// hm10 ganho de antena maximo.
			comandHM10(ble,"AT+NOTI1"	,300);	// hm10 notifica quando conectado.
			comandHM10(ble,"AT+PIO11"	,300);	// 1 - CONECT = 1  \  DISC = 0.
			char comando[COMANDO_BUFFER_SIZE]; 	// buffer para o comando AT.
			snprintf(comando, sizeof(comando), "AT+NAME%s", BLE_DEVICE_NAME);
			comandHM10(ble,comando, 400); 		// stm32 seta nome do hm10.
			M_BLE_RESET							// hm10 reset via software.
			sequenciaBLE = capturaAddr;			// proxima sequencia.
			break;
		case capturaAddr:
			MACRO_DEFINE_INTERRUPT
			while (tryingAddr < max_attempts) {

				comandHM10(ble,"AT+ADDR?",300); // stm32 pede addr do hm10.
				MACRO_DEFINE_INTERRUPT
				if (ble->chave.hilo != 0){
					sequenciaBLE = final;	// capturou o addr e vai para a proxima sequencia.
					tryingAddr=0;
					break;
				} else {
					tryingAddr++;
					break;
				}
			}
			if(tryingAddr >= max_attempts){
				sequenciaBLE = erro;	// extrapolou as tentativas.
			}
			break;
		case final:
			comandHM10(ble,"AT+ADTY0",300);	// hm10 desbloqueia conexao.
			MACRO_RESET_BLE
			MACRO_DEFINE_INTERRUPT
			ble->MaquinaConexao = RX_DESCONECTADO;
			return;
			break;
		case erro:
			bleError_Handler(BLE_EXTRAPOLOU_TRY);
			break;
		}
	}
}

/**
 * \brief 	Processa informacao recebida com chave apropriada
 * \param 	*ble - Ponteiro para o objeto pai.
 */
bool 			bluetoothErroCRC(volatile Bluetooth* ble){
	CRC_short 		CRCReceive,CRCKey;

	// Caso algum ponteiro seja nulo, retorne código de erro correspondente.
	if (ble == NULL){
		bleError_Handler(BLE_OBJETO_NULO);
		return 1;
	}

	// garante que o tamanho da mensagem seja coerente.
	if (ble->RxSize < 4) {
		putQueueComando(ble, TX_ERRO_CRC);
		return 1;
	}

	// captura chave recebida pelo App.
	CRCKey.byte.hi	= ble->_RxDataArr[ble->RxSize-2];
	CRCKey.byte.lo 	= ble->_RxDataArr[ble->RxSize-1];

	// recalcula chave em funcao da msg.
	CRCReceive = CRC16(ble->_RxDataArr,ble->RxSize-2);

	// compara os crc.
	if(validaCRC(CRCKey,CRCReceive)){
		putQueueComando(ble, TX_ERRO_CRC); // erro de crc.
		return 1;
	}

	return 0;
}

/**
 * \brief 	Envia mensagens em buffer via HM-10 com CRC implementado ao fim da string.
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	_out[] - Buffer com mensagem a ser enviada.
 * \param 	size - Tamanho da mensagem original.
 */
void		 	bluetoothEnviaComando(volatile Bluetooth *ble,unsigned char _out[], int size)
{
	uint8_t	TX_Buffer[size+3];
	HAL_StatusTypeDef result;
	uint8_t tries = 0, MAX_TRIES = 10;
	CRC_short CRCVar;

	// caso algum ponteiro seja nulo, retorne codigo de erro correspondente.
	if (ble == NULL)
		bleError_Handler(BLE_OBJETO_NULO);

	//varredura para local.
	for (int i = 0; i <= size; ++i) {
		TX_Buffer[i]=_out[i];
	}

	//calculo e atribuicao do crc.
	CRCVar = CRC16(_out,size+1);
	TX_Buffer[size+2] = (unsigned char) CRCVar.byte.lo;
	TX_Buffer[size+1] = (unsigned char) CRCVar.byte.hi;

	while (tries++ < MAX_TRIES) {
		result = HAL_UART_Transmit(ble->UARTHandle, (uint8_t *)TX_Buffer, size+3,50);
		if (result == HAL_OK)	break;
	}
}

/**
 * \brief 	Envia mensagens em buffer via HM-10 .
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	_out[] - Buffer com mensagem a ser enviada.
 * \param 	size - Tamanho da mensagem original.
 */
void		 	bluetoothEnviaComandoSCRC(volatile Bluetooth *ble,unsigned char _out[], int size){
	HAL_StatusTypeDef result;
	uint8_t tries = 0, MAX_TRIES = 10;

	// caso algum ponteiro seja nulo, retorne codigo de erro correspondente.
	if (ble == NULL)
		bleError_Handler(BLE_OBJETO_NULO);

	while (tries++ < MAX_TRIES) {
		result = HAL_UART_Transmit(ble->UARTHandle, (uint8_t *)_out, size,50);
		if (result == HAL_OK)	break;
	}
}

/**
 * \brief 	Processa contadores internos a passo de 100 milisegundos
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			bluetoothActivitymonitor(volatile Bluetooth* ble){

	/*INCREMENTO DE INATIVIDADE-------------------*/
	(ble->msIdle<=DEF_TEMPO_MAX_S_MSG_HIGH)?ble->msIdle++:0;

	/*MONITOR INATIVIDADE-------------------------*/
	if(ble->JanelaConexao>0 && (ble->MaquinaConexao == RX_CONECTADO || ble->MaquinaConexao == RX_VALIDADO)){
		if(ble->msIdle > DEF_TEMPO_MAX_S_MSG_HIGH)
			putQueueComando(ble, TX_DESCONECTA);
	}
	else if(ble->MaquinaConexao == RX_CONECTADO || ble->MaquinaConexao == RX_VALIDADO){
		if(ble->msIdle > DEF_TEMPO_MAX_S_MSG_LOW)
			putQueueComando(ble, TX_DESCONECTA);
	}
}

/**
 * \brief 	Processa contadores internos a passo de 1000 milisegundos
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			bluetooth1000ms(volatile Bluetooth* ble){

	// caso algum ponteiro seja nulo, retorna.
	if (ble == NULL)
		return;

	// decrementa janela de conexao.
	if(ble->JanelaConexao>0)
		ble->JanelaConexao--;
}

/**
 * \brief 	Processa interrupcoes internas
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			BLEUSART_IrqHandler(volatile Bluetooth *ble){
	if (ble->UARTHandle->Instance->SR & UART_FLAG_IDLE) {    	/* if Idle flag is set */
		__IO uint32_t __attribute__((unused))tmp;      			/* Must be volatile to prevent optimizations */
		tmp = ble->UARTHandle->Instance->SR;                 	/* Read status register */
		tmp = ble->UARTHandle->Instance->DR;                 	/* Read data register */
		__HAL_DMA_DISABLE (ble->UARTDMAHandle);       			/* Disabling DMA will force transfer complete interrupt if enabled */
		BLEDMA_IrqHandler (ble);
	}
}

/**
 * \brief 	Funcao exclusiva para envio de comandos ao HM10.
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	_out[] - Buffer com mensagem a ser enviada.
 * \param 	delay - Delay requirido via datasheet.
 */
void 			comandHM10(volatile Bluetooth *ble, char _out[], uint16_t delay){
	HAL_StatusTypeDef result;
	uint8_t tries = 0, MAX_TRIES = 5;

	// caso algum ponteiro seja nulo, retorne codigo de erro correspondente.
	if (ble == NULL)
		bleError_Handler(BLE_OBJETO_NULO);

	while (tries++ < MAX_TRIES) {
		result = HAL_UART_Transmit(ble->UARTHandle, (uint8_t *) _out, strlen(_out),100);
		if (result == HAL_OK)	break;
	}

	if(delay != 0){
		osDelay(delay);
	}
}

/**
 * \brief 	Desconecta qualquer dispositivo conectado de forma apropriada
 * \param 	*ble - Ponteiro para o objeto pai.
 */
void 			bluetoothDescon(volatile Bluetooth* ble){
	comandHM10(ble,"AT",50);//DESCONECTA
	comandHM10(ble,"AT",50);//DESCONECTA
}

/**
 * \brief 	Simples envio de aknowladge indexado a comando
 * \param 	*ble - Ponteiro para o objeto pai.
 * \param 	Cmd - comando que esta enviando o Aknowladge.
 */
void 			sendAknowladge(volatile Bluetooth* ble,uint8_t Cmd){
	putQueueComando(ble, TX_AKNOWLADGE);
}
void putQueueComando(volatile Bluetooth *ble, ConexaoBleRX comando) {
	ble->myQ_bleCom->insert(ble->myQ_bleCom, comando);
	osSignalSet(ble->Task, newMessage);
}
void putQueueDataRx(volatile Bluetooth *ble, ComandosBleRX comando) {
	// permitir apenas um item por vez na fila. evitando dessincronia com dado recebida.
	if (ble->myQ_dataRx->is_empty(ble->myQ_dataRx)) {
		ble->myQ_dataRx->insert(ble->myQ_dataRx, comando);
		osSignalSet(ble->Task, newMessage);
	}
}
void putQueueDataTx(volatile Bluetooth *ble, ComandosBleTX comando) {
	ble->myQ_dataTx->insert(ble->myQ_dataTx, comando);
	osSignalSet(ble->Task, newMessage);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void 			bleError_Handler(ErrorCode erro)
{
	//	__disable_irq();
	while (1)
	{
		ErrorBuffer_add(&eeprom, erro);
		NVIC_SystemReset();
	}
}



















