/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include "Bluetooth.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId TaskBluetoothHandle;
osThreadId TaskTemperaturaHandle;
osThreadId TaskBuzzerHandle;
uint32_t TaskBuzzerBuffer[ 128 ];
osStaticThreadDef_t TaskBuzzerControlBlock;
osThreadId TaskTimerHandle;
uint32_t TaskTimerBuffer[ 128 ];
osStaticThreadDef_t TaskTimerControlBlock;
osThreadId TaskEepromHandle;
osMessageQId FilaComandoHandle;
osMessageQId FilaTXBluetoothHandle;
osMessageQId FilaRXBluetoothHandle;
osMessageQId FilaEepromHandle;
osTimerId timer10msHandle;
osTimerId timer1000msHandle;
osSemaphoreId BinSemUartTxHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
void StartBluetooth(void const * argument);
void StartTemperatura(void const * argument);
void StartBuzzer(void const * argument);
void StartTimer(void const * argument);
void StartEeprom(void const * argument);
void CBTimer10ms(void const * argument);
void CBTimer1000ms(void const * argument);
/* USER CODE BEGIN PFP */
extern void taskTemperatura1sec(void);
extern void taskBluetooth1sec(void);
void leTempInterna(void);
void controleCooler(void);
void timeoutAquecimento (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float tempInterna;

Bluetooth bluetooth;

//---ARRAY DE LEITURA DO AD
uint32_t 	buffer_ADC[3];

//---VARIAVEIS PRIMITIVAS
GlobalPrimitiveIOStates PrimitiveStates;

TYPE_CALENDARIO Calendario;


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/*observacoes aplicativo
	 * gradiente da barra de temperatura nao aparente
	 * falta de vibracao nos botoes
	 * campo de insercao de numeros inves de rolagem
	 * alfabeto na lista de receitas
	 * */

	/* LISTA TODO
	 *
	 * 1  - FEITO - CONFERIR - Contador ContTempMaxima. FEITO - CONFERIR
	 * 2  - FEITO - CONFERIR - Contador ciclos timer feitos CiclosFeitos.
	 * 3  - Efeito visual e sonoro initerrupto ao chegar na temp e terminar ciclo.
	 * 4  - FEITO - Criar funcao para saida dos leds.
	 * 5  - FEITO - Conferir funcao inatividade.
	 * 6  - FEITO - Contador de ciclos
	 * 7  - FEITO - CONFERIR - assegurar contagem de minutimetro e horimetro
	 * 8  - FEITO - implementar classe digitalOutPut
	 * 9  - incluir no aplicativo alteracoes, consultar taskBluetooth.c
	 * 10 - FEITO -  revisar a metodologia de envio de informacoes via serial, testar sem utilizar o _IT ps.:possivelmente retornar o osDelay
	 *
	 * */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA		(&hadc1	,(uint32_t*)&buffer_ADC, 3	);// ADC_DMA

	HAL_TIM_PWM_Start		(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start		(&htim3,TIM_CHANNEL_4);

	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.Lampada		,Digital	,RELE_5_Pin	,RELE_5_GPIO_Port	,0	,0	,0);
	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.Cooler		,Digital	,RELE_4_Pin	,RELE_4_GPIO_Port	,0	,0	,0);

	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.Cooler		,Digital	,RELE_4_Pin	,RELE_4_GPIO_Port	,0	,0	,0);
	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.pwmTeto		,PIDVal		,0			,0					,timeoutAquecimento	,TIME_MAX_AQUECIMENTO	,0);
	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.pwmLastro	,PIDVal		,0			,0					,timeoutAquecimento	,TIME_MAX_AQUECIMENTO	,0);

	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.LedLastro	,Digital	,RELE_3_Pin	,RELE_3_GPIO_Port	,0	,0	,0);
	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.LedTeto		,Digital	,RELE_2_Pin	,RELE_2_GPIO_Port	,0	,0	,0);
	OutputAddComp(&PrimitiveStates.outPuts,&PrimitiveStates.LedVerde	,Digital	,RELE_1_Pin	,RELE_1_GPIO_Port	,0	,0	,0);

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of BinSemUartTx */
	osSemaphoreDef(BinSemUartTx);
	BinSemUartTxHandle = osSemaphoreCreate(osSemaphore(BinSemUartTx), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* definition and creation of timer10ms */
	osTimerDef(timer10ms, CBTimer10ms);
	timer10msHandle = osTimerCreate(osTimer(timer10ms), osTimerPeriodic, NULL);

	/* definition and creation of timer1000ms */
	osTimerDef(timer1000ms, CBTimer1000ms);
	timer1000msHandle = osTimerCreate(osTimer(timer1000ms), osTimerPeriodic, NULL);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of FilaComando */
	osMessageQDef(FilaComando, 10, uint16_t);
	FilaComandoHandle = osMessageCreate(osMessageQ(FilaComando), NULL);

	/* definition and creation of FilaTXBluetooth */
	osMessageQDef(FilaTXBluetooth, 16, uint32_t);
	FilaTXBluetoothHandle = osMessageCreate(osMessageQ(FilaTXBluetooth), NULL);

	/* definition and creation of FilaRXBluetooth */
	osMessageQDef(FilaRXBluetooth, 10, uint8_t);
	FilaRXBluetoothHandle = osMessageCreate(osMessageQ(FilaRXBluetooth), NULL);

	/* definition and creation of FilaEeprom */
	osMessageQDef(FilaEeprom, 10, uint16_t);
	FilaEepromHandle = osMessageCreate(osMessageQ(FilaEeprom), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of TaskBluetooth */
	osThreadDef(TaskBluetooth, StartBluetooth, osPriorityHigh, 0, 128);
	TaskBluetoothHandle = osThreadCreate(osThread(TaskBluetooth), NULL);

	/* definition and creation of TaskTemperatura */
	osThreadDef(TaskTemperatura, StartTemperatura, osPriorityAboveNormal, 0, 128);
	TaskTemperaturaHandle = osThreadCreate(osThread(TaskTemperatura), NULL);

	/* definition and creation of TaskBuzzer */
	osThreadStaticDef(TaskBuzzer, StartBuzzer, osPriorityIdle, 0, 128, TaskBuzzerBuffer, &TaskBuzzerControlBlock);
	TaskBuzzerHandle = osThreadCreate(osThread(TaskBuzzer), NULL);

	/* definition and creation of TaskTimer */
	osThreadStaticDef(TaskTimer, StartTimer, osPriorityNormal, 0, 128, TaskTimerBuffer, &TaskTimerControlBlock);
	TaskTimerHandle = osThreadCreate(osThread(TaskTimer), NULL);

	/* definition and creation of TaskEeprom */
	osThreadDef(TaskEeprom, StartEeprom, osPriorityNormal, 0, 128);
	TaskEepromHandle = osThreadCreate(osThread(TaskEeprom), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	osTimerStart(timer10msHandle,10);
	osTimerStart(timer1000msHandle,1000);
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1200-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 50;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4000-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BLE_EN_Pin|BLE_RESET_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RELE_1_GPIO_Port, RELE_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RELE_2_Pin|RELE_3_Pin|RELE_4_Pin|RELE_5_Pin
			|EEPROM_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : BLE_EN_Pin BLE_RESET_Pin RELE_2_Pin RELE_3_Pin
                           RELE_4_Pin RELE_5_Pin */
	GPIO_InitStruct.Pin = BLE_EN_Pin|BLE_RESET_Pin|RELE_2_Pin|RELE_3_Pin
			|RELE_4_Pin|RELE_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BLE_STATUS_Pin */
	GPIO_InitStruct.Pin = BLE_STATUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLE_STATUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOTAO_BLE_Pin */
	GPIO_InitStruct.Pin = BOTAO_BLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOTAO_BLE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RELE_1_Pin */
	GPIO_InitStruct.Pin = RELE_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RELE_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : EEPROM_EN_Pin */
	GPIO_InitStruct.Pin = EEPROM_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EEPROM_EN_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void desligaForno(void){
	PrimitiveStates.RTTimerMinutos 	= 0;
	PrimitiveStates.RTTimerSegundos = 0;
	PrimitiveStates.SetPointLastro 	= 0;
	PrimitiveStates.SetPointTeto 	= 0;
	PrimitiveStates.stateTimer 		= TIMER_idle;
}

void leTempInterna(void){
#define Avg_slope .0043
#define V25_	1.43
#define VSENSE 3.3/4096 //12bit

	tempInterna = ((V25_ - VSENSE*buffer_ADC[2])/Avg_slope)+25;
}

void controleCooler(void){
	if(PrimitiveStates.RealtimeLastro>200 || PrimitiveStates.RealtimeTeto>200){
		onOutput(&PrimitiveStates.Cooler);
	}else 	if(PrimitiveStates.RealtimeLastro<195 && PrimitiveStates.RealtimeTeto<195){
		offOutput(&PrimitiveStates.Cooler);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#define TAM 500
#define AVG_SLOPE 	4.3 // mV per C (internal temp)
#define V25 		1.43 // V	(internal temp)

	float 	Temp1,Temp2;
	double TempTeto, TempLastro;
	static long somatorio1,somatorio2;
	static uint16_t i = 0;


	leTempInterna();

	if(i<TAM){
		somatorio1+=buffer_ADC[1]; // somatorio
		somatorio2+=buffer_ADC[0]; // somatorio
		i++;
	}else{
		somatorio1 = somatorio1 / TAM; //calc media
		somatorio2 = somatorio2 / TAM; //calc media

		Temp1 = somatorio1*16; //calc valor final
		Temp1 = Temp1/100;
		somatorio1 = 0; 		//zera somatorio

		Temp2 = somatorio2*16; 	//calc valor final
		Temp2 = Temp2/100;
		somatorio2 = 0; 		//zera somatorio

		TempLastro=(double)Temp1;
		TempTeto=(double)Temp2;

		PrimitiveStates.RealtimeTeto = TempTeto;
		PrimitiveStates.RealtimeLastro = TempLastro;
		i=0;
	}


	//	//	TempInterna
	//	TempInterna = (   (buffer_ADC[2]*(3.3/4095)-V25)   /   (AVG_SLOPE*1000) )         + 35;
	//
	//	// Handles the IRQ of ADC1. EOC flag is cleared by reading data register
	//	static uint32_t temp = 0;
	//	temp = ADC1->DR;
	//	TempInterna = (temp-V25)/(AVG_SLOPE)+25;

}
void timeoutAquecimento (void){
	//verifica erro temperatura lastro
	if(PrimitiveStates.RealtimeLastro < PrimitiveStates.SetPointLastro-5)
		PrimitiveStates.Erro.bit.IdleLastro=1;

	//verifica erro temperatura teto
	if(PrimitiveStates.RealtimeTeto < PrimitiveStates.SetPointTeto-5)
		PrimitiveStates.Erro.bit.IdleTeto=1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBluetooth */
/**
 * @brief  Function implementing the TaskBluetooth thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBluetooth */
__weak void StartBluetooth(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTemperatura */
/**
 * @brief Function implementing the TaskTemperatura thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTemperatura */
__weak void StartTemperatura(void const * argument)
{
	/* USER CODE BEGIN StartTemperatura */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartTemperatura */
}

/* USER CODE BEGIN Header_StartBuzzer */
/**
 * @brief Function implementing the TaskBuzzer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBuzzer */
__weak void StartBuzzer(void const * argument)
{
	/* USER CODE BEGIN StartBuzzer */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartBuzzer */
}

/* USER CODE BEGIN Header_StartTimer */
/**
 * @brief Function implementing the TaskTimer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTimer */
__weak void StartTimer(void const * argument)
{
	/* USER CODE BEGIN StartTimer */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartTimer */
}

/* USER CODE BEGIN Header_StartEeprom */
/**
 * @brief Function implementing the TaskEeprom thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEeprom */
__weak void StartEeprom(void const * argument)
{
	/* USER CODE BEGIN StartEeprom */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartEeprom */
}

/* CBTimer10ms function */
void CBTimer10ms(void const * argument)
{
	/* USER CODE BEGIN CBTimer10ms */

	bluetooth10ms(&bluetooth);

	/* USER CODE END CBTimer10ms */
}

/* CBTimer1000ms function */
void CBTimer1000ms(void const * argument)
{
	/* USER CODE BEGIN CBTimer1000ms */

	bluetooth1000ms(&bluetooth);

	taskTemperatura1sec();

	taskBluetooth1sec();

	controleCooler();

	//contadores de todas as saidas
	contadorOutput(&PrimitiveStates.outPuts);

	//se existir erros do tipo abaixo, o forno é desligado
	if(PrimitiveStates.Erro.byte != 0){
		desligaForno();
	}
	/* USER CODE END CBTimer1000ms */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
