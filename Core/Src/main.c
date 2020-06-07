/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x_proximity.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_device.h"

#include "stm32l4xx_hal.h"

#include "vl53l0x_api_ranging.h"
#include "vl53l0x_api.h"
#include "stdio.h"
#include "math.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01.h"
#include "tof_gestures_DIRSWIPE_1.h"
#include "tof_gestures_platform.h"
#include "tof_gestures_SWIPE_1.h"
#include "tof_gestures_TAP_1.h"
#include "tof_gestures_types.h"
#include "tof_gestures.h"
#include "tof_motion.h"

#include "message.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define tempMax 28
#define nTempValid 5
#define distMax 100
#define distMin 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId TaskBottoneHandle;
osThreadId TaskTemperaturaHandle;
osThreadId TaskDistanzaHandle;
osThreadId TaskLedHandle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osMessageQId myQueue03Handle;
osMutexId myMutex01Handle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myBinarySem02Handle;
osSemaphoreId myBinarySem03Handle;
/* USER CODE BEGIN PV */
char str_tmp2[30];
char str_tmp4[30];
char str_tmp5[30];
char str_tmp7[100];
char str_tmp8[100];
char str_tmp9[100];
char str_tmp10[100];
char str_tmp11[100];
char str_tmp12[100];
char str_tmp13[100];
char str_tmp14[100];
char str1[100];
char str2[100];
char str3[100];
char str4[100];
char linea[100]="----------------------------------------\n\r";
int delay = 1500;
int messaggioInviato = 0;
int messaggioTrasmesso = 0;
uint8_t temp_value=0;
int nTempBasse=0;
int nTempAlte=0;
int b2=1;
int b3=1;
osEvent distanzaRicevuta;
osEvent tempRicevuta;
osEvent statoPulsanteRicevuto;
uint32_t bitstatus=0;
int pulsante = 0;
int incendio = 0;
int action = 0;


uint32_t time1=0;
uint32_t time2=0;
uint32_t time3=0;
uint32_t time4=0;

typedef struct{
	uint16_t Distance;
	int Gesture;
} Data;

typedef struct{
	uint16_t tempValue;
	int id;
} Temp;

Gesture_SWIPE_1_Data_t gestureSwipeData;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartTaskBottone(void const * argument);
void StartTaskTemperatura(void const * argument);
void StartTaskDistanza(void const * argument);
void StartTaskLed(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  VL53L0X_PROXIMITY_Init();
  BSP_TSENSOR_Init();
  tof_gestures_initSWIPE_1(&gestureSwipeData);
  /* USER CODE END 2 */
  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
   /* definition and creation of myBinarySem02 */
  osSemaphoreDef(myBinarySem02);
  myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

  /* definition and creation of myBinarySem03 */
  osSemaphoreDef(myBinarySem03);
  myBinarySem03Handle = osSemaphoreCreate(osSemaphore(myBinarySem03), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 256, Data);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* definition and creation of myQueue02 */
  osMessageQDef(myQueue02, 64, Temp);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);

  /* definition and creation of myQueue03 */
  osMessageQDef(myQueue03, 64, Temp);
  myQueue03Handle = osMessageCreate(osMessageQ(myQueue03), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TaskBottone */
  osThreadDef(TaskBottone, StartTaskBottone, osPriorityHigh, 0, 128);
  TaskBottoneHandle = osThreadCreate(osThread(TaskBottone), NULL);

  /* definition and creation of TaskTemperatura */
  osThreadDef(TaskTemperatura, StartTaskTemperatura, osPriorityNormal, 0, 128);
  TaskTemperaturaHandle = osThreadCreate(osThread(TaskTemperatura), NULL);

  /* definition and creation of TaskDistanza */
  osThreadDef(TaskDistanza, StartTaskDistanza, osPriorityNormal, 0, 512);
  TaskDistanzaHandle = osThreadCreate(osThread(TaskDistanza), NULL);

  /* definition and creation of TaskLed */
  osThreadDef(TaskLed, StartTaskLed, osPriorityNormal, 0, 128);
  TaskLedHandle = osThreadCreate(osThread(TaskLed), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin 
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin 
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin 
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin 
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin 
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin 
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin 
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskBottone */
/**
  * @brief  Function implementing the TaskBottone thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTaskBottone */
void StartTaskBottone(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
			  osDelay(150);
			  uint32_t inizio1 = HAL_GetTick();
			  osSemaphoreWait(myMutex01Handle,osWaitForever);
	  	  	  snprintf(str_tmp7,100," Il task default e' arrivato = %ld\n\r", inizio1);
	  	  	  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp7,sizeof(str_tmp7),HAL_MAX_DELAY);
	  	  	  uint32_t start = HAL_GetTick();
	  	  	  snprintf(str1,100," Il task default ha iniziato = %ld\n\r", start);
	  	  	  HAL_UART_Transmit(&huart1,( uint8_t * )str1,sizeof(str1),HAL_MAX_DELAY);

	  	  	  bitstatus = HAL_GPIO_ReadPin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin);

	  	  	  //temperature basse, premo il pulsante -> il sistema si attiva
	  	  	  if(bitstatus == 0 && pulsante == 0 && incendio==0){
	  	  		  if(b2>0){
	  	  			b2=0;
	  	  			osSemaphoreRelease(myBinarySem02Handle);
	  	  		  }
	  	  		  pulsante = 1;
	  	  		  action = 0;
	  	  		  nTempBasse=0;
	  	  		  nTempAlte = nTempValid+1;
	  	  		  delay = 500;
	  	  		  Temp DataToSend = {1,0};
	  	  		  osMessagePut(myQueue03Handle,(uint32_t)&DataToSend,200);
	  	  		  HAL_UART_Transmit(&huart1,( uint8_t * )msg_attivato,sizeof(msg_attivato),HAL_MAX_DELAY);
	  	  	  }

	  	  	  // premo il pulsante -> il sistema si disattiva indipendentemente dalle temperature
	  	  	  else if (bitstatus == 0){
	  	  		 b2=1;
	  	  		 pulsante = 0;
	  	  		 incendio=0;
	  	  		 nTempAlte=0;
	  	  		 nTempBasse = 0;
	  	  		 action = 0;
	  	  		 delay = 1500;
	  	  		 Temp DataToSend = {0,0};
	  	  		 osMessagePut(myQueue03Handle,(uint32_t)&DataToSend,200);
	  	  		 HAL_UART_Transmit(&huart1,( uint8_t * )msg_disattivato,sizeof(msg_disattivato),HAL_MAX_DELAY);
	  	  	  }

			   uint32_t end = HAL_GetTick();
	  		   time1 = end - start;
	  		   snprintf(str_tmp11,100," WCET default = %ld\n\r",time1);
	  		   HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp11,sizeof(str_tmp11),HAL_MAX_DELAY);
	  		   HAL_UART_Transmit(&huart1,( uint8_t * )linea,sizeof(linea),HAL_MAX_DELAY);
	  	       osSemaphoreRelease(myMutex01Handle);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTaskTemperatura */
/**
* @brief Function implementing the TaskTemperatura thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTemperatura */
void StartTaskTemperatura(void const * argument)
{
  /* USER CODE BEGIN StartTaskTemperatura */
  /* Infinite loop */
	  for(;;)
	  {
		  osDelay(delay);
		  uint32_t inizio2 = HAL_GetTick();
		  osSemaphoreWait(myMutex01Handle,osWaitForever);
		  snprintf(str_tmp8,100," Task 1 e' arrivato  = %ld\n\r", inizio2);
		  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp8,sizeof(str_tmp8),HAL_MAX_DELAY);
		  uint32_t start = HAL_GetTick();
		  snprintf(str2,100," Il task 1 ha iniziato = %ld\n\r", start);
		  HAL_UART_Transmit(&huart1,( uint8_t * )str2,sizeof(str2),HAL_MAX_DELAY);
		  
		  //accendo il led verde quando la situazione è normale
		  if(nTempAlte == 0 && nTempBasse == 0){
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin,  GPIO_PIN_SET);
		  }

		   temp_value = BSP_TSENSOR_ReadTemp();
		   if(temp_value > tempMax){
			   nTempAlte++;
			   nTempBasse=0;
		   } else{
			   nTempBasse++;
			   if(nTempBasse>nTempValid && pulsante == 0){
					   nTempAlte = 0;
					   nTempBasse=0;
					   if(incendio==1){
						   incendio=0;
					   }
			   }
		   }


		   int tmpInt1 = temp_value;
		   float tmpFrac = temp_value - tmpInt1;
		   int tmpInt2 = trunc(tmpFrac * 100);


		   if(snprintf(str_tmp4,30,"Temperatura = %d.%02d\n\r", tmpInt1,tmpInt2)>0){
			   HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp4,sizeof(str_tmp4),HAL_MAX_DELAY);
		   }
		   //se abbiamo letto molte temperature elevate, sblocchiamo il task della distanza e dichiariamo l'incendio
		   // e inviamo un messaggio
		   if(nTempAlte > nTempValid){
			   if(b2>0){
				   b2=0;
				   osSemaphoreRelease(myBinarySem02Handle);
				   HAL_UART_Transmit(&huart1,( uint8_t * )str_incendio,sizeof(str_incendio),HAL_MAX_DELAY);
			   }
			   incendio=1;
			   Temp DataToSend = {1,1};
			   osMessagePut(myQueue02Handle,(uint32_t)&DataToSend,200);
			   messaggioInviato = 1;
			   delay = 500;
		   }
		   else if(messaggioInviato == 1){
			   messaggioInviato = 0;
			   delay = 1500;
			   b2 = 1;
			   Temp DataToSend = {0,1};
			   osMessagePut(myQueue02Handle,(uint32_t)&DataToSend,200);
		   }
		   
		   uint32_t end = HAL_GetTick();
		   time2 = end - start;
		   snprintf(str_tmp12,100," WCET 1 = %ld\n\r",time2);
		   HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp12,sizeof(str_tmp12),HAL_MAX_DELAY);
		   HAL_UART_Transmit(&huart1,( uint8_t * )linea,sizeof(linea),HAL_MAX_DELAY);
		   osSemaphoreRelease(myMutex01Handle);

	  }
  /* USER CODE END StartTaskTemperatura */
}

/* USER CODE BEGIN Header_StartTaskDistanza */
/**
* @brief Function implementing the TaskDistanza thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDistanza */
void StartTaskDistanza(void const * argument)
{
  /* USER CODE BEGIN StartTaskDistanza */
  /* Infinite loop */
	osSemaphoreWait(myBinarySem02Handle, osWaitForever);
		  for(;;)
		  {
			  osDelay(delay);
			  uint32_t inizio3 = HAL_GetTick();
			  osSemaphoreWait(myMutex01Handle, osWaitForever);
			  snprintf(str_tmp9,100," Task 2 e' arrivato = %ld\n\r", inizio3);
			  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp9,sizeof(str_tmp9),HAL_MAX_DELAY);
			  uint32_t start = HAL_GetTick();
			  snprintf(str3,100," Il task 2 ha iniziato = %ld\n\r", start);
			  HAL_UART_Transmit(&huart1,( uint8_t * )str3,sizeof(str3),HAL_MAX_DELAY);
			  
			  //ricezione dati
			  uint16_t temperaturaElevata=0;
			  tempRicevuta=osMessageGet(myQueue02Handle,200);
			  temperaturaElevata = (((Temp*)tempRicevuta.value.p)->tempValue);

			  uint16_t pulsanteAttivato=0;
			  statoPulsanteRicevuto=osMessageGet(myQueue03Handle,200);
			  pulsanteAttivato = (((Temp*)statoPulsanteRicevuto.value.p)->tempValue);


			  if(temperaturaElevata == 1 || pulsanteAttivato == 1 ){
				  uint16_t PROXIMITY_Value = VL53L0X_PROXIMITY_GetDistance();
				  if(PROXIMITY_Value != 0){
					  int gesture_code = tof_gestures_detectSWIPE_1(PROXIMITY_Value, &gestureSwipeData);
					  Data DataToSend1 = {PROXIMITY_Value,gesture_code};
					  osMessagePut(myQueue01Handle,(uint32_t)&DataToSend1,200);
					  messaggioTrasmesso = 1;
				  }
				  if(b3>0){
					  b3=0;
					  osSemaphoreRelease(myBinarySem03Handle);
				  }
				  osSemaphoreRelease(myBinarySem02Handle);
			  }
			  else if (temperaturaElevata == 0 || pulsanteAttivato == 0){
				  b2=1;
			  }
			  
			  uint32_t end = HAL_GetTick();
			  time3 = end - start;
			  snprintf(str_tmp13,100," WCET 2 = %ld\n\r",time3);
			  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp13,sizeof(str_tmp13),HAL_MAX_DELAY);
			  HAL_UART_Transmit(&huart1,( uint8_t * )linea,sizeof(linea),HAL_MAX_DELAY);
			  
			  osSemaphoreRelease(myMutex01Handle);
			  osSemaphoreWait(myBinarySem02Handle, osWaitForever);

		  }
  /* USER CODE END StartTaskDistanza */
}

/* USER CODE BEGIN Header_StartTaskLed */
/**
* @brief Function implementing the TaskLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLed */
void StartTaskLed(void const * argument)
{
  /* USER CODE BEGIN StartTaskLed */
  /* Infinite loop */
	osSemaphoreWait(myBinarySem03Handle, osWaitForever);
			  for(;;)
			  {
				  osDelay(delay);
				  uint32_t inizio4 = HAL_GetTick();				  
				  osSemaphoreWait(myMutex01Handle, osWaitForever);
				  snprintf(str_tmp10,100," Task 3 e' arrivato = %ld\n\r", inizio4);
				  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp10,sizeof(str_tmp10),HAL_MAX_DELAY);
				  uint32_t start = HAL_GetTick();
				  snprintf(str4,100," Il task 3 ha iniziato = %ld\n\r", start);
				  HAL_UART_Transmit(&huart1,( uint8_t * )str4,sizeof(str4),HAL_MAX_DELAY);
				  
				  if(b2>0 || messaggioTrasmesso == 0){
					  b3=1;
				  }
				  else{
					  messaggioTrasmesso = 0;
					  distanzaRicevuta=osMessageGet(myQueue01Handle,200);
					  uint16_t distance = (((Data*)distanzaRicevuta.value.p)->Distance);
					  int gesture = (((Data*)distanzaRicevuta.value.p)->Gesture);

					  if(distance == 8190){
						  HAL_UART_Transmit(&huart1,( uint8_t * )msgdist,sizeof(msgdist),HAL_MAX_DELAY);
					  }
					  else{
						  if(snprintf(str_tmp2,30,"Distanza = %d \n\r", distance)>0){
							  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp2,sizeof(str_tmp2),HAL_MAX_DELAY);
						  }
					  }

					  if(gesture == 5){
						  action = 2;
					  }
					  if(gesture == 0 && action == 0){
						  HAL_UART_Transmit(&huart1,( uint8_t * )msggest,sizeof(msggest),HAL_MAX_DELAY);
					  }
					  else if(gesture == 0 && action == 2){
						  HAL_UART_Transmit(&huart1,( uint8_t * )msggest2,sizeof(msggest2),HAL_MAX_DELAY);

					  }
					  else{
						  if(snprintf(str_tmp5,30,"Gesto = %d\n\r",gesture)>0){
							  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp5,sizeof(str_tmp5),HAL_MAX_DELAY);
						  }
					  }

	  				  if((gesture==0 && action == 2)||((gesture == 5 || gesture == 6 ) && distance < distMax && distance > distMin)){
	  					HAL_GPIO_TogglePin(LED3_WIFI__LED4_BLE_GPIO_Port,LED3_WIFI__LED4_BLE_Pin);
	  				  }
	  				  else if (distance < distMax && distance > distMin){
	  					HAL_GPIO_TogglePin(LED3_WIFI__LED4_BLE_GPIO_Port,LED3_WIFI__LED4_BLE_Pin);
	  				  }
	  				  else if (distance <= distMin){
	  					HAL_GPIO_TogglePin(LED3_WIFI__LED4_BLE_GPIO_Port,LED3_WIFI__LED4_BLE_Pin);
	  					HAL_UART_Transmit(&huart1,( uint8_t * )porta,sizeof(porta),HAL_MAX_DELAY);
	  				  }
	  				  else{
	  					HAL_GPIO_WritePin(LED3_WIFI__LED4_BLE_GPIO_Port, LED3_WIFI__LED4_BLE_Pin,  GPIO_PIN_RESET);
	  				  }
	  				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  				osSemaphoreRelease(myBinarySem03Handle);

				  }
				  uint32_t end = HAL_GetTick();
				  time4 = end - start;
				  snprintf(str_tmp14,100," WCET 3 = %ld\n\r",time4);
				  HAL_UART_Transmit(&huart1,( uint8_t * )str_tmp14,sizeof(str_tmp14),HAL_MAX_DELAY);
				  HAL_UART_Transmit(&huart1,( uint8_t * )linea,sizeof(linea),HAL_MAX_DELAY);
				  
				  osSemaphoreRelease(myMutex01Handle);
				  osSemaphoreWait(myBinarySem03Handle, osWaitForever);

			  }
  /* USER CODE END StartTaskLed */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
