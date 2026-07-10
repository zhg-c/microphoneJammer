/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  PWR_SW = 1,
  AUTO_SW,
  LOW_SW,
  HIGH_SW,
} key_t;
typedef struct{
  GPIO_PinState mic;
  GPIO_PinState high;
  GPIO_PinState low;
} state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BIAS 2048
#define SAMPLE_WINDOW 120
#define ZCR_DEADBAND 60
#define MIN_ZCR 2
#define MAX_ZCR 60

#define DMA_BUF_SIZE 240 // (SAMPLE_WINDOW * 2)


// 9v / (3.28v / 4096) * [10k / (53.6k + 10k)] ≈ 1767
#define PWR_LOW_RAW 1720  //为了对应电量显示模块的 0%
#define PWR_LOW_RAW_SUM 206400 // PWR_LOW_RAW * SAMPLE_WINDOW
#define PWR_LOW_RAW_SUM_2 216400 // PWR_LOW_RAW * SAMPLE_WINDOW + 10000 为了防止反复横跳

#define EV1527_SYNC_MIN 6000      // 同步码低电平最小 (us)
#define EV1527_SYNC_MAX 20000     // 同步码低电平最大 (us)
#define EV1527_BIT_TOTAL_MIN 800  // 一个位(高+低)最小总时间 (us)
#define EV1527_BIT_TOTAL_MAX 2500 // 一个位(高+低)最大总时间 (us)
#define EV1527_FRAME_LEN 24       // 24位数据
#define PAIR_FLASH_ADDR 0x0800F800 // PAIR 在 Flash 中的地址

#define FREQ_SIZE 50

// #define TEST 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
key_t g_key = PWR_SW;
uint8_t autoSwOnce = 0;
uint32_t sysTickCnt = 0;
uint8_t blinked = 0,lastBlinked = 0;
uint8_t pwrlow = 0,pwrlow2 = 0,pwrOn = 0;//pwrlow 表示电量低
uint8_t micAuto = 1,micRun = 0;
uint16_t adcBuf[DMA_BUF_SIZE] = {};

uint8_t g_pair = 0,g_cancelPair = 0,g_paired = 0,g_bRemote = 0;
state_t g_state = {0};
uint32_t g_remoteId = 0;

uint8_t pwmRun = 0;
const uint16_t freqs[FREQ_SIZE] = {
    24600, 24800, 25000, 25200, 25400, 25200, 25000, 24800, 24600, 24400,
    24300, 24500, 24700, 24900, 25100, 25300, 25500, 25300, 25100, 24900,
    24700, 24500, 24300, 24500, 24700, 24900, 25100, 25300, 25500, 25600,
    25400, 25200, 25000, 24800, 24600, 24400, 24200, 24400, 24600, 24800,
    25000, 25200, 25400, 25600, 25800, 25600, 25400, 25200, 25000, 24800};
const uint8_t dutys[FREQ_SIZE] = {51, 51, 50, 50, 49, 49, 50, 50, 51, 51, 52, 51, 51,
                         50, 50, 49, 48, 49, 49, 50, 51, 51, 52, 51, 51, 50,
                         50, 49, 48, 48, 49, 49, 50, 50, 51, 51, 52, 51, 51,
                         50, 50, 49, 49, 48, 48, 49, 49, 50, 50, 51};
uint16_t arrs[FREQ_SIZE] = {};   
uint16_t ccrs[FREQ_SIZE] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void keyScan(void);
void stopWork(void);
void startWork(void);
uint8_t clearId();
uint32_t readId();
void recoverState();
void pwrSwitch(uint8_t bLow);
void offLeds();
void onLeds();
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  for(int i = 0; i < FREQ_SIZE; i++) {
      arrs[i] = (48000000 / freqs[i]) - 1;
      ccrs[i] = (dutys[i] * arrs[i]) / 100;
  }
  g_remoteId = readId();
  if(g_remoteId != 0xFFFFFFFF){
    g_paired = 1;
  }
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcBuf, DMA_BUF_SIZE);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if(pwrlow2){
      pwrlow = 1;
      continue;
    }
    if(pwrOn && pwrlow){
      static uint8_t bOnce;
      if(!bOnce){
        bOnce = 1;
        blinked = 0;
        offLeds();
        HAL_GPIO_WritePin(PWR_LED_GPIO_Port,PWR_LED_Pin,GPIO_PIN_RESET);
      }
      if(pwmRun){
        stopWork();
      }
      continue;
    }
    static uint8_t bOnce;
    if(blinked && (g_key != PWR_SW && g_key != AUTO_SW)){
      if(!bOnce){
        bOnce = 1;
        sysTickCnt = 0;
      }
      if(sysTickCnt >= 600000){
        HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_RESET);
      }
      continue;
    }
    bOnce = 0;
    keyScan();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 124;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1919;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 960;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HIGH_LED_Pin|PWR_LOW_LED_Pin|PWR_LED_Pin|LOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AUTO_LED_Pin|VSPK_BST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_ON_Pin */
  GPIO_InitStruct.Pin = PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HIGH_LED_Pin PWR_LOW_LED_Pin LOW_LED_Pin */
  GPIO_InitStruct.Pin = HIGH_LED_Pin|PWR_LOW_LED_Pin|LOW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_LED_Pin */
  GPIO_InitStruct.Pin = PWR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AUTO_LED_Pin VSPK_BST_Pin */
  GPIO_InitStruct.Pin = AUTO_LED_Pin|VSPK_BST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HIGH_SW_Pin */
  GPIO_InitStruct.Pin = HIGH_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HIGH_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_SW_Pin */
  GPIO_InitStruct.Pin = PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AUTO_SW_Pin LOW_SW_Pin */
  GPIO_InitStruct.Pin = AUTO_SW_Pin|LOW_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void keyScan(void) {
  if(g_pair || g_cancelPair) {
    g_key = 0;
    return;
  }
  key_t key = g_key;
  g_key = 0;
  if(!g_bRemote){
    if (key == AUTO_SW && HAL_GPIO_ReadPin(AUTO_SW_GPIO_Port, AUTO_SW_Pin) == GPIO_PIN_SET) {
      return;
    }
    if (key == LOW_SW && HAL_GPIO_ReadPin(LOW_SW_GPIO_Port, LOW_SW_Pin) == GPIO_PIN_SET) {
      return;
    }
  }
  g_bRemote = 0;
  if(HAL_GPIO_ReadPin(HIGH_SW_GPIO_Port, HIGH_SW_Pin) == GPIO_PIN_RESET) {
    key = HIGH_SW;
  }
  if(key) {
    switch(key) {
      case PWR_SW:
        sysTickCnt = 0;
        uint8_t shortPress = 1;
        while (HAL_GPIO_ReadPin(PWR_SW_GPIO_Port, PWR_SW_Pin) == GPIO_PIN_RESET) {
          if(sysTickCnt >= 1000){
            shortPress = 0;
            if(!pwrOn) {
              pwrOn = 1;
              blinked = !pwrlow2;
              g_state.low = GPIO_PIN_SET;
              HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_SET);
            }else{
              pwrOn = 0;
              blinked = 0;
              HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_RESET);
              g_state.mic = g_state.high = g_state.low = GPIO_PIN_RESET;
              offLeds();
              HAL_GPIO_WritePin(PWR_LED_GPIO_Port,PWR_LED_Pin,GPIO_PIN_RESET);
            }
            while(HAL_GPIO_ReadPin(PWR_SW_GPIO_Port, PWR_SW_Pin) == GPIO_PIN_RESET);
          }
        }
        if(pwrlow2){
          break;
        }
        if(shortPress && pwrOn) {
          if(blinked){
            blinked = 0;
            if(micAuto){
              g_state.mic = GPIO_PIN_RESET;
            }
            onLeds();
            startWork();
          }else{
            blinked = 1;
            micRun = 0;
            stopWork();
          }
          micAuto = blinked;
        }
        break;
      case AUTO_SW:{
        if(micRun){
          micRun = 0;
          blinked = micAuto;
          if(blinked){
            stopWork();
          }else{
            startWork();
          }
          g_state.mic = GPIO_PIN_RESET;
          HAL_GPIO_WritePin(AUTO_LED_GPIO_Port,AUTO_LED_Pin,GPIO_PIN_RESET);
        }else{
          micRun = 1;
          blinked = 0;
          g_state.mic = GPIO_PIN_SET;
          if(micAuto){
            onLeds();
          }else{
            HAL_GPIO_WritePin(AUTO_LED_GPIO_Port,AUTO_LED_Pin,GPIO_PIN_SET);
          }
        }
      }
        break;
      case LOW_SW:
        sysTickCnt = 0;
        while (HAL_GPIO_ReadPin(LOW_SW_GPIO_Port, LOW_SW_Pin) == GPIO_PIN_RESET) {
          if(sysTickCnt >= 1000){
            g_cancelPair = 1;
            g_paired = 0;
            clearId();
            while (HAL_GPIO_ReadPin(LOW_SW_GPIO_Port, LOW_SW_Pin) == GPIO_PIN_RESET);
          }
        }
        if(g_cancelPair) {
          break;
        }
        g_state.low = GPIO_PIN_SET;
        g_state.high = GPIO_PIN_RESET;
        pwrSwitch(1);
        break;
      case HIGH_SW:
        sysTickCnt = 0;
        while (HAL_GPIO_ReadPin(HIGH_SW_GPIO_Port, HIGH_SW_Pin) == GPIO_PIN_RESET) {
          if(sysTickCnt >= 1000){
            g_pair = 1;
            while (HAL_GPIO_ReadPin(HIGH_SW_GPIO_Port, HIGH_SW_Pin) == GPIO_PIN_RESET);
          }
        }
        if(g_pair) {
          break;
        }
        g_state.high = GPIO_PIN_SET;
        g_state.low = GPIO_PIN_RESET;
        pwrSwitch(0);
        break;
      default:
        break;
    }
  }
  if(blinked){
    offLeds();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch(GPIO_Pin) {
    case PWR_SW_Pin:
      g_key = PWR_SW;
      break;
    case AUTO_SW_Pin:
      g_key = AUTO_SW;
      break;
    case LOW_SW_Pin:
      g_key = LOW_SW;
      break;
    default:
      break;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    static uint8_t lastSide;
    static uint8_t talkCnt;
    static uint8_t winCnt;
    static uint8_t changeWin = 10;//可通过调整窗口大小，实现延时关闭pwm
    uint8_t zcrCnt = 0;

    uint32_t voltSum = 0;
    for(uint16_t i = 0;i < DMA_BUF_SIZE; i += 2){
      voltSum += adcBuf[i];

      int16_t diff = adcBuf[i + 1] - ADC_BIAS;
      uint16_t absDiff = abs(diff);
      uint8_t side = (diff > 0) ? 1 : 0;
      if(side != lastSide && absDiff > ZCR_DEADBAND) {
        lastSide = side;
        zcrCnt++;
      }
    }
    #ifndef TEST
    static uint8_t bOnce;
    if(!bOnce){
      bOnce = 1;
      if(voltSum < PWR_LOW_RAW_SUM_2){
        pwrlow2 = 1;
      }
    }
    if(voltSum < PWR_LOW_RAW_SUM) {
      pwrlow = 1;
      return;
    }
    #endif // !TEST
    if(!micRun) {
      return;
    }
    if(zcrCnt >= MIN_ZCR && zcrCnt <= MAX_ZCR) {
      talkCnt++;
    }
    if(winCnt++ < changeWin) {
      return;
    }
    if(!pwmRun && talkCnt > 3) {
      HAL_GPIO_WritePin(PWR_LOW_LED_GPIO_Port,PWR_LOW_LED_Pin,GPIO_PIN_SET);
      startWork();
      changeWin *= 10;
    } else if(pwmRun && talkCnt < 2) {
      HAL_GPIO_WritePin(PWR_LOW_LED_GPIO_Port,PWR_LOW_LED_Pin,GPIO_PIN_RESET);
      stopWork();
      changeWin = 10;
    }
    winCnt = 0;
    talkCnt = 0;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  switch ((uint32_t)htim->Instance)
  {
  case (uint32_t)TIM3:
    static uint16_t period = 1919;
    static uint16_t pulseWidth = 950;
    static uint8_t cnt;
    static uint8_t idx;
    if (++cnt >= 8) {
      cnt = 0;
      pulseWidth = ccrs[idx % FREQ_SIZE];
      period = arrs[idx++ % FREQ_SIZE];
    }
    TIM3->ARR = period;
    TIM3->CCR1 = pulseWidth;
    TIM3->CCR2 = pulseWidth;
    break;
  case (uint32_t)TIM14:
    sysTickCnt++;
    static uint16_t led_cnt;
    if(++led_cnt >= 500) {
      led_cnt = 0;
      if(pwrlow && pwrOn) {
        static uint8_t off;
        if(off++ > 8) {
          off = 0;
          HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_RESET);
        }
        HAL_GPIO_TogglePin(PWR_LOW_LED_GPIO_Port, PWR_LOW_LED_Pin);
        break;
      } 
      if(blinked){
        HAL_GPIO_TogglePin(PWR_LED_GPIO_Port, PWR_LED_Pin);
      }
      static uint32_t pairTick;
      if(g_pair) {
        HAL_GPIO_TogglePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin);
        static uint8_t bOnce;
        if(!bOnce){
          bOnce = 1;
          pairTick = sysTickCnt;
        }
        if(sysTickCnt - pairTick >= 5000){
          g_pair = 0;
          bOnce = 0;
          HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, g_state.high);
        }
      }else if(g_cancelPair) {
        HAL_GPIO_TogglePin(LOW_LED_GPIO_Port, LOW_LED_Pin);
        static uint8_t bOnce;
        if(!bOnce){
          bOnce = 1;
          pairTick = sysTickCnt;
        }
        if(sysTickCnt - pairTick >= 2000){
          g_cancelPair = 0;
          bOnce = 0;
          HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, g_state.low);
        } 
      }
    }
    break;
  default:
    break;
  }
}

/* USER CODE BEGIN 4 */
uint8_t saveId(uint32_t id) {
  if(HAL_FLASH_Unlock() != HAL_OK) {
    return 0;
  }
  HAL_StatusTypeDef status = HAL_OK; 
  do
  {
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = PAIR_FLASH_ADDR;
    erase.NbPages = 1;
    uint32_t err;
    if((status = HAL_FLASHEx_Erase(&erase, &err)) != HAL_OK) {
      break;
    }
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PAIR_FLASH_ADDR, id);
  } while (0);
  HAL_FLASH_Lock();
  return status == HAL_OK;
}
uint8_t clearId(){
  return saveId(0xFFFFFFFF);
}
uint32_t readId() {
  return *(uint32_t*)PAIR_FLASH_ADDR;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM15) {
    static uint32_t high, low,decodeState, bitCnt, tempBuf, lastCnt;
    uint32_t currCnt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    uint32_t diff = currCnt >= lastCnt ? (currCnt - lastCnt) : (65535 - lastCnt + currCnt + 1);
    lastCnt = currCnt;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
      high = diff;
    } else {
      low = diff;
      if(low >= EV1527_SYNC_MIN && low <= EV1527_SYNC_MAX) {
        decodeState = 1;
        bitCnt = 0;
        tempBuf = 0;
        return;
      }
      if(decodeState == 1) {
        uint32_t bitTime = high + low;
        if(bitTime < EV1527_BIT_TOTAL_MIN || bitTime > EV1527_BIT_TOTAL_MAX) {
          decodeState = 0;
          return;
        }
        tempBuf <<= 1;
        if(high > (low * 1.5)) {
          tempBuf |= 1;
        }
        bitCnt++;
        if(bitCnt >= EV1527_FRAME_LEN) {
          static uint32_t lastData;
          static uint32_t lastTick,debounceTick;
          uint32_t data = tempBuf;
          uint32_t now = HAL_GetTick();
          if(data == lastData && (now - lastTick < 200)) { //两组数据相同且时间小于200ms，认为是有效按键
            uint32_t id = data >> 4;
            uint8_t btn = data & 0x0F;
            
            if(g_pair){
              g_pair = 0;
              saveId(id);
              g_paired = 1;
              g_remoteId = id;
              HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, g_state.high);
              return;
            }
            if(g_paired && id != g_remoteId) {
              return;
            }
            if(!debounceTick){
              debounceTick = now;
            }
            if(now - debounceTick < 100){
              return;
            }
            if(btn && ((btn & 0x0f) == btn)){
              g_bRemote = 1;
            }
            switch(btn) {
            case 0x01:
              g_key = PWR_SW;
              break;
            case 0x02:
              g_key = AUTO_SW;
              break;
            case 0x04:
              g_key = HIGH_SW;
              break;
            case 0x08:
              g_key = LOW_SW;
              break;
            default:
              break;
            }
            debounceTick = 0;
          }
          lastData = data;
          lastTick = now;
          decodeState = 0;
          bitCnt = 0;
        }
      }    
    }
  }
}

void stopWork(void)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Stop_IT(&htim3);

  GPIO_InitTypeDef pwmInit = {0};
  pwmInit.Pin = PWM1_Pin | PWM2_Pin;
  pwmInit.Mode = GPIO_MODE_OUTPUT_PP;
  pwmInit.Pull = GPIO_NOPULL;
  pwmInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PWM1_GPIO_Port, &pwmInit);

  HAL_GPIO_WritePin(PWM1_GPIO_Port, PWM1_Pin | PWM2_Pin, GPIO_PIN_RESET);
  pwmRun = 0;
}

void startWork(void)
{
  HAL_TIM_MspPostInit(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  pwmRun = 1;
}

void pwrSwitch(uint8_t bLow){
  if(bLow){
    HAL_GPIO_WritePin(VSPK_BST_GPIO_Port, VSPK_BST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, GPIO_PIN_RESET);
  }else{
    HAL_GPIO_WritePin(VSPK_BST_GPIO_Port, VSPK_BST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, GPIO_PIN_RESET);
  }
}

void offLeds(){
  HAL_GPIO_WritePin(AUTO_LED_GPIO_Port, AUTO_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, GPIO_PIN_RESET);
}

void onLeds(){
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port,PWR_LED_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(AUTO_LED_GPIO_Port, AUTO_LED_Pin, g_state.mic);
  HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, g_state.high);
  HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, g_state.low);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
