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
} _tKey;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BIAS 2048
#define SAMPLE_WINDOW 120
#define ZCR_DEADBAND 60
#define MIN_ZCR 2
#define MAX_ZCR 60

#define DMA_BUF_SIZE 240 // (SAMPLE_WINDOW * 2)

// 低于 9v 为不工作电压
// 9v / (3.3v / 4096) * [(53.6k + 10k) / 10k] ≈ 1756
#define PWR_LOW_RAW 1756
#define PWR_LOW_RAW_SUM 210720 // PWR_LOW_RAW * SAMPLE_WINDOW

#define EV1527_SYNC_MIN 6000      // 同步码低电平最小 (us)
#define EV1527_SYNC_MAX 20000     // 同步码低电平最大 (us)
#define EV1527_BIT_TOTAL_MIN 800  // 一个位(高+低)最小总时间 (us)
#define EV1527_BIT_TOTAL_MAX 2500 // 一个位(高+低)最大总时间 (us)
#define EV1527_FRAME_LEN 24       // 24位数据

#define FREQ_SIZE 50
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
_tKey key = PWR_SW;
uint8_t autoSwOnce = 0;
uint16_t sysTickCnt = 0;
uint8_t blinked = 0;
uint8_t pwrlow = 0,pwrOn = 0;
uint8_t openedMic = 0;
uint16_t adcBuf[DMA_BUF_SIZE] = {};

uint8_t pwmRun = 0,pwmStop = 0;
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
void Safe_PWM_Stop(void);
void Safe_PWM_Start(void);
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
      ccrs[i] = (dutys[i] * ((48000000 / freqs[i]) - 1)) / 100;
  }
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcBuf, DMA_BUF_SIZE);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim15);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if(pwrOn && pwrlow){
      pwmStop = 1;
      if(pwmRun){
        pwmRun = 0;
        Safe_PWM_Stop();
      }
      continue;
    }
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
  htim15.Init.Prescaler = 479;
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
  if(HAL_GPIO_ReadPin(HIGH_SW_GPIO_Port, HIGH_SW_Pin) == GPIO_PIN_RESET) {
    key = HIGH_SW;
  }
  if(key) {
    switch(key) {
      case PWR_SW:
        sysTickCnt = 0;
        while (HAL_GPIO_ReadPin(PWR_SW_GPIO_Port, PWR_SW_Pin) == GPIO_PIN_RESET) {
          if(sysTickCnt >= 1000){
            if(HAL_GPIO_ReadPin(PWR_ON_GPIO_Port, PWR_ON_Pin) == GPIO_PIN_RESET) {
              blinked = 1;
              HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_SET);
              pwrOn = 1;
            }else{
              blinked = 0;
              HAL_GPIO_WritePin(PWR_ON_GPIO_Port, PWR_ON_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, HIGH_LED_Pin|LOW_LED_Pin|PWR_LED_Pin|PWR_LOW_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(AUTO_LED_GPIO_Port,AUTO_LED_Pin, GPIO_PIN_RESET);
            }
            break;
          }
        }
        if(sysTickCnt < 1000 && pwrOn) {
          if(pwmStop){
            pwmStop = 0;
            blinked = 0;
            Safe_PWM_Start();
            HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_SET);
            static uint8_t once;
            if(!once){
              once = 1;
              HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, GPIO_PIN_SET);
            }
          }else{
            pwmStop = 1;
            blinked = 1;
            Safe_PWM_Stop();
          }
        }
        break;
      case AUTO_SW:
        if(!openedMic) {
          openedMic = 1;
          HAL_GPIO_WritePin(AUTO_LED_GPIO_Port,AUTO_LED_Pin, GPIO_PIN_SET);
        }else{
          openedMic = 0;
          HAL_GPIO_WritePin(AUTO_LED_GPIO_Port,AUTO_LED_Pin, GPIO_PIN_RESET);
        }
        break;
      case LOW_SW:
        HAL_GPIO_WritePin(VSPK_BST_GPIO_Port, VSPK_BST_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, GPIO_PIN_RESET);
        break;
      case HIGH_SW:
        HAL_GPIO_WritePin(VSPK_BST_GPIO_Port, VSPK_BST_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(HIGH_LED_GPIO_Port, HIGH_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LOW_LED_GPIO_Port, LOW_LED_Pin, GPIO_PIN_RESET);
        break;
      default:
        key = 0;
        break;
    }
    key = 0;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch(GPIO_Pin) {
    case PWR_SW_Pin:
      key = PWR_SW;
      break;
    case AUTO_SW_Pin:
      key = AUTO_SW;
      break;
    case LOW_SW_Pin:
      key = LOW_SW;
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
    static uint8_t changeWin = 10;
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
    if(voltSum < PWR_LOW_RAW_SUM) {
      pwrlow = 1;
      return;
    }

    if(pwmStop || !openedMic) {
      return;
    }

    if(zcrCnt >= MIN_ZCR && zcrCnt <= MAX_ZCR) {
      talkCnt++;
    }
    if(winCnt++ < changeWin) {
      return;
    }
    if(!pwmRun && talkCnt > 3) {
      Safe_PWM_Start();
      changeWin *= 10;
      pwmRun = 1;
    } else if(pwmRun && talkCnt < 2) {
      Safe_PWM_Stop();
      changeWin = 10;
      pwmRun = 0;
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
    static uint8_t cnt = 0;
    static uint8_t idx = 0;
    if (++cnt >= 8) {
      cnt = 0;
      pulseWidth = ccrs[idx++ % FREQ_SIZE];
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
    }
    break;
  case (uint32_t)TIM15:
    static uint32_t last_ic_val = 0;
    static uint32_t high_t = 0;
    static uint32_t low_t = 0;
    static uint8_t decode_state = 0;
    static uint8_t bit_cnt = 0;
    static uint32_t temp_buf = 0;
    static uint32_t rf_final_data = 0;   // 最终确认的24位数据
    // static uint32_t rf_remote_id = 0;    // 提取的ID
    static uint8_t rf_remote_key = 0;    // 提取的按键值
    static uint32_t last_raw_data = 0;   // 用于双帧校验的缓存
    static uint32_t last_frame_tick = 0; // 用于计算双帧之间的时间间隔

    uint32_t current_ic_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    uint32_t diff = (current_ic_val >= last_ic_val)
                        ? (current_ic_val - last_ic_val)
                        : (65535 - last_ic_val + current_ic_val + 1);
    last_ic_val = current_ic_val;

    diff *= 10;

    // 2. 识别电平（根据刚才结束的那一段）
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
      high_t = diff; // 刚才结束的是高电平
    } else {
      low_t = diff; // 刚才结束的是低电平

      // 3. 状态机 - 寻找同步码
      if (low_t >= EV1527_SYNC_MIN && low_t <= EV1527_SYNC_MAX) {
        decode_state = 1;
        bit_cnt = 0;
        temp_buf = 0;
        return;
      }

      // 4. 状态机 - 解码数据位
      if (decode_state == 1) {
        uint32_t bit_time = high_t + low_t;

        // 严苛过滤：如果这一位总时长不符合协议，直接判定为杂讯
        if (bit_time < EV1527_BIT_TOTAL_MIN ||
            bit_time > EV1527_BIT_TOTAL_MAX) {
          decode_state = 0;
          return;
        }

        // 比例判定逻辑 (1.5倍是兼顾灵敏度和抗噪的最佳值)
        temp_buf <<= 1;
        if (high_t > (low_t * 1.5)) {
          temp_buf |= 1; // 长高短低 = 1
        }
        // 反之即为 0 (无需操作)

        bit_cnt++;

        // 5. 帧接收完成
        if (bit_cnt >= EV1527_FRAME_LEN) {
          uint32_t current_raw = temp_buf;
          uint32_t now_tick = HAL_GetTick();

          // --- 核心：动态双帧校验逻辑 ---
          // 必须满足两个条件：
          // 1. 两次数据完全一致
          // 2. 两次数据的时间间隔不能超过 200ms (防止两个不同时间的乱码撞车)
          if ((current_raw == last_raw_data) &&
              (now_tick - last_frame_tick < 200)) {
            rf_final_data = current_raw;
            // rf_remote_id = rf_final_data >> 4;
            rf_remote_key = rf_final_data & 0x0F;
            if (rf_remote_key == 0x02) {
              if(pwmStop){
                pwmStop = 0;
                key = PWR_SW;
              }
            } else if (rf_remote_key == 0x04) {
              if(!openedMic) {
                openedMic = 1;
                key = PWR_SW;
              }
            }
          }

          // 更新上一帧参考信息
          last_raw_data = current_raw;
          last_frame_tick = now_tick;

          decode_state = 0;
          bit_cnt = 0;
        }
      }
    }
    break;
  default:
    break;
  }
}

/**
  * @brief 安全关闭 PWM：强制切为 GPIO 低电平，保护硬件
  */
void Safe_PWM_Stop(void)
{
  // 1. 停止定时器中断和输出
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Stop_IT(&htim3);

  // 2. 强行切为普通推挽输出模式
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5; // 确认为你的实际引脚
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 强制普通推挽
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);        // 确认为你的实际端口

  // 3. 强行拉低，锁定绝对安全状态
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
}

/**
  * @brief 安全开启 PWM：利用 MSP 恢复复用模式，再使能定时器
  */
void Safe_PWM_Start(void)
{
  // 1. 直接调用 CubeMX 自动生成的 MSP 后初始化函数
  // 它会精准、安全地将 GPIOB4/5 重新配置为 TIM3 的复用 PWM 引脚
  HAL_TIM_MspPostInit(&htim3);

  // 2. 重新启动硬件 PWM 输出和更新中断
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
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
