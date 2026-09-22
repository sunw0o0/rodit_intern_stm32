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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
#define LED_COUNT 4
GPIO_TypeDef* led_port[LED_COUNT] = {GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t led_pin[LED_COUNT]       = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10};
uint8_t led_pattern = 0x01;

#define ADC_CH_COUNT     4
#define MA_WINDOW_SIZE   5
#define ADC_MIN          0
#define ADC_MAX          4095

/* 0 = 기존 flag 방식 (콜백에서 flag 세팅) / 1 = 4단계: NDTR 폴링 방식 (flag 없이) */
#define USE_NDTR_METHOD  0

uint32_t adc1_buffer[ADC_CH_COUNT];
volatile uint8_t adc_updated_flag = 0;

/* 3단계: 채널별 필터 상태를 구조체로 묶어서 독립 관리 */
typedef struct {
    uint16_t history[MA_WINDOW_SIZE];
    uint8_t  index;
    uint32_t sum;
    uint8_t  filled;
} MovingAvgFilter_t;

MovingAvgFilter_t ma_filter[ADC_CH_COUNT];

uint16_t adc_raw_value[ADC_CH_COUNT];
uint16_t adc_filtered_value[ADC_CH_COUNT];
float    adc_normalized_value[ADC_CH_COUNT];

/* 4단계용: 이전에 읽은 DMA 남은 카운터 값 */
uint32_t last_ndtr = ADC_CH_COUNT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SW_PRESSED(port, pin) (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)

static void UpdateLEDs(uint8_t pattern, uint8_t invert)
{
    uint8_t out = invert ? (~pattern & 0x0F) : (pattern & 0x0F);
    for (int i = 0; i < LED_COUNT; i++) {
        HAL_GPIO_WritePin(led_port[i], led_pin[i],
                           (out & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static uint8_t RotateLeft4(uint8_t val, uint8_t n)
{
    n %= 4;
    return ((val << n) | (val >> (4 - n))) & 0x0F;
}

static uint8_t RotateRight4(uint8_t val, uint8_t n)
{
    n %= 4;
    return ((val >> n) | (val << (4 - n))) & 0x0F;
}

/* 3단계: 이동평균 필터를 함수로 분리, 채널별 상태(f)를 인자로 받음 */
static uint16_t MovingAverageFilter(MovingAvgFilter_t *f, uint16_t new_value)
{
    f->sum -= f->history[f->index];
    f->history[f->index] = new_value;
    f->sum += new_value;
    f->index = (f->index + 1) % MA_WINDOW_SIZE;
    if (f->index == 0) f->filled = 1;

    uint8_t divisor = f->filled ? MA_WINDOW_SIZE : f->index;
    return (uint16_t)(f->sum / divisor);
}

static float NormalizeMinMax(uint16_t value, uint16_t min, uint16_t max)
{
    return (float)(value - min) / (float)(max - min);
}

/* 4채널 전부 처리하는 공통 함수 (flag 방식이든 NDTR 방식이든 여기 재사용) */
static void ProcessAllChannels(void)
{
    for (uint8_t ch = 0; ch < ADC_CH_COUNT; ch++)
    {
        adc_raw_value[ch]        = (uint16_t)adc1_buffer[ch];
        adc_normalized_value[ch] = NormalizeMinMax(adc_raw_value[ch], ADC_MIN, ADC_MAX);
        adc_filtered_value[ch]   = MovingAverageFilter(&ma_filter[ch], adc_raw_value[ch]);
    }
}
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
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* TIM8이 ADC1 트리거 소스이므로 타이머부터 시작 */
  HAL_TIM_Base_Start(&htim8);

  /* ADC를 DMA 모드로 시작 (circular, 4채널) */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC_CH_COUNT);

  while (1)
  {
#if USE_NDTR_METHOD
      /* 4단계: flag 없이, DMA 남은 전송 개수(NDTR)로 새 라운드 감지 */
      uint32_t current_ndtr = __HAL_DMA_GET_COUNTER(&hdma_adc1);

      if (current_ndtr > last_ndtr)   /* 카운터가 리셋되어 커졌다 = 한 바퀴 돌았다 */
      {
          ProcessAllChannels();
      }
      last_ndtr = current_ndtr;
#else
      /* 기존 방식: 콜백이 세운 flag로 감지 */
      if (adc_updated_flag)
      {
          adc_updated_flag = 0;
          ProcessAllChannels();
      }
#endif

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_updated_flag = 1;   // "한 라운드 변환 끝났다" 신호만
    }
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
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
