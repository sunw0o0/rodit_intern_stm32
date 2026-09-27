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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t PCrxBuffer[RX_BUFFER_SIZE];

uint8_t SW_DATA;          // sw 데이터. LSB(오른쪽)부터 PB12,13,14,15 (안 눌림=1, 눌림=0)

volatile uint16_t duty = 0; // 현재 PWM duty (0~1000), Live Expressions 확인용
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sw_data_set(void);
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
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // PA8 = 속도(PWM), 처음엔 멈춤
  TIM8->CCR3 = 0;                                      // PC8 = 방향, 항상 LOW로 고정
  TIM8->CCR4 = 0;                                      // 속도값 저장용
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);            // 타이머 시작
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE | TIM_IT_CC4);  // 인터럽트 켜기

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_Delay(20);  // 간단한 디바운싱
		sw_data_set();  // 스위치 값 읽기 (안 눌림=1, 눌림=0)


		// 누르고 있는 동안만 해당 duty, 떼면 정지
		if      (!(SW_DATA & 0x08)) duty = 700; // PB15 누르는 중 -> 70%
		else if (!(SW_DATA & 0x04)) duty = 500; // PB14 누르는 중 -> 50%
		else if (!(SW_DATA & 0x02)) duty = 300; // PB13 누르는 중 -> 30%
		else                        duty = 0;   // PB12 누르는 중이거나 아무것도 안 누름 -> 0%

			TIM8->CCR4 = duty; // Counter Period 1000 기준: 0~1000 = 0~100%

		// 스위치 눌림 상태를 LED로 표시 (LOW = 켜짐)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  (SW_DATA & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB12
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  (SW_DATA & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB13
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  (SW_DATA & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB14
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (SW_DATA & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB15


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 스위치 세팅 (PB12,13,14,15 -> SW_DATA 비트0~3)
void sw_data_set(void) {
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
		SW_DATA |= 0x01;
	else
		SW_DATA &= ~(0x01);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
		SW_DATA |= 0x02;
	else
		SW_DATA &= ~(0x02);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
		SW_DATA |= 0x04;
	else
		SW_DATA &= ~(0x04);

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
		SW_DATA |= 0x08;
	else
		SW_DATA &= ~(0x08);
}

// 주기 시작(카운터 0) -> PA8 HIGH
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM8) {
		if (duty > 0) GPIOA->BSRR = GPIO_PIN_8;                   // HIGH
		else          GPIOA->BSRR = (uint32_t)GPIO_PIN_8 << 16;   // duty 0이면 계속 LOW
	}
}

// 카운터가 CCR4에 도달 -> PA8 LOW
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (duty < 1000) GPIOA->BSRR = (uint32_t)GPIO_PIN_8 << 16; // LOW
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
