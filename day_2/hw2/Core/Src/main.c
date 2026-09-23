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
#define RX_BUFFER_SIZE 256 // USART 수신 버퍼 크기
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t PCrxBuffer[RX_BUFFER_SIZE];

uint8_t SW_DATA; // sw 데이터. LSB(오른쪽)부터 PB12,13,14,15
uint8_t SW_DATA_PREV = 0; // [수정1] 이전 스위치 상태 저장용 (Edge Detection)

/* Protocol 2.0 패킷 (CRC 포함, 정확하게 수정됨)
 * torq_on     : ID14, Address64(Torque Enable) = 1
 * pos_center  : ID14, Address116(Goal Position) = 1024 -> 약 90도  (스위치 PB12용)
 * pos_p90     : ID14, Address116(Goal Position) = 2048 -> 약 180도 (스위치 PB13용)
 * pos_m90     : ID14, Address116(Goal Position) = 0    -> 0도     (스위치 PB14용)
 */
//Protocol 2.0 규칙(Header+ID+Len+Inst+Addr+Data+CRC) 및 정확한 CRC16 반영
const uint8_t torq_on[] = {
	0xFF, 0xFF, 0xFD, 0x00, 0x0E, 0x06, 0x00, 0x03,
	0x40, 0x00, 0x01, 0x2B, 0x69
};
const uint8_t pos_center[] = {
	0xFF, 0xFF, 0xFD, 0x00, 0x0E, 0x09, 0x00, 0x03,
	0x74, 0x00, 0x00, 0x04, 0x00, 0x00, 0x71, 0x29
};
const uint8_t pos_p90[] = {
	0xFF, 0xFF, 0xFD, 0x00, 0x0E, 0x09, 0x00, 0x03,
	0x74, 0x00, 0x00, 0x08, 0x00, 0x00, 0x81, 0x29
};
const uint8_t pos_m90[] = {
	0xFF, 0xFF, 0xFD, 0x00, 0x0E, 0x09, 0x00, 0x03,
	0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0xA9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sw_data_set(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 패킷 송신
void pkt_send(const uint8_t *pkt, uint8_t len) {
	// [수정3] RS-485 / Half-Duplex 사용 시 Direction 핀을 TX HIGH로 설정하는 코드 예시
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // 사용 중인 DIR 핀에 맞게 주석 해제하여 사용

	for (uint8_t i = 0; i < len; i++) {
		while (!LL_USART_IsActiveFlag_TXE(USART3))
			;
		LL_USART_TransmitData8(USART3, pkt[i]);
	}
	while (!LL_USART_IsActiveFlag_TC(USART3))
		;

	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // 송신 종료 후 RX Mode (LOW) 전환
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
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	/* RX용 DMA + IDLE 인터럽트 설정 (다이나믹셀 응답 수신용) */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t) PCrxBuffer);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t) &USART3->DR);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, RX_BUFFER_SIZE);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	LL_USART_EnableDMAReq_RX(USART3);
	LL_USART_EnableIT_IDLE(USART3);

	HAL_Delay(100);            // 모터 전원 안정화 대기
	pkt_send(torq_on, sizeof(torq_on));   // 토크 켜기
	HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_Delay(50);
		sw_data_set(); // 스위치 값 읽어옴 (안 눌림=1, 눌림=0, 풀업 기준)

		// [수정4] Edge Detection: 눌리는 '순간'(High -> Low 전환) 1회만 패킷 전송
		uint8_t sw_pressed = (~SW_DATA) & SW_DATA_PREV;

		/* PB12 눌림 -> 90도 (pos_center) */
		if (!(SW_DATA & 0x01)) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			if (sw_pressed & 0x01) {
				pkt_send(pos_center, sizeof(pos_center));
			}
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		}

		/* PB13 눌림 -> 180도 (pos_p90) */
		if (!(SW_DATA & 0x02)) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			if (sw_pressed & 0x02) {
				pkt_send(pos_p90, sizeof(pos_p90));
			}
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		}

		/* PB14 눌림 -> 0도 (pos_m90) */
		if (!(SW_DATA & 0x04)) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			if (sw_pressed & 0x04) {
				pkt_send(pos_m90, sizeof(pos_m90));
			}
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		}

		SW_DATA_PREV = SW_DATA; // 현재 스위치 상태를 이전 상태로 업데이트

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
