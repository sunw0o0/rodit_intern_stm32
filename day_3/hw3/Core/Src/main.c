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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>   // memcpy
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256

/* ---- Robstride00 (private protocol, 29bit extended ID, 1Mbps) ---- */
#define RS_MOTOR_ID     14      // 강의 자료의 ID (스캔 실패 시 사용)
#define RS_MASTER_ID    0xFD    // 호스트(STM32) ID

/* 통신 타입 (29bit ID의 bit28~24) */
#define RS_TYPE_GET_ID    0     // 장치 ID 요청 (ID 스캔용)
#define RS_TYPE_FEEDBACK  2     // 모터 -> 호스트 상태 피드백
#define RS_TYPE_ENABLE    3     // 모터 Enable
#define RS_TYPE_STOP      4     // 모터 Stop
#define RS_TYPE_PARAM_WR  0x12  // 단일 파라미터 쓰기 (18)

/* 파라미터 인덱스 */
#define RS_IDX_RUN_MODE   0x7005  // uint8 : 2 = 속도 모드
#define RS_IDX_SPD_REF    0x700A  // float : 목표 속도 [rad/s] (-33~33)
#define RS_IDX_LIMIT_CUR  0x7018  // float : 전류 제한 [A] (0~16)
#define RS_IDX_ACC_RAD    0x7022  // float : 속도 모드 가속도 [rad/s^2]

/* 과제 조건: 24V, 속도는 데이터시트 값의 절반 미만
 * 무부하 315rpm(48V) ≈ 33 rad/s -> 절반 16.5, 24V 기준 절반 ≈ 8 -> 5 rad/s 사용 */
#define SPD_MAX     5.0f    // motor_cmd = ±1 일 때 속도 [rad/s]
#define LIMIT_CUR   3.0f    // 전류 제한 [A]
#define ACC_RAD     10.0f   // 가속도 [rad/s^2]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t PCrxBuffer[RX_BUFFER_SIZE];

volatile double motor_cmd = 0.0;   // -1.0 ~ 1.0 (일시정지 후 Expressions에서 입력)
volatile float  spd_target = 0.0f; // 보내는 목표 속도 [rad/s]

/* ---- 진단용 (Live Expressions) ---- */
volatile uint8_t  rs_id    = RS_MOTOR_ID; // 실제 사용 중인 모터 ID
volatile uint8_t  id_found = 0;           // 1: 스캔으로 모터를 찾음, 0: 못 찾음(14 사용)
volatile uint32_t tx_ok    = 0;           // 송신 요청 성공 횟수
volatile uint32_t tx_fail  = 0;           // 송신 실패 횟수 (메일박스 꽉 참 등)
volatile uint32_t rx_any   = 0;           // 받은 CAN 메시지 수 (종류 무관)
volatile uint32_t last_rx_id = 0;         // 마지막으로 받은 메시지의 29bit ID
volatile uint32_t can_esr  = 0;           // CAN 에러 레지스터 (끝자리 3x = ACK 에러)

/* ---- 모터 피드백 ---- */
volatile float    fb_speed = 0.0f;  // 현재 속도 [rad/s]
volatile uint8_t  fb_mode  = 0;     // 0: Reset, 1: Cali, 2: Run
volatile uint8_t  fb_fault = 0;     // 0이면 정상
volatile uint32_t fb_count = 0;     // 받은 피드백 개수 (계속 늘어나야 정상)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void rs_send(uint8_t type, uint8_t data[8]);
static void rs_scan(void);
static void rs_enable(void);
static void rs_stop(void);
static void rs_write_u8(uint16_t index, uint8_t value);
static void rs_write_float(uint16_t index, float value);
static void rs_read_feedback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 29bit ID = [통신타입(bit28~24)] [호스트ID(bit15~8)] [모터ID(bit7~0)] */
static void rs_send(uint8_t type, uint8_t data[8])
{
	CAN_TxHeaderTypeDef tx = {0};
	uint32_t mailbox;

	tx.ExtId = ((uint32_t)type << 24) | ((uint32_t)RS_MASTER_ID << 8) | rs_id;
	tx.IDE   = CAN_ID_EXT;
	tx.RTR   = CAN_RTR_DATA;
	tx.DLC   = 8;
	tx.TransmitGlobalTime = DISABLE;

	uint32_t t0 = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
	{
		if (HAL_GetTick() - t0 > 10) { tx_fail++; return; }
	}
	if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) == HAL_OK) tx_ok++;
	else tx_fail++;
}

/* 통신 타입 0을 ID 14, 127, 1~126 순서로 보내고 응답한 모터 ID를 찾음
 * 응답: 타입 0, bit15~8 = 모터 CAN ID */
static void rs_scan(void)
{
	uint8_t zero[8] = {0};
	CAN_RxHeaderTypeDef rx;
	uint8_t d[8];

	for (uint16_t n = 0; n < 128; n++)
	{
		uint8_t id;
		if      (n == 0) id = RS_MOTOR_ID;   // 14 먼저
		else if (n == 1) id = 127;           // 공장 기본값
		else             id = (uint8_t)(n - 1);  // 1 ~ 126
		if (n >= 2 && id == RS_MOTOR_ID) continue;

		rs_id = id;
		rs_send(RS_TYPE_GET_ID, zero);
		HAL_Delay(5);

		while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, d) != HAL_OK) break;
			rx_any++;
			last_rx_id = rx.ExtId;
			if (rx.IDE == CAN_ID_EXT && ((rx.ExtId >> 24) & 0x1F) == RS_TYPE_GET_ID)
			{
				rs_id = (rx.ExtId >> 8) & 0xFF;
				id_found = 1;
				return;
			}
		}
	}
	rs_id = RS_MOTOR_ID;   // 못 찾으면 14
}

// 통신 타입 3: 모터 Enable
static void rs_enable(void)
{
	uint8_t d[8] = {0};
	rs_send(RS_TYPE_ENABLE, d);
}

// 통신 타입 4: 모터 Stop
static void rs_stop(void)
{
	uint8_t d[8] = {0};
	rs_send(RS_TYPE_STOP, d);
}

// 통신 타입 18: Byte0~1 = index(low byte 먼저), Byte4 = 값(uint8)
static void rs_write_u8(uint16_t index, uint8_t value)
{
	uint8_t d[8] = {0};
	d[0] = index & 0xFF;
	d[1] = index >> 8;
	d[4] = value;
	rs_send(RS_TYPE_PARAM_WR, d);
}

// 통신 타입 18: Byte0~1 = index, Byte4~7 = float(low byte 먼저)
static void rs_write_float(uint16_t index, float value)
{
	uint8_t d[8] = {0};
	d[0] = index & 0xFF;
	d[1] = index >> 8;
	memcpy(&d[4], &value, 4);
	rs_send(RS_TYPE_PARAM_WR, d);
}

// 수신 처리: 타입 2 피드백이면 상태 갱신
static void rs_read_feedback(void)
{
	CAN_RxHeaderTypeDef rx;
	uint8_t d[8];

	while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
	{
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, d) != HAL_OK) break;
		rx_any++;
		last_rx_id = rx.ExtId;
		if (rx.IDE != CAN_ID_EXT) continue;

		if (((rx.ExtId >> 24) & 0x1F) != RS_TYPE_FEEDBACK) continue;

		fb_fault = (rx.ExtId >> 16) & 0x3F;
		fb_mode  = (rx.ExtId >> 22) & 0x03;

		uint16_t raw = ((uint16_t)d[2] << 8) | d[3];   // high byte 먼저
		fb_speed = (float)raw * 66.0f / 65535.0f - 33.0f;
		fb_count++;
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
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	/* CAN1을 1Mbps로 재설정 (CubeMX 값과 상관없이)
	 * APB1 45MHz / 5 / (1 + 6 + 2) = 1Mbps */
	HAL_CAN_DeInit(&hcan1);
	hcan1.Init.Prescaler     = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1      = CAN_BS1_6TQ;
	hcan1.Init.TimeSeg2      = CAN_BS2_2TQ;
	HAL_CAN_Init(&hcan1);

	/* CAN 필터: 모든 메시지 수신 -> FIFO0 */
	CAN_FilterTypeDef filter = {0};
	filter.FilterBank           = 0;
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation     = ENABLE;
	filter.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &filter);

	HAL_CAN_Start(&hcan1);

	HAL_Delay(500);   // 모터 전원 대기

	rs_scan();        // 모터 ID 찾기 (결과: rs_id, id_found)

	/* 속도 모드 시작 순서 (매뉴얼 4.3.3)
	 * run_mode=2 -> Enable -> limit_cur -> acc_rad -> spd_ref */
	rs_stop();
	HAL_Delay(20);
	rs_write_u8(RS_IDX_RUN_MODE, 2);
	HAL_Delay(20);
	rs_enable();
	HAL_Delay(20);
	rs_write_float(RS_IDX_LIMIT_CUR, LIMIT_CUR);
	HAL_Delay(20);
	rs_write_float(RS_IDX_ACC_RAD, ACC_RAD);
	HAL_Delay(20);
	rs_write_float(RS_IDX_SPD_REF, 0.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		double cmd = motor_cmd;

		// 범위 제한 (-1 ~ 1)
		if (cmd > 1.0)
		{
			cmd = 1.0;
		}
		if (cmd < -1.0)
		{
			cmd = -1.0;
		}

		// 부호 = 방향, 크기 = 속도
		spd_target = (float)(cmd * SPD_MAX);

		// 20ms마다 속도 명령 전송 (응답 피드백이 계속 와서 fb_count가 늘어남)
		rs_write_float(RS_IDX_SPD_REF, spd_target);

		rs_read_feedback();
		can_esr = CAN1->ESR;   // CAN 에러 상태

		HAL_Delay(20);

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
