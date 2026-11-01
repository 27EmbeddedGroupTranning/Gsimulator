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
#include "can.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max7219.h"
#include "can_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Peak speed of the sine sweep, in rpm: the shaft runs forward, coasts through
   zero and runs backwards, so the sign of the command carries the direction. */
#define CAN_SWING_RPM         1000

/* One full sine period, in milliseconds. */
#define CAN_SINE_PERIOD_MS    4000U

/* How often a command frame is queued: 50 frames per second, far below the
   200 kbit/s the pair runs at, and fast enough to look continuous. */
#define CAN_SEND_PERIOD_MS    20U

/* Points of the sine table; must be a power of two divisor of the period. */
#define CAN_SINE_POINTS       64U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* sin(2*pi*i / 64) in Q15, so the sweep needs no floating point in the loop. */
static const int16_t can_sine_q15[CAN_SINE_POINTS] =
{
      0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
  23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
  32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
  23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
      0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
 -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
 -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
 -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212,
};

/* Target speed of the sweep at `elapsed_ms`, in rpm. */
static int16_t can_sine_rpm(uint32_t elapsed_ms)
{
  uint32_t phase = ((elapsed_ms % CAN_SINE_PERIOD_MS) * CAN_SINE_POINTS)
                   / CAN_SINE_PERIOD_MS;
  int32_t scaled = ((int32_t)can_sine_q15[phase] * CAN_SWING_RPM) / 32767;

  return (int16_t)scaled;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* SPI1 drives a single MAX7219 8x8 module: light it up and draw a heart. */
  MAX7219_Init();
  MAX7219_ShowHeart();

  /* CAN1 commands the motor of the bench (standard frame 0x200). */
  CAN_Motor_Init();

  /* Clock of the speed sweep. */
  uint32_t sweep_start = HAL_GetTick();
  uint32_t next_frame = sweep_start;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* One command frame every CAN_SEND_PERIOD_MS, so the shaft follows a
       continuous sine instead of stepping between a few fixed speeds. */
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_frame) >= 0)
    {
      int16_t rpm = can_sine_rpm(now - sweep_start);

      (void)CAN_Motor_SetSpeedRpm(rpm);

      /* The heart glows with the speed: still at a standstill, brightest at
         full swing. */
      uint32_t magnitude = (rpm < 0) ? (uint32_t)(-rpm) : (uint32_t)rpm;
      uint32_t level = 7U + (magnitude * 8U) / CAN_SWING_RPM;
      MAX7219_SetIntensity((uint8_t)((level > 15U) ? 15U : level));

      next_frame += CAN_SEND_PERIOD_MS;
      if ((int32_t)(now - next_frame) >= 0)
      {
        /* A long HAL_Delay (or a debug halt) made us late: resynchronise
           instead of firing a burst of catch-up frames. */
        next_frame = now + CAN_SEND_PERIOD_MS;
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
