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
#include <string.h>
#include "max7219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 发送帧的标准帧 ID：11 位有效，取值 0x000~0x7FF；这里用户自定义，收发两端一致即可 */
#define CAN_HELLO_STDID      0x123U
/* 相邻两帧之间的发送间隔，单位 ms */
#define CAN_HELLO_PERIOD_MS  500U

/* 32 位过滤器寄存器位分配：bit31~21 = STID[10:0]，bit20~3 = EXID[17:0]，
   bit2 = IDE，bit1 = RTR。故标准 ID 在 32 位量中要左移 21 位。
   （常见写法 FilterIdHigh = ID << 5 只是这个值的"高 16 位"，数学上等价） */
#define CAN_FILTER_STDID_SHIFT  21U
/* 只关心 11 位标准 ID：0x7FF << 21 = 0xFFE00000；IDE/RTR/扩展 ID 对应位为 0 → 不参与比较 */
#define CAN_FILTER_STDID_MASK   (0x7FFUL << CAN_FILTER_STDID_SHIFT)
/* 硬件滤波器期望的 ID 值，跟随 CAN_HELLO_STDID 自动更新，避免收发 ID 与滤波器不一致 */
#define CAN_FILTER_STDID_VALUE  ((uint32_t)CAN_HELLO_STDID << CAN_FILTER_STDID_SHIFT)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* 发送帧头：StdId / ExtId / IDE / RTR / DLC 各字段含义见 CAN_SendHello() */
static CAN_TxHeaderTypeDef CAN_TxHeader;
/* 接收帧头：由 HAL_CAN_GetRxMessage() 把收到帧的 ID、DLC 等信息回填到这里 */
static CAN_RxHeaderTypeDef CAN_RxHeader;

/* 发送数据区："Hello" 加上字符串结束符 '\0' 共 6 字节，DLC 取 sizeof-1 = 5 */
static uint8_t CAN_TxData[] = "Hello";
/* 接收缓冲：经典 CAN 一帧数据场最大 8 字节 */
static uint8_t CAN_RxData[8];

/* 运行统计，在调试器 Watch 窗口观察：
     TxCount  = 成功写入发送邮箱的帧数
     RxCount  = 回环收到且内容校验通过的帧数（正常应与 TxCount 同步增长）
     ErrCount = 读取失败或 ID/数据不匹配的次数（正常应保持 0） */
volatile uint32_t CAN_HelloTxCount = 0U;
volatile uint32_t CAN_HelloRxCount = 0U;
volatile uint32_t CAN_HelloErrCount = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_ConfigFilter(void);
static uint8_t CAN_SendHello(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  配置硬件验收滤波器：只放行标准帧 ID = CAN_HELLO_STDID 的报文，
  *         其余 ID 的帧由 bxCAN 硬件丢弃，既不占 FIFO 也不产生中断。
  * @note   FilterId 与 FilterMask 不是"两层掩码"：
  *         硬件只做一次判断 (收到的ID ⊕ FilterId) & FilterMask == 0，
  *         FilterId 提供期望值，FilterMask 决定哪些位参与比较。
  */
static void CAN_ConfigFilter(void)
{
  CAN_FilterTypeDef filter = {0};

  /* FilterBank：使用哪一组硬件滤波器。STM32F103 共 14 组（0~13），
     每组在 32 位模式下是 1 个过滤器，在 16 位模式下是 2 个 */
  filter.FilterBank = 0;

  /* FilterMode：CAN_FILTERMODE_IDMASK = 掩码模式（ID 与掩码逐位比较）
                  CAN_FILTERMODE_IDLIST = 列表模式（列出多个 ID，全部精确匹配） */
  filter.FilterMode = CAN_FILTERMODE_IDMASK;

  /* FilterScale：CAN_FILTERSCALE_32BIT = 每组 1 个 32 位过滤器（可放 1 个扩展 ID 或 1 个标准 ID）
                  CAN_FILTERSCALE_16BIT = 每组 2 个 16 位过滤器 */
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  /* FilterIdHigh / FilterIdLow：32 位过滤器 ID 的高 16 位与低 16 位——
     这是"同一个 32 位期望值"拆成两半，不是两套配置。
     32 位布局：bit31~21 = STID[10:0]，bit20~3 = EXID[17:0]，bit2 = IDE，bit1 = RTR
     这里放标准 ID 0x123 << 21 = 0x24600000，故 High = 0x2460、Low = 0x0000 */
  filter.FilterIdHigh = (uint16_t)(CAN_FILTER_STDID_VALUE >> 16);
  filter.FilterIdLow = (uint16_t)(CAN_FILTER_STDID_VALUE & 0xFFFFUL);

  /* FilterMaskIdHigh / FilterMaskIdLow：同一个 32 位掩码的高/低 16 位，作用是指出"哪些位要比较"：
       掩码位 = 1 → 该位必须与 FilterId 相同
       掩码位 = 0 → 该位不关心（FilterId 对应位写什么都行）
     这里 = 0x7FF << 21 = 0xFFE00000，只比较 11 位标准 ID →
     ID 不等于 0x123 的帧由硬件直接丢弃，CPU 不会收到中断 */
  filter.FilterMaskIdHigh = (uint16_t)(CAN_FILTER_STDID_MASK >> 16);
  filter.FilterMaskIdLow = (uint16_t)(CAN_FILTER_STDID_MASK & 0xFFFFUL);

  /* FilterFIFOAssignment：命中该滤波器的报文进入哪个接收 FIFO，可填 CAN_RX_FIFO0 或 CAN_RX_FIFO1 */
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;

  /* FilterActivation：ENABLE = 启用本组滤波器，DISABLE = 关闭 */
  filter.FilterActivation = ENABLE;

  /* SlaveStartFilterBank：双 CAN（CAN1/CAN2）器件中 CAN2 的起始滤波器组号；
     部分 STM32F103 型号只有 CAN1，此参数无实际作用，保持默认 14 即可 */
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t CAN_SendHello(void)
{
  uint32_t mailbox = 0U;

  /* StdId：标准帧 ID，11 位，范围 0x000~0x7FF */
  CAN_TxHeader.StdId = CAN_HELLO_STDID;

  /* ExtId：扩展帧 ID，29 位，范围 0x00000000~0x1FFFFFFF；发标准帧时该字段不用，填 0 */
  CAN_TxHeader.ExtId = 0U;

  /* IDE：帧 ID 格式，CAN_ID_STD = 标准帧（11 位 ID），CAN_ID_EXT = 扩展帧（29 位 ID） */
  CAN_TxHeader.IDE = CAN_ID_STD;

  /* RTR：帧类型，CAN_RTR_DATA = 数据帧（携带数据），CAN_RTR_REMOTE = 远程请求帧（只请求数据） */
  CAN_TxHeader.RTR = CAN_RTR_DATA;

  /* DLC：数据长度码，有效范围 0~8，表示本帧实际携带的字节数（"Hello" = 5） */
  CAN_TxHeader.DLC = sizeof(CAN_TxData) - 1U;

  /* TransmitGlobalTime：是否在发送时锁存定时器值到时间戳（仅时间触发通信模式使用），
     普通收发填 DISABLE */
  CAN_TxHeader.TransmitGlobalTime = DISABLE;

  /* HAL_CAN_AddTxMessage 参数：
       hcan          : CAN 句柄
       &CAN_TxHeader : 帧头（ID、帧类型、DLC）
       CAN_TxData    : 待发送数据指针，长度由 DLC 决定
       &mailbox      : 输出参数，返回实际占用的发送邮箱编号（0~2），可用于等待该邮箱发送完成
     返回值 HAL_OK 表示已成功放入邮箱（并非已发送完毕）；邮箱满时返回 HAL_ERROR */
  return (HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, CAN_TxData, &mailbox) == HAL_OK) ? 0U : 1U;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t last_refresh = 0U;   /* MAX7219 画面刷新计时 */
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
  /* 滤波器必须在 HAL_CAN_Start() 之前配置好，否则收到的帧会因无匹配滤波器而被丢弃 */
  CAN_ConfigFilter();

  /* HAL_CAN_Start：让 CAN 退出初始化模式进入工作（LISTENING）状态，之后才能收发 */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* HAL_CAN_ActivateNotification 参数 ActiveITs 指定要打开的中断源：
       CAN_IT_RX_FIFO0_MSG_PENDING = FIFO0 收到新报文，触发 HAL_CAN_RxFifo0MsgPendingCallback
     其它常用值：CAN_IT_RX_FIFO1_MSG_PENDING（FIFO1）、CAN_IT_TX_MAILBOX_EMPTY（发送完成）、
                CAN_IT_ERROR / CAN_IT_BUSOFF / CAN_IT_LAST_ERROR_CODE（错误类） */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* MAX7219 串行时钟上限 10 MHz；CubeMX 已把 Prescaler 设为 8（9 MHz） */

  MAX7219_Init(&hspi1);      /* 内部会配置 PA4 片选并完成 MAX7219 寄存器初始化 */

  /* 上电自检：全亮 1 秒。能全亮说明 SPI/片选/接线都通 */
  MAX7219_DisplayTest(1U);
  HAL_Delay(1000U);
  MAX7219_DisplayTest(0U);

  MAX7219_ShowCheck();       /* 显示对号 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 发一帧：返回 0 表示已成功放进发送邮箱；回环模式下该帧会被硬件回灌到接收 FIFO0，
       随后由 HAL_CAN_RxFifo0MsgPendingCallback() 取走并校验 */
    if (CAN_SendHello() == 0U)
    {
      CAN_HelloTxCount++;
    }

    /* 不阻塞 CPU 的延时，单位 ms；期间接收中断照常触发 */
    HAL_Delay(CAN_HELLO_PERIOD_MS);

    /* 每 1 s 重画一次对号：画面更稳，也方便用示波器在 PA5/PA4 上抓到波形 */
    if ((HAL_GetTick() - last_refresh) >= 1000U)
    {
      last_refresh = HAL_GetTick();
      MAX7219_Reinit();               /* 受干扰复位后自动恢复配置 */
      MAX7219_ShowCheck();            /* 每秒重画对号 */
      MAX7219_EchoTest();             /* DOUT 回读自检，结果存 MAX7219_Echo */
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

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 36;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  RX FIFO0 收到新报文时的回调（由 HAL_CAN_IRQHandler 在中断里调用）。
  *         回环模式下收到的正是刚才自己发出的那一帧。
  * @param  hcan: CAN 句柄（等同全局 hcan）
  * @note   DLC 最大 8，接收缓冲 CAN_RxData 已按 8 字节申请
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t dlc = sizeof(CAN_TxData) - 1U;

  /* HAL_CAN_GetRxMessage 参数：
       hcan        : CAN 句柄
       CAN_RX_FIFO0: 从哪个接收 FIFO 取（FIFO0 / FIFO1，需与滤波器配置一致）
       &CAN_RxHeader: 输出参数，回填该帧的 ID、IDE、RTR、DLC、时间戳
       CAN_RxData  : 输出缓冲，HAL 把 DLC 个数据字节写入此处
     读取成功后该帧从 FIFO 中释放；失败说明 FIFO 为空或句柄状态异常 */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData) != HAL_OK)
  {
    CAN_HelloErrCount++;
    return;
  }

  /* 逐项校验：标准帧、数据帧、ID 一致、DLC 一致、数据内容一致。
     硬件滤波器其实已按 ID 过滤过，这里再确认一次，可发现滤波器误配置或 ID 冲突，
     同时覆盖 DLC 与数据内容的检查（硬件滤波器不校验数据场）。 */
  if ((CAN_RxHeader.IDE == CAN_ID_STD) &&
      (CAN_RxHeader.RTR == CAN_RTR_DATA) &&
      (CAN_RxHeader.StdId == CAN_HELLO_STDID) &&
      (CAN_RxHeader.DLC == dlc) &&
      (memcmp(CAN_RxData, CAN_TxData, dlc) == 0))
  {
    CAN_HelloRxCount++;
  }
  else
  {
    CAN_HelloErrCount++;
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
