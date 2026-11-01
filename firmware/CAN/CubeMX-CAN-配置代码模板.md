> 说明：本文是**用 HAL 写 CAN 外设**的 CubeMX 参考模板，保留作对照。
> 工程里当前的 Core/Src/main.c 已改成不依赖 HAL 的全外设自检固件
> （见仓库根目录 README.md 第 1 节“运行真实 ELF”），CAN1 的寄存器序列在
> Core/Src/selftest.c::st_test_can() 里。
# CubeMX 配好工程后：CAN 代码模板

> 前提：CubeMX 已勾选 CAN1，并已使能 `USB low priority or CAN1 RX0` 中断。
> 代码只贴在 `USER CODE BEGIN/END` 之间。

## 32 位过滤器布局

| 位 | 31~21 | 20~3 | 2 | 1 |
|---|---|---|---|---|
| 内容 | STID[10:0] | EXID[17:0] | IDE | RTR |

硬件只做一次判断：`(收到的ID ⊕ FilterId) & FilterMask == 0`
- `FilterId` = 期望值（32 位量，`High` = 高 16 位，`Low` = 低 16 位）
- `FilterMask` = 关心位，1 才参与比较

波特率：`PCLK1 / (Prescaler × (1 + BS1 + BS2))`

## 1. 定义 `USER CODE PD`

```c
#define CAN_STDID        0x123U
#define CAN_PERIOD_MS    500U
```

## 2. 变量 `USER CODE PV`

```c
static CAN_RxHeaderTypeDef CAN_RxHeader;
static uint8_t CAN_RxData[8];

volatile uint32_t CAN_TxCount = 0U;
volatile uint32_t CAN_RxCount = 0U;
```

## 3. 滤波器 + 发送 `USER CODE 0`

```c
static void CAN_FilterInit(uint32_t std_id)
{
  CAN_FilterTypeDef f = {0};
  uint32_t v = (std_id & 0x7FFU) << 21;   /* 期望值 */
  uint32_t m = 0x7FFUL << 21;             /* 只比较 11 位标准 ID */

  f.FilterBank           = 0;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = (uint16_t)(v >> 16);
  f.FilterIdLow          = (uint16_t)v;
  f.FilterMaskIdHigh     = (uint16_t)(m >> 16);
  f.FilterMaskIdLow      = (uint16_t)m;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t CAN_Send(uint32_t std_id, uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mailbox;

  tx.StdId = std_id & 0x7FFU;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = len;

  return (HAL_CAN_AddTxMessage(&hcan, &tx, data, &mailbox) == HAL_OK) ? 0U : 1U;
}
```

## 4. 启动 `USER CODE 2`

```c
  CAN_FilterInit(CAN_STDID);

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
```

## 5. 接收回调 `USER CODE 4`

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData) != HAL_OK)
  {
    return;
  }

  CAN_RxCount++;
}
```

## 6. 主循环 `USER CODE 3`

```c
    uint8_t msg[] = "Hello";

    if (CAN_Send(CAN_STDID, msg, sizeof(msg) - 1U) == 0U)
    {
      CAN_TxCount++;
    }
    HAL_Delay(CAN_PERIOD_MS);
```

## 注意

- 顺序固定：`HAL_CAN_Init`（CubeMX 已做）→ `CAN_FilterInit` → `HAL_CAN_Start` → `ActivateNotification`
- `HAL_CAN_AddTxMessage` 返回 `HAL_OK` 只表示进了邮箱，不代表已发出
- 单机自测把 Mode 设为 `CAN_MODE_LOOPBACK`，不需要收发器
- 正常模式总线上必须有第二个节点应答 ACK
