/**
  ******************************************************************************
  * @file    max7219.c
  * @brief   MAX7219 8x8 LED 点阵驱动
  ******************************************************************************
  * 通信方式：MAX7219 在 LOAD/CS 上升沿锁存数据，每次要连续移入 16 位
  *           D15~D8 = 寄存器地址，D7~D0 = 数据。
  *           SPI1 已是 Mode 0（CPOL=0, CPHA=0）、MSB first、8 位，正好匹配，
  *           所以一次写操作 = 连续 2 个 8 位 SPI 字节，中间保持 CS 为低。
  ******************************************************************************
  */
#include "max7219.h"

/* 对号（√）位图：每个字节 = 一行，bit7 为最左列，bit0 为最右列
     row1: 00000000
     row2: 00000001
     row3: 00000010
     row4: 00000100
     row5: 01001000
     row6: 00110000
     row7: 00100000
     row8: 00000000                                            */
static const uint8_t max7219_check_img[8] =
{
  0x00U, 0x01U, 0x02U, 0x04U, 0x48U, 0x30U, 0x20U, 0x00U
};

static SPI_HandleTypeDef *max7219_spi = NULL;

volatile uint32_t MAX7219_ErrCount = 0U;
volatile uint16_t MAX7219_Echo = 0U;      /* DOUT 回读值：正常应等于回读测试发出的帧 */

/**
  * @brief  向所有级联的 MAX7219 写同一个寄存器
  * @param  reg : 寄存器地址（0x01~0x0F）
  * @param  data: 写入的数据
  * @note   用 TransmitReceive 发送，顺带接收 DOUT 上"上一帧"的回显，便于诊断接线
  */
static void MAX7219_WriteReg(uint8_t reg, uint8_t data)
{
  uint8_t tx[2];
  uint8_t rx[2] = {0U, 0U};
  uint8_t i;

  tx[0] = reg;
  tx[1] = data;

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);

  for (i = 0U; i < MAX7219_CHAIN; i++)
  {
    if (HAL_SPI_TransmitReceive(max7219_spi, tx, rx, 2U, HAL_MAX_DELAY) != HAL_OK)
    {
      MAX7219_ErrCount++;
    }
  }

  /* 上升沿锁存 */
  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);

  MAX7219_Echo = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
}

/**
  * @brief  DOUT 回读自检：先发一帧已知数据，再发 NO-OP 把回显读回来
  * @note   结果存入 MAX7219_Echo，正常应为 0x0A07（即上一帧的内容）
  */
void MAX7219_EchoTest(void)
{
  uint8_t tx[2];
  uint8_t rx[2] = {0U, 0U};

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);

  tx[0] = 0x0AU;      /* 亮度寄存器，写入最坏情况也只是改变亮度，不影响显示内容 */
  tx[1] = 0x07U;
  (void)HAL_SPI_TransmitReceive(max7219_spi, tx, rx, 2U, HAL_MAX_DELAY);

  tx[0] = 0x00U;      /* NO-OP：只为把上一帧的回显移出来 */
  tx[1] = 0x00U;
  (void)HAL_SPI_TransmitReceive(max7219_spi, tx, rx, 2U, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);

  MAX7219_Echo = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
}

/**
  * @brief  写入全部配置寄存器（不含点阵数据）
  * @note   Init 与 Reinit 共用；周期性调用可让 MAX7219 在受干扰复位后自动恢复
  */
static void MAX7219_WriteConfig(void)
{
  MAX7219_WriteReg(MAX7219_REG_DISPLAYTEST, 0x00U);              /* 退出测试模式 */
  MAX7219_WriteReg(MAX7219_REG_DECODE, 0x00U);                   /* 不译码，按点阵控制 */
  MAX7219_WriteReg(MAX7219_REG_SCANLIMIT, 0x07U);                /* 8 行全部扫描 */
  MAX7219_WriteReg(MAX7219_REG_INTENSITY, MAX7219_INTENSITY);    /* 亮度 */
  MAX7219_WriteReg(MAX7219_REG_SHUTDOWN, 0x01U);                 /* 退出关断 */
}

/**
  * @brief  初始化 MAX7219
  * @param  hspi: 已初始化的 SPI 句柄（本工程为 &hspi1）
  * @note   CubeMX 中 NSS 为软件管理，PA4 不会自动配置，这里补上推挽输出。
  */
void MAX7219_Init(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef gpio = {0};

  max7219_spi = hspi;

  if (hspi->Init.NSS == SPI_NSS_SOFT)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 先拉高再配置，避免初始化过程中产生一个假的锁存脉冲 */
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);

    gpio.Pin   = MAX7219_CS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MAX7219_CS_PORT, &gpio);
  }

  /* MISO(PA6) 配成上拉输入：接模块 DOUT 后可用于回读诊断；
     若上拉后仍读到 1，说明该引脚实际悬空（线没接上） */
  gpio.Pin  = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);

  MAX7219_WriteConfig();
  MAX7219_Clear();
}

/**
  * @brief  重写配置寄存器（不动 GPIO）
  * @note   接线接触不良时 MAX7219 可能被复位回关断模式，周期调用可自动恢复显示
  */
void MAX7219_Reinit(void)
{
  MAX7219_WriteConfig();
}

/**
  * @brief  设置亮度
  * @param  level: 0~15，数值越大越亮
  */
void MAX7219_SetIntensity(uint8_t level)
{
  MAX7219_WriteReg(MAX7219_REG_INTENSITY, (uint8_t)(level & 0x0FU));
}

/**
  * @brief  显示测试模式：enable=1 时 64 个点全亮（不受亮度/扫描/数据影响）
  * @param  enable: 1 = 进入测试模式，0 = 恢复正常显示
  * @note   用来区分"软件/接线正常但画面不对"与"完全没通信"
  */
void MAX7219_DisplayTest(uint8_t enable)
{
  MAX7219_WriteReg(MAX7219_REG_DISPLAYTEST, (enable != 0U) ? 0x01U : 0x00U);
}

/**
  * @brief  清屏（8 行全灭）
  */
void MAX7219_Clear(void)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    MAX7219_WriteReg((uint8_t)(MAX7219_REG_DIGIT0 + i), 0x00U);
  }
}

#if MAX7219_FLIP_HORIZONTAL
static uint8_t MAX7219_ReverseBits(uint8_t v)
{
  v = (uint8_t)(((v & 0xF0U) >> 4) | ((v & 0x0FU) << 4));
  v = (uint8_t)(((v & 0xCCU) >> 2) | ((v & 0x33U) << 2));
  v = (uint8_t)(((v & 0xAAU) >> 1) | ((v & 0x55U) << 1));
  return v;
}
#endif

/**
  * @brief  显示一幅 8x8 位图
  * @param  image: 8 字节，image[0] 对应最上面一行，bit7 为最左列
  * @note   实际写入时按 MAX7219_FLIP_VERTICAL / MAX7219_FLIP_HORIZONTAL 修正方向
  */
void MAX7219_ShowImage(const uint8_t image[8])
{
  uint8_t i;
  uint8_t row;
  uint8_t val;

  for (i = 0U; i < 8U; i++)
  {
    row = i;
    val = image[i];

#if MAX7219_FLIP_VERTICAL
    row = (uint8_t)(7U - i);
#endif
#if MAX7219_FLIP_HORIZONTAL
    val = MAX7219_ReverseBits(val);
#endif

    MAX7219_WriteReg((uint8_t)(MAX7219_REG_DIGIT0 + row), val);
  }
}

/**
  * @brief  显示一个对号（√）
  */
void MAX7219_ShowCheck(void)
{
  MAX7219_ShowImage(max7219_check_img);
}
