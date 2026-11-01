/**
  ******************************************************************************
  * @file    max7219.c
  * @brief   MAX7219 8x8 LED matrix driver.
  ******************************************************************************
  * The controller latches a 16 bit word on the rising edge of LOAD: D15..D8 is
  * the register address, D7..D0 the data. SPI1 runs in mode 0, MSB first, 8 bit
  * frames, so one register write is two back-to-back bytes with LOAD held low.
  ******************************************************************************
  */
#include "max7219.h"
#include "spi.h"

/* SPI timeout for one register write (two bytes at 9 Mbit/s). */
#define MAX7219_TIMEOUT_MS 10U

/* Filled heart: one byte per row, bit7 is the leftmost column. */
static const uint8_t max7219_heart_img[8] =
{
  0x66U, 0xFFU, 0xFFU, 0xFFU, 0x7EU, 0x3CU, 0x18U, 0x00U
};

volatile uint32_t MAX7219_ErrCount = 0U;

/* LOAD low while the 16 bit word is shifted out, then the rising edge latches. */
static void max7219_write(uint8_t address, uint8_t data)
{
  uint8_t frame[2] = { address, data };

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, frame, sizeof(frame), MAX7219_TIMEOUT_MS) != HAL_OK)
  {
    MAX7219_ErrCount++;
  }
  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);
}

static void max7219_write_config(void)
{
  max7219_write(MAX7219_REG_DISPLAYTEST, 0x00U);
  max7219_write(MAX7219_REG_DECODE, 0x00U);
  max7219_write(MAX7219_REG_SCANLIMIT, 0x07U);
  max7219_write(MAX7219_REG_INTENSITY, MAX7219_INTENSITY);
  max7219_write(MAX7219_REG_SHUTDOWN, 0x01U);
}

void MAX7219_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* SPI1 does not drive a hardware NSS pin in this project: LOAD is a plain
     GPIO that idles high between transactions. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = MAX7219_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MAX7219_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);

  MAX7219_ErrCount = 0U;
  max7219_write_config();
  MAX7219_Clear();
}

void MAX7219_SetIntensity(uint8_t level)
{
  max7219_write(MAX7219_REG_INTENSITY, (uint8_t)(level & 0x0FU));
}

void MAX7219_DisplayTest(uint8_t enable)
{
  max7219_write(MAX7219_REG_DISPLAYTEST, (enable != 0U) ? 0x01U : 0x00U);
}

void MAX7219_Clear(void)
{
  for (uint8_t row = 0U; row < 8U; row++)
  {
    max7219_write((uint8_t)(MAX7219_REG_DIGIT0 + row), 0x00U);
  }
}

#if MAX7219_FLIP_HORIZONTAL
static uint8_t max7219_reverse_bits(uint8_t value)
{
  value = (uint8_t)(((value & 0xF0U) >> 4) | ((value & 0x0FU) << 4));
  value = (uint8_t)(((value & 0xCCU) >> 2) | ((value & 0x33U) << 2));
  value = (uint8_t)(((value & 0xAAU) >> 1) | ((value & 0x55U) << 1));
  return value;
}
#endif

void MAX7219_ShowImage(const uint8_t image[8])
{
  for (uint8_t index = 0U; index < 8U; index++)
  {
    uint8_t row = index;
    uint8_t value = image[index];

#if MAX7219_FLIP_VERTICAL
    row = (uint8_t)(7U - index);
#endif
#if MAX7219_FLIP_HORIZONTAL
    value = max7219_reverse_bits(value);
#endif

    max7219_write((uint8_t)(MAX7219_REG_DIGIT0 + row), value);
  }
}

void MAX7219_ShowHeart(void)
{
  MAX7219_ShowImage(max7219_heart_img);
}
