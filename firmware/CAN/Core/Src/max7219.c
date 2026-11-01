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
#include "selftest.h"

/* Check mark: one byte per row, bit7 is the leftmost column. */
static const uint8_t max7219_check_img[8] =
{
  0x00U, 0x01U, 0x02U, 0x04U, 0x48U, 0x30U, 0x20U, 0x00U
};

static const uint8_t s_echo_word = MAX7219_REG_INTENSITY;

volatile uint32_t MAX7219_ErrCount = 0U;

/* Shift one byte and return what came back on MISO. */
static uint8_t max7219_transfer(uint8_t value)
{
  for (uint32_t guard = 0U; guard < 100000U; guard++)
  {
    if ((SPI1->SR & SPI_SR_TXE) != 0U)
    {
      break;
    }
  }
  SPI1->DR = (uint32_t)value;

  for (uint32_t guard = 0U; guard < 100000U; guard++)
  {
    if ((SPI1->SR & SPI_SR_RXNE) != 0U)
    {
      return (uint8_t)SPI1->DR;
    }
  }
  MAX7219_ErrCount++;
  return 0U;
}

static void max7219_write(uint8_t address, uint8_t data)
{
  MAX7219_CS_PORT->BRR = 1U << MAX7219_CS_PIN;
  (void)max7219_transfer(address);
  (void)max7219_transfer(data);
  MAX7219_CS_PORT->BSRR = 1U << MAX7219_CS_PIN; /* latch */
}

/* Shift a NO-OP word and return the 16 bits the chain echoed back. */
static uint16_t max7219_read_back(void)
{
  uint16_t high;
  uint16_t low;

  MAX7219_CS_PORT->BRR = 1U << MAX7219_CS_PIN;
  high = max7219_transfer(MAX7219_REG_NOOP);
  low = max7219_transfer(MAX7219_REG_NOOP);
  MAX7219_CS_PORT->BSRR = 1U << MAX7219_CS_PIN;

  return (uint16_t)((uint16_t)(high << 8) | low);
}

int MAX7219_Detect(uint16_t *echo)
{
  /* Write the intensity register again (a harmless word), give the harness time
     to hand the transfer to the part (it services the devices between
     simulation slices) and then read the word it shifted out on DOUT. */
  max7219_write(s_echo_word, MAX7219_INTENSITY);
  st_delay_ms(5U);

  *echo = max7219_read_back();
  return (*echo >> 8) == s_echo_word || (*echo & 0xFFU) == s_echo_word;
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
  /* The SPI block and PA4..PA7 were configured by the self test. */
  MAX7219_CS_PORT->BSRR = 1U << MAX7219_CS_PIN;
  MAX7219_ErrCount = 0U;
  max7219_write_config();
  MAX7219_Clear();
}

void MAX7219_Reinit(void)
{
  max7219_write_config();
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

void MAX7219_ShowCheck(void)
{
  MAX7219_ShowImage(max7219_check_img);
}
