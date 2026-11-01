/**
  ******************************************************************************
  * @file    max7219.h
  * @brief   MAX7219 8x8 LED matrix driver on SPI1 (software chip select).
  ******************************************************************************
  * The CubeMX project configures SPI1 as master, mode 0, 8 bit, MSB first
  * (PA5 = SCK, PA6 = MISO, PA7 = MOSI). Only LOAD/CS is missing, so the driver
  * takes PA4 as a plain push-pull output:
  *
  *   SCK   -> PA5  (SPI1_SCK)
  *   DIN   -> PA7  (SPI1_MOSI)
  *   DOUT  -> PA6  (SPI1_MISO, optional: used to read the echo back)
  *   LOAD  -> PA4  (software chip select)
  *   VCC   -> 5 V (or 3.3 V on modules with an on-board regulator)
  *   GND   -> GND
  ******************************************************************************
  */
#ifndef __MAX7219_H
#define __MAX7219_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---- chip select ---- */
#define MAX7219_CS_PORT        GPIOA
#define MAX7219_CS_PIN         GPIO_PIN_4

/* Number of chained dies: one 8x8 module = 1, a 4-in-1 module = 4. */
#define MAX7219_CHAIN          1u

/* Brightness 0..15 */
#define MAX7219_INTENSITY      0x0Au

/* ---- row / column orientation ----
     MAX7219_FLIP_VERTICAL   = 1 -> DIGIT0 is the bottom row (upside-down image)
     MAX7219_FLIP_HORIZONTAL = 1 -> bit7 is the rightmost column (mirrored image) */
#define MAX7219_FLIP_VERTICAL     0
#define MAX7219_FLIP_HORIZONTAL   0

/* ---- register addresses ---- */
#define MAX7219_REG_NOOP        0x00u
#define MAX7219_REG_DIGIT0      0x01u  /* DIGIT0..DIGIT7 are rows 1..8 */
#define MAX7219_REG_DECODE      0x09u
#define MAX7219_REG_INTENSITY   0x0Au
#define MAX7219_REG_SCANLIMIT   0x0Bu
#define MAX7219_REG_SHUTDOWN    0x0Cu
#define MAX7219_REG_DISPLAYTEST 0x0Fu

/* Configures PA4, leaves the display enabled and blank. */
void MAX7219_Init(void);

void MAX7219_SetIntensity(uint8_t level);
void MAX7219_DisplayTest(uint8_t enable);
void MAX7219_Clear(void);

/* Writes all eight digits: image[0] is the top row, bit7 the leftmost column. */
void MAX7219_ShowImage(const uint8_t image[8]);

/* Draws the 8x8 heart the demo shows. */
void MAX7219_ShowHeart(void);

/* Number of failed SPI transfers (should stay 0). */
extern volatile uint32_t MAX7219_ErrCount;

#ifdef __cplusplus
}
#endif

#endif /* __MAX7219_H */
