/**
  ******************************************************************************
  * @file    board.h
  * @brief   Interactive part of the test firmware: parts wired to the MCU.
  ******************************************************************************
  */
#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Configures the LED, the encoder, the LED matrix and the CAN bus, prints the
   `DEV` line of the report and returns. */
void board_init(void);

/* Never returns: services the devices, prints `EVT` lines and keeps the
   watchdogs alive. */
void board_loop(void);

/* Interrupt entry points used by stm32f1xx_it.c */
void board_encoder_irq(void);
void board_button_irq(void);
void board_uart1_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
