/**
  ******************************************************************************
  * @file    selftest.h
  * @brief   Register level self test for every peripheral the simulator models.
  ******************************************************************************
  * The test runs straight after reset, prints one line per peripheral on
  * USART1 and leaves the device in a usable state for the interactive part of
  * the firmware (encoder, LED matrix, CAN bus, UART terminal).
  ******************************************************************************
  */
#ifndef __SELFTEST_H
#define __SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Baud rate of the report on USART1 (PA9 = TX, PA10 = RX). */
#define ST_CONSOLE_BAUD 115200u

/* Crystal fitted on OSC_IN/OSC_OUT by the board model. */
#define ST_HSE_HZ 8000000u
/* PLL multiplier used when the crystal answers, 8 MHz * 9 = 72 MHz. */
#define ST_PLL_MULTIPLIER 9u

/** Clock tree the self test brought up. */
typedef struct
{
  uint32_t hclk_hz;
  uint32_t pclk1_hz;
  uint32_t pclk2_hz;
  uint8_t hse_ready;
  uint8_t pll_used;
} st_clock_t;

/* ---- clock tree and SysTick ---------------------------------------------- */
void st_clock_init(st_clock_t *clock);
void st_systick_init(uint32_t hclk_hz);

/* ---- console (USART1) ---------------------------------------------------- */
void st_console_init(uint32_t pclk2_hz);
void st_console_puts(const char *text);
void st_console_u32(uint32_t value);
void st_console_hex(uint32_t value);
void st_console_newline(void);
void st_console_flush(void);

/* ---- time ---------------------------------------------------------------- */
uint32_t st_millis(void);
void st_delay_ms(uint32_t milliseconds);

/* ---- GPIO helpers shared with the application part ----------------------- */
void st_gpio_output_pp(GPIO_TypeDef *port, uint32_t pin);
void st_gpio_af_pp(GPIO_TypeDef *port, uint32_t pin);
void st_gpio_af_od(GPIO_TypeDef *port, uint32_t pin);
void st_gpio_input_pull(GPIO_TypeDef *port, uint32_t pin, uint8_t pull_up);

/* ---- interrupt entry points (called from stm32f1xx_it.c) ----------------- */
void st_tick(void);
void st_exti_line_irq(void);
void st_can_rx_irq(void);
uint32_t st_can_irq_count(void);

/* ---- the self test ------------------------------------------------------- */
void st_run_selftest(const st_clock_t *clock);
uint32_t st_failures(void);

/* Watchdogs started by the self test; the application keeps them alive. */
void st_service_watchdogs(void);

#ifdef __cplusplus
}
#endif

#endif /* __SELFTEST_H */
