/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt service routines of the peripheral self test firmware.
  ******************************************************************************
  * Every handler is a thin shell: the work lives in the module that owns the
  * state it touches (`selftest.c` for the watchdogs/EXTI test line,
  * `main.c` for the encoder, the terminal and the CAN controller).
  ******************************************************************************
  */
#include "main.h"
#include "board.h"
#include "selftest.h"
#include "stm32f1xx_it.h"

/* ------------------------------------------------------------------------- */
/* Cortex-M3 exceptions                                                      */
/* ------------------------------------------------------------------------- */

void NMI_Handler(void)
{
  for (;;)
  {
  }
}

void HardFault_Handler(void)
{
  for (;;)
  {
  }
}

void MemManage_Handler(void)
{
  for (;;)
  {
  }
}

void BusFault_Handler(void)
{
  for (;;)
  {
  }
}

void UsageFault_Handler(void)
{
  for (;;)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  st_tick();
}

/* ------------------------------------------------------------------------- */
/* Peripheral interrupts                                                     */
/* ------------------------------------------------------------------------- */

void EXTI0_IRQHandler(void)
{
  board_encoder_irq();
}

void EXTI1_IRQHandler(void)
{
  board_encoder_irq();
}

void EXTI2_IRQHandler(void)
{
  board_button_irq();
}

void EXTI3_IRQHandler(void)
{
  st_exti_line_irq();
}

void USART1_IRQHandler(void)
{
  board_uart1_irq();
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  st_can_rx_irq();
}
