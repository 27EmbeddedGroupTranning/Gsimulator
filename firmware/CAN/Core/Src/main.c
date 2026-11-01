/**
  ******************************************************************************
  * @file    main.c
  * @brief   Peripheral self test firmware for the Gsimulator STM32F103 model.
  ******************************************************************************
  * `st_run_selftest()` drives every on-chip block through its registers and
  * prints one line per block on USART1. This file then brings up the parts
  * that are wired to the MCU on the schematic:
  *
  *   USART1  PA9 / PA10   report + terminal echo
  *   USART2  PA2 / PA3    second terminal (one test byte at boot)
  *   USART3  PB10 / PB11  third terminal (one test byte at boot)
  *   SPI1    PA5 / PA6 / PA7, PA4 = LOAD   MAX7219 8x8 matrix
  *   CAN1    PA12 / PA11  CAN transceiver, virtual bus, termination resistor
  *   I2C1    PB6 / PB7    (no part answers: the self test expects a NACK)
  *   LED     PB12         blinks while the firmware runs
  *   Encoder PB0 / PB1 / PB2 (A / B / switch)  quadrature counter
  *
  * and keeps them alive in `board_loop()`.
  ******************************************************************************
  */
#include "main.h"
#include "board.h"
#include "max7219.h"
#include "selftest.h"

/* ---- pins of the interactive part ---------------------------------------- */
#define BOARD_LED_PIN 12u
#define BOARD_ENCODER_A 0u
#define BOARD_ENCODER_B 1u
#define BOARD_ENCODER_SW 2u

/* ---- timings ------------------------------------------------------------- */
#define BOARD_LED_PERIOD_MS 250u
#define BOARD_CAN_PERIOD_MS 500u
#define BOARD_CAN_REPORT_MS 1000u
#define BOARD_MATRIX_PERIOD_MS 1000u
#define BOARD_LOOP_PERIOD_MS 2u

/* ---- CAN frames ---------------------------------------------------------- */
#define BOARD_CAN_ID 0x123u
#define BOARD_CAN_LENGTH 5u

#define RX_RING 64u

static volatile uint8_t s_rx_ring[RX_RING];
static volatile uint32_t s_rx_head;
static volatile uint32_t s_rx_tail;
static volatile uint32_t s_rx_count;

static volatile int32_t s_encoder_steps;
static volatile uint32_t s_encoder_presses;
static uint8_t s_encoder_phase;

static uint8_t s_matrix_pattern;

/* Second picture drawn on the matrix: a large "X". */
static const uint8_t s_matrix_cross[8] =
{
  0x81U, 0x42U, 0x24U, 0x18U, 0x18U, 0x24U, 0x42U, 0x81U
};

/* ------------------------------------------------------------------------- */
/* Encoder (PB0 / PB1 / PB2)                                                 */
/* ------------------------------------------------------------------------- */

/* Phase index in clockwise order: 11, 01, 00, 10. */
static uint8_t board_encoder_phase(void)
{
  static const uint8_t grey_index[4] = {2u, 1u, 3u, 0u};
  uint32_t idr = GPIOB->IDR;
  uint8_t a = (uint8_t)((idr >> BOARD_ENCODER_A) & 1u);
  uint8_t b = (uint8_t)((idr >> BOARD_ENCODER_B) & 1u);

  return grey_index[(a << 1) | b];
}

void board_encoder_irq(void)
{
  EXTI->PR = (1u << BOARD_ENCODER_A) | (1u << BOARD_ENCODER_B);

  uint8_t phase = board_encoder_phase();
  uint8_t delta = (uint8_t)((phase + 4u - s_encoder_phase) & 3u);

  if (delta == 1u)
  {
    s_encoder_steps++;
  }
  else if (delta == 3u)
  {
    s_encoder_steps--;
  }
  s_encoder_phase = phase;
}

void board_button_irq(void)
{
  EXTI->PR = 1u << BOARD_ENCODER_SW;
  s_encoder_presses++;
}

/* ------------------------------------------------------------------------- */
/* UART terminal                                                             */
/* ------------------------------------------------------------------------- */

void board_uart1_irq(void)
{
  if ((USART1->SR & USART_SR_RXNE) != 0u)
  {
    uint8_t byte = (uint8_t)USART1->DR;
    uint32_t next = (s_rx_head + 1u) % RX_RING;

    if (next != s_rx_tail)
    {
      s_rx_ring[s_rx_head] = byte;
      s_rx_head = next;
    }
  }
}

/* ------------------------------------------------------------------------- */
/* Board bring up                                                            */
/* ------------------------------------------------------------------------- */

static void board_gpio_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

  st_gpio_output_pp(GPIOB, BOARD_LED_PIN);
  GPIOB->BRR = 1u << BOARD_LED_PIN;
}

static void board_encoder_init(void)
{
  st_gpio_input_pull(GPIOB, BOARD_ENCODER_A, 1u);
  st_gpio_input_pull(GPIOB, BOARD_ENCODER_B, 1u);
  st_gpio_input_pull(GPIOB, BOARD_ENCODER_SW, 1u);
  s_encoder_phase = board_encoder_phase();

  /* EXTI0..EXTI3 are taken from port B. */
  AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~0x0000FFFFu) | 0x00001111u;

  EXTI->RTSR |= (1u << BOARD_ENCODER_A) | (1u << BOARD_ENCODER_B);
  EXTI->FTSR |= (1u << BOARD_ENCODER_A) | (1u << BOARD_ENCODER_B) | (1u << BOARD_ENCODER_SW);
  EXTI->IMR |= (1u << BOARD_ENCODER_A) | (1u << BOARD_ENCODER_B) | (1u << BOARD_ENCODER_SW);

  NVIC_SetPriority(EXTI0_IRQn, 0x60u);
  NVIC_SetPriority(EXTI1_IRQn, 0x60u);
  NVIC_SetPriority(EXTI2_IRQn, 0x60u);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_EnableIRQ(EXTI2_IRQn);
}

static void board_console_init(void)
{
  /* USART1 already runs at 115200 8N1 with the receive interrupt enabled. */
  NVIC_SetPriority(USART1_IRQn, 0x70u);
  NVIC_EnableIRQ(USART1_IRQn);
}

static void board_matrix_init(void)
{
  uint16_t echo = 0u;

  MAX7219_Init();
  MAX7219_ShowCheck();

  st_console_puts("DEV MATRIX=");
  if (MAX7219_Detect(&echo) != 0)
  {
    st_console_puts("OK ECHO=");
  }
  else
  {
    st_console_puts("NA ECHO=");
  }
  st_console_hex(echo);
  st_console_newline();
}

/* ------------------------------------------------------------------------- */
/* CAN bus                                                                   */
/* ------------------------------------------------------------------------- */

static void board_can_transmit(void)
{
  static const char payload[BOARD_CAN_LENGTH] = {'H', 'e', 'l', 'l', 'o'};

  if ((CAN1->TSR & CAN_TSR_TME0) == 0u)
  {
    return; /* the previous frame is still in flight */
  }

  CAN1->sTxMailBox[0].TDTR = BOARD_CAN_LENGTH;
  CAN1->sTxMailBox[0].TDLR = (uint32_t)payload[0]
                           | ((uint32_t)payload[1] << 8)
                           | ((uint32_t)payload[2] << 16)
                           | ((uint32_t)payload[3] << 24);
  CAN1->sTxMailBox[0].TDHR = (uint32_t)payload[4];
  CAN1->sTxMailBox[0].TIR = (BOARD_CAN_ID << 21) | CAN_TI0R_TXRQ;
}

static void board_can_drain(void)
{
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0u)
  {
    uint32_t identifier = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FFu;
    uint32_t length = CAN1->sFIFOMailBox[0].RDTR & 0xFu;
    CAN1->RF0R = CAN_RF0R_RFOM0;

    st_console_puts("EVT CAN RX ");
    st_console_hex(identifier);
    st_console_puts(" ");
    st_console_u32(length);
    st_console_newline();
  }
}

static void board_can_report(void)
{
  uint32_t esr = CAN1->ESR;

  st_console_puts("EVT CAN STATE TEC=");
  st_console_u32((esr & CAN_ESR_TEC) >> 16);
  st_console_puts(" REC=");
  st_console_u32((esr & CAN_ESR_REC) >> 24);
  st_console_puts(" IRQ=");
  st_console_u32(st_can_irq_count());
  st_console_newline();
}

/* ------------------------------------------------------------------------- */
/* LED matrix animation                                                      */
/* ------------------------------------------------------------------------- */

static void board_matrix_step(void)
{
  s_matrix_pattern ^= 1u;
  MAX7219_Reinit();
  if (s_matrix_pattern == 0u)
  {
    MAX7219_ShowCheck();
  }
  else
  {
    MAX7219_ShowImage(s_matrix_cross);
  }

  st_console_puts("EVT MATRIX ");
  st_console_u32(s_matrix_pattern);
  st_console_newline();
}

/* ------------------------------------------------------------------------- */
/* Terminal echo                                                             */
/* ------------------------------------------------------------------------- */

static void board_uart_poll(void)
{
  uint32_t received = 0u;

  while (s_rx_tail != s_rx_head)
  {
    uint8_t byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1u) % RX_RING;
    s_rx_count++;
    received++;

    /* Wait for room in the transmitter, then echo the character back. */
    for (uint32_t guard = 0u; guard < 100000u; guard++)
    {
      if ((USART1->SR & USART_SR_TXE) != 0u)
      {
        break;
      }
    }
    USART1->DR = (uint32_t)byte;
  }

  if (received != 0u)
  {
    st_console_puts("EVT UART1 RX ");
    st_console_u32(s_rx_count);
    st_console_newline();
  }
}

/* ------------------------------------------------------------------------- */
/* Entry points                                                              */
/* ------------------------------------------------------------------------- */

void board_init(void)
{
  board_gpio_init();
  board_console_init();
  board_encoder_init();

  /* The self test armed the watchdogs; keep them alive from here on. */
  st_service_watchdogs();

  board_matrix_init();

  st_console_puts("LOOP");
  st_console_newline();
  st_console_flush();
}

void board_loop(void)
{
  uint32_t last_led = st_millis();
  uint32_t last_can = st_millis() - BOARD_CAN_PERIOD_MS;
  uint32_t last_can_report = st_millis();
  uint32_t last_matrix = st_millis();
  uint32_t last_lse_check = st_millis();
  int32_t last_steps = 0;
  uint32_t last_presses = 0u;
  uint32_t lse_reported = 0u;

  /* One line the host test can wait for to know the loop is alive. */
  st_console_puts("EVT READY");
  st_console_newline();
  st_console_flush();

  if ((RCC->BDCR & RCC_BDCR_LSERDY) != 0u)
  {
    lse_reported = 1u;
  }

  for (;;)
  {
    uint32_t now = st_millis();

    st_service_watchdogs();

    if ((now - last_led) >= BOARD_LED_PERIOD_MS)
    {
      last_led = now;
      if ((GPIOB->ODR & (1u << BOARD_LED_PIN)) != 0u)
      {
        GPIOB->BRR = 1u << BOARD_LED_PIN;
      }
      else
      {
        GPIOB->BSRR = 1u << BOARD_LED_PIN;
      }
    }

    if ((now - last_can) >= BOARD_CAN_PERIOD_MS)
    {
      last_can = now;
      board_can_transmit();
    }

    board_can_drain();

    if ((now - last_can_report) >= BOARD_CAN_REPORT_MS)
    {
      last_can_report = now;
      board_can_report();
    }

    if ((now - last_matrix) >= BOARD_MATRIX_PERIOD_MS)
    {
      last_matrix = now;
      board_matrix_step();
    }

    if (lse_reported == 0u && (now - last_lse_check) >= 10u)
    {
      last_lse_check = now;
      if ((RCC->BDCR & RCC_BDCR_LSERDY) != 0u)
      {
        lse_reported = 1u;
        st_console_puts("EVT LSE READY");
        st_console_newline();
      }
    }

    board_uart_poll();

    int32_t steps = s_encoder_steps;
    uint32_t presses = s_encoder_presses;
    if (steps != last_steps || presses != last_presses)
    {
      last_steps = steps;
      last_presses = presses;
      st_console_puts("EVT ENC ");
      if (steps < 0)
      {
        st_console_puts("-");
      }
      st_console_u32((uint32_t)((steps < 0) ? -steps : steps));
      st_console_puts(" ");
      st_console_u32(presses);
      st_console_newline();
    }

    st_delay_ms(BOARD_LOOP_PERIOD_MS);
  }
}

int main(void)
{
  st_clock_t clock;

  st_clock_init(&clock);
  st_console_init(clock.pclk2_hz);
  st_run_selftest(&clock);
  board_init();
  board_loop();
  return 0;
}

void Error_Handler(void)
{
  __disable_irq();
  for (;;)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
