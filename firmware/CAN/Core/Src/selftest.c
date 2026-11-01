/**
  ******************************************************************************
  * @file    selftest.c
  * @brief   Register level self test for every modelled STM32F103 peripheral.
  ******************************************************************************
  * Every block the simulator implements is exercised through the same
  * register interface a real firmware would use, and one line per block is
  * printed on USART1:
  *
  *   GSIM-SELFTEST 1
  *   CLK 72000000 HSE
  *   SYSTICK OK
  *   GPIO OK
  *   ...
  *   ONCHIP 23/23
  *
  * The report is deliberately plain ASCII and byte stable so the host test in
  * `crates/gsim-cpu/tests/selftest_firmware.rs` can assert on it. Lines that
  * depend on a part being wired to the board (matrix, encoder, CAN peer) are
  * reported by the application part in `main.c` instead.
  ******************************************************************************
  */
#include "selftest.h"

/* ------------------------------------------------------------------------- */
/* Console                                                                   */
/* ------------------------------------------------------------------------- */

static volatile uint32_t s_millis;
static uint32_t s_console_divider;

void st_tick(void)
{
  s_millis++;
}

/* `BRR` value for the configured baud rate, with the same rounding the
   console uses, so the register can be checked back. */
static uint32_t st_console_brr(void)
{
  return s_console_divider;
}

uint32_t st_millis(void)
{
  return s_millis;
}

static void st_putc(uint8_t byte)
{
  while ((USART1->SR & USART_SR_TXE) == 0u)
  {
  }
  USART1->DR = (uint32_t)byte;
}

void st_console_puts(const char *text)
{
  while (*text != '\0')
  {
    st_putc((uint8_t)*text++);
  }
}

void st_console_u32(uint32_t value)
{
  char digits[11];
  uint32_t count = 0u;

  if (value == 0u)
  {
    st_putc('0');
    return;
  }
  while (value > 0u && count < 10u)
  {
    digits[count++] = (char)('0' + (value % 10u));
    value /= 10u;
  }
  while (count > 0u)
  {
    st_putc((uint8_t)digits[--count]);
  }
}

void st_console_hex(uint32_t value)
{
  static const char digits[] = "0123456789ABCDEF";
  st_console_puts("0x");
  for (int32_t shift = 28; shift >= 0; shift -= 4)
  {
    st_putc((uint8_t)digits[(value >> shift) & 0xFu]);
  }
}

void st_console_newline(void)
{
  st_putc((uint8_t)'\n');
}

void st_console_flush(void)
{
  while ((USART1->SR & USART_SR_TC) == 0u)
  {
  }
}

void st_console_init(uint32_t pclk2_hz)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

  st_gpio_af_pp(GPIOA, 9u);          /* USART1_TX */
  st_gpio_input_pull(GPIOA, 10u, 1u); /* USART1_RX, pulled up while nothing drives it */

  USART1->CR1 = 0u;
  s_console_divider = (pclk2_hz + (ST_CONSOLE_BAUD / 2u)) / ST_CONSOLE_BAUD;
  USART1->BRR = s_console_divider;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

/* ------------------------------------------------------------------------- */
/* GPIO helpers                                                              */
/* ------------------------------------------------------------------------- */

static void st_gpio_config(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t cnf)
{
  volatile uint32_t *register_value = (pin < 8u) ? &port->CRL : &port->CRH;
  uint32_t shift = (pin % 8u) * 4u;
  uint32_t bits = (cnf << 2) | mode;

  *register_value = (*register_value & ~(0xFu << shift)) | (bits << shift);
}

void st_gpio_output_pp(GPIO_TypeDef *port, uint32_t pin)
{
  st_gpio_config(port, pin, 0x2u, 0x0u); /* output push-pull, 2 MHz */
}

void st_gpio_af_pp(GPIO_TypeDef *port, uint32_t pin)
{
  st_gpio_config(port, pin, 0x3u, 0x2u); /* alternate function push-pull, 50 MHz */
}

void st_gpio_af_od(GPIO_TypeDef *port, uint32_t pin)
{
  st_gpio_config(port, pin, 0x3u, 0x3u); /* alternate function open-drain, 50 MHz */
}

void st_gpio_input_pull(GPIO_TypeDef *port, uint32_t pin, uint8_t pull_up)
{
  st_gpio_config(port, pin, 0x0u, 0x2u); /* input with pull-up / pull-down */
  if (pull_up != 0u)
  {
    port->BSRR = 1u << pin;
  }
  else
  {
    port->BRR = 1u << pin;
  }
}

/* ------------------------------------------------------------------------- */
/* Clock tree and time base                                                  */
/* ------------------------------------------------------------------------- */

void st_systick_init(uint32_t hclk_hz)
{
  uint32_t reload = (hclk_hz / 1000u) - 1u;

  /* Loading VAL with LOAD starts a full period, so the first interrupt is a
     real millisecond instead of one core cycle. */
  SysTick->LOAD = reload;
  SysTick->VAL = reload;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void st_delay_ms(uint32_t milliseconds)
{
  for (uint32_t index = 0u; index < milliseconds; index++)
  {
    uint32_t start = s_millis;
    while (s_millis == start)
    {
    }
  }
}

void st_clock_init(st_clock_t *clock)
{
  clock->hse_ready = 0u;
  clock->pll_used = 0u;

  /* Run from the internal 8 MHz oscillator first: SysTick and the HSE search
     below need a time base before the PLL exists. */
  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0u)
  {
  }
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_0;
  st_systick_init(8000000u);

  /* The crystal only answers when the board really has one fitted. */
  RCC->CR |= RCC_CR_HSEON;
  for (uint32_t waited = 0u; waited < 5u; waited++)
  {
    if ((RCC->CR & RCC_CR_HSERDY) != 0u)
    {
      break;
    }
    st_delay_ms(1u);
  }

  if ((RCC->CR & RCC_CR_HSERDY) != 0u)
  {
    clock->hse_ready = 1u;
    /* HSE / 1 -> PLL x9 = 72 MHz, APB1 / 2 = 36 MHz, ADC / 6 = 12 MHz. */
    RCC->CFGR = RCC_CFGR_PLLSRC
              | (RCC_CFGR_PLLMULL9)
              | RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_ADCPRE_DIV6;
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u)
    {
    }
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
    }
    clock->pll_used = 1u;
    clock->hclk_hz = ST_HSE_HZ * ST_PLL_MULTIPLIER;
    clock->pclk1_hz = clock->hclk_hz / 2u;
    clock->pclk2_hz = clock->hclk_hz;
  }
  else
  {
    /* Bare chip: stay on the 8 MHz internal oscillator. */
    RCC->CFGR = RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_ADCPRE_DIV2;
    clock->hclk_hz = 8000000u;
    clock->pclk1_hz = clock->hclk_hz;
    clock->pclk2_hz = clock->hclk_hz;
  }

  st_systick_init(clock->hclk_hz);
}

/* ------------------------------------------------------------------------- */
/* Report helpers                                                            */
/* ------------------------------------------------------------------------- */

static uint32_t s_checks;
static uint32_t s_failures;

/* Clock tree of the run, so the tests program real divisors instead of
   hard-coded ones that only fit a 72 MHz PLL. */
static uint32_t s_pclk1_hz;
/* APB1/APB2 timers run at the core clock with the dividers used here. */
static uint32_t s_timer_hz;

static uint32_t st_timer_prescaler(void)
{
  uint32_t divider = s_timer_hz / 1000000u;
  return (divider == 0u) ? 0u : (divider - 1u);
}

static void st_check(const char *name, int ok)
{
  s_checks++;
  if (ok == 0)
  {
    s_failures++;
  }
  st_console_puts(name);
  st_console_puts((ok != 0) ? " OK" : " FAIL");
  st_console_newline();
}

static void st_check_u32(const char *name, int ok, uint32_t value)
{
  s_checks++;
  if (ok == 0)
  {
    s_failures++;
  }
  st_console_puts(name);
  st_console_puts((ok != 0) ? " OK " : " FAIL ");
  st_console_u32(value);
  st_console_newline();
}

static void st_check_hex(const char *name, int ok, uint32_t value)
{
  s_checks++;
  if (ok == 0)
  {
    s_failures++;
  }
  st_console_puts(name);
  st_console_puts((ok != 0) ? " OK " : " FAIL ");
  st_console_hex(value);
  st_console_newline();
}

uint32_t st_failures(void)
{
  return s_failures;
}

/* ------------------------------------------------------------------------- */
/* SysTick                                                                   */
/* ------------------------------------------------------------------------- */

static int st_test_systick(void)
{
  uint32_t start = st_millis();
  st_delay_ms(3u);
  return (st_millis() - start) >= 3u;
}

/* ------------------------------------------------------------------------- */
/* GPIO and bit band                                                         */
/* ------------------------------------------------------------------------- */

#define ST_LED_PORT GPIOB
#define ST_LED_PIN 12u

static uint32_t s_bitband_word;

#define ST_BITBAND_PERIPH(address, bit) \
  (*(volatile uint32_t *)(0x42000000u + (((address) - 0x40000000u) * 32u) + ((bit) * 4u)))
#define ST_BITBAND_SRAM(address, bit) \
  (*(volatile uint32_t *)(0x22000000u + (((address) - 0x20000000u) * 32u) + ((bit) * 4u)))

static int st_test_gpio(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  /* PB12 output (the status LED), PB13 input with a pull-up (lock test). */
  st_gpio_output_pp(ST_LED_PORT, ST_LED_PIN);
  st_gpio_input_pull(GPIOB, 13u, 1u);

  ST_LED_PORT->BSRR = 1u << ST_LED_PIN;
  int high = (ST_LED_PORT->IDR & (1u << ST_LED_PIN)) != 0u;
  /* Hold the level long enough to be seen on the schematic and by the host
     test that samples the LED. */
  st_delay_ms(1u);
  ST_LED_PORT->BRR = 1u << ST_LED_PIN;
  int low = (ST_LED_PORT->IDR & (1u << ST_LED_PIN)) == 0u;
  int pulled_up = (GPIOB->IDR & (1u << 13)) != 0u;

  /* Lock PB13, then make sure the configuration can no longer change. */
  uint32_t before = GPIOB->CRH & 0xFu;
  GPIOB->LCKR = (1u << 13) | GPIO_LCKR_LCKK;
  GPIOB->LCKR = (1u << 13);
  GPIOB->LCKR = (1u << 13) | GPIO_LCKR_LCKK;
  int locked = (GPIOB->LCKR & GPIO_LCKR_LCKK) != 0u;
  st_gpio_output_pp(GPIOB, 13u);
  int unchanged = (GPIOB->CRH & 0xFu) == before;

  return high && low && pulled_up && locked && unchanged;
}

static int st_test_bitband(void)
{
  /* PB12 through the peripheral alias. */
  ST_BITBAND_PERIPH((uint32_t)&ST_LED_PORT->ODR, ST_LED_PIN) = 1u;
  int set_direct = (ST_LED_PORT->ODR & (1u << ST_LED_PIN)) != 0u;
  int set_alias = (ST_BITBAND_PERIPH((uint32_t)&ST_LED_PORT->ODR, ST_LED_PIN) & 1u) != 0u;
  ST_BITBAND_PERIPH((uint32_t)&ST_LED_PORT->ODR, ST_LED_PIN) = 0u;
  int cleared = (ST_LED_PORT->ODR & (1u << ST_LED_PIN)) == 0u;

  /* A single bit of an SRAM word through the SRAM alias. */
  s_bitband_word = 0u;
  ST_BITBAND_SRAM((uint32_t)&s_bitband_word, 17u) = 1u;
  int sram_bit = ((s_bitband_word >> 17) & 1u) != 0u;

  return set_direct && set_alias && cleared && sram_bit;
}

/* ------------------------------------------------------------------------- */
/* EXTI (software request on line 3, the encoder uses lines 0..2)             */
/* ------------------------------------------------------------------------- */

static volatile uint32_t s_exti_line3;

void st_exti_line_irq(void)
{
  EXTI->PR = 1u << 3;
  s_exti_line3++;
}

static int st_test_exti(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

  AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~0x0000F000u) | 0x00001000u; /* EXTI3 <- port B */
  EXTI->RTSR |= 1u << 3;
  EXTI->IMR |= 1u << 3;
  NVIC_SetPriority(EXTI3_IRQn, 0x40u);
  NVIC_EnableIRQ(EXTI3_IRQn);

  uint32_t before = s_exti_line3;
  EXTI->SWIER = 1u << 3;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if (s_exti_line3 != before)
    {
      return (EXTI->PR & (1u << 3)) == 0u;
    }
  }
  return 0;
}

/* ------------------------------------------------------------------------- */
/* USART1 (console, also receives on RX), USART2 and USART3                  */
/* ------------------------------------------------------------------------- */

static int st_test_usart1(void)
{
  /* The transmitter may still be shifting the line that was just printed, so
     only the configuration is checked here; TXE/TC are covered by USART2 and
     USART3 below. */
  return ((USART1->BRR & 0xFFFFu) == st_console_brr())
      && ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) == (USART_CR1_UE | USART_CR1_TE));
}

static int st_test_usart_port(USART_TypeDef *usart, uint32_t gate, uint32_t pclk_hz, uint8_t byte)
{
  RCC->APB1ENR |= gate;
  usart->CR1 = 0u;
  usart->BRR = (pclk_hz + (ST_CONSOLE_BAUD / 2u)) / ST_CONSOLE_BAUD;
  usart->CR1 = USART_CR1_UE | USART_CR1_TE;

  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((usart->SR & USART_SR_TXE) != 0u)
    {
      break;
    }
  }
  usart->DR = (uint32_t)byte;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((usart->SR & USART_SR_TC) != 0u)
    {
      return 1;
    }
  }
  return 0;
}

/* ------------------------------------------------------------------------- */
/* SPI1                                                                      */
/* ------------------------------------------------------------------------- */

static int st_spi_byte(uint8_t value)
{
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((SPI1->SR & SPI_SR_TXE) != 0u)
    {
      break;
    }
  }
  SPI1->DR = (uint32_t)value;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((SPI1->SR & SPI_SR_RXNE) != 0u)
    {
      (void)SPI1->DR;
      return 1;
    }
  }
  return 0;
}

static int st_test_spi(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  st_gpio_af_pp(GPIOA, 5u);          /* SCK  */
  st_gpio_af_pp(GPIOA, 7u);          /* MOSI */
  st_gpio_input_pull(GPIOA, 6u, 1u); /* MISO */
  st_gpio_output_pp(GPIOA, 4u);      /* LOAD / chip select */
  GPIOA->BSRR = 1u << 4;

  /* Master, mode 0, 8 bit, software slave management, fPCLK2 / 8. */
  SPI1->CR1 = 0u;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_SPE;

  int ok = (SPI1->CR1 & SPI_CR1_SPE) != 0u;

  GPIOA->BRR = 1u << 4;
  ok &= st_spi_byte(0x0Fu);
  ok &= st_spi_byte(0x00u); /* MAX7219: leave display test mode */
  GPIOA->BSRR = 1u << 4;

  ok &= (SPI1->SR & SPI_SR_OVR) == 0u;
  ok &= (SPI1->SR & SPI_SR_BSY) == 0u;
  ok &= (SPI1->SR & SPI_SR_RXNE) == 0u;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* I2C1                                                                      */
/* ------------------------------------------------------------------------- */

static int st_test_i2c(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  st_gpio_af_od(GPIOB, 6u); /* SCL */
  st_gpio_af_od(GPIOB, 7u); /* SDA */

  I2C1->CR1 = I2C_CR1_SWRST;
  I2C1->CR1 = 0u;
  uint32_t pclk1_mhz = s_pclk1_hz / 1000000u;
  uint32_t clock_control = s_pclk1_hz / (2u * 100000u); /* 100 kHz standard mode */
  uint32_t rise_time = pclk1_mhz + 1u;

  I2C1->CR2 = pclk1_mhz;
  I2C1->CCR = clock_control;
  I2C1->TRISE = rise_time;
  I2C1->CR1 = I2C_CR1_PE;

  int ok = (I2C1->CR2 & 0x3Fu) == pclk1_mhz;
  ok &= (I2C1->CCR & 0x0FFFu) == clock_control;
  ok &= (I2C1->TRISE & 0x3Fu) == rise_time;

  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_START;
  ok &= (I2C1->SR1 & I2C_SR1_SB) != 0u;
  ok &= (I2C1->SR2 & (I2C_SR2_BUSY | I2C_SR2_MSL)) == (I2C_SR2_BUSY | I2C_SR2_MSL);

  /* Nothing answers on the empty bus, so the address phase is not
     acknowledged: exactly what a real F103 reports. */
  I2C1->DR = 0xA0u;
  ok &= (I2C1->SR1 & I2C_SR1_AF) != 0u;
  ok &= (I2C1->SR1 & I2C_SR1_SB) == 0u;

  I2C1->SR1 = 0u; /* AF is cleared by writing a zero */
  ok &= (I2C1->SR1 & I2C_SR1_AF) == 0u;

  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_STOP;
  ok &= (I2C1->SR2 & I2C_SR2_BUSY) == 0u;
  I2C1->CR1 = I2C_CR1_PE;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* Timers                                                                    */
/* ------------------------------------------------------------------------- */

static int st_test_tim1(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->CR1 = 0u;
  TIM1->PSC = st_timer_prescaler();  /* 1 MHz counter clock */
  TIM1->ARR = 999u;
  TIM1->CCMR1 = (6u << 4);  /* channel 1: PWM mode 1 */
  TIM1->CCER = TIM_CCER_CC1E;
  TIM1->CCR1 = 500u;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = 0u;
  TIM1->CR1 = TIM_CR1_CEN;

  st_delay_ms(2u);

  int ok = (TIM1->SR & (TIM_SR_UIF | TIM_SR_CC1IF)) == (TIM_SR_UIF | TIM_SR_CC1IF);
  ok &= (TIM1->BDTR & TIM_BDTR_MOE) != 0u;
  ok &= TIM1->CCR1 == 500u;
  TIM1->CR1 = 0u;
  return ok;
}

static int st_test_tim2(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->CR1 = 0u;
  TIM2->PSC = st_timer_prescaler();
  TIM2->ARR = 999u;
  TIM2->EGR = TIM_EGR_UG; /* apply PSC/ARR and clear the counter */
  TIM2->SR = 0u;
  TIM2->CR1 = TIM_CR1_CEN;

  st_delay_ms(2u);

  int ok = (TIM2->SR & TIM_SR_UIF) != 0u;
  uint32_t up = TIM2->CNT;
  ok &= (up <= 999u);

  /* Down-counting: the counter walks back from ARR instead of wrapping up. */
  TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
  TIM2->EGR = TIM_EGR_UG;
  st_delay_ms(2u);
  uint32_t down = TIM2->CNT;
  ok &= (down > 0u) && (down <= 999u);

  TIM2->CR1 = 0u;
  return ok;
}

static int st_test_tim3(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  TIM3->CR1 = 0u;
  TIM3->PSC = st_timer_prescaler();
  TIM3->ARR = 999u;
  TIM3->CCMR1 = (6u << 4) | TIM_CCMR1_OC1PE; /* PWM mode 1 with preload */
  TIM3->CCER = TIM_CCER_CC1E;
  TIM3->CCR1 = 250u;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0u;
  TIM3->CR1 = TIM_CR1_CEN;

  st_delay_ms(2u);

  int ok = (TIM3->SR & TIM_SR_CC1IF) != 0u;
  ok &= (TIM3->SR & TIM_SR_UIF) != 0u;
  ok &= (TIM3->CCR1 & 0xFFFFu) == 250u;
  TIM3->CR1 = 0u;
  return ok;
}

static int st_test_tim4(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  TIM4->CR1 = 0u;
  TIM4->PSC = st_timer_prescaler();
  TIM4->ARR = 99u;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR = 0u;
  TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_OPM; /* one pulse: stop after one update */

  st_delay_ms(1u);

  int ok = (TIM4->CR1 & TIM_CR1_CEN) == 0u;
  ok &= (TIM4->SR & TIM_SR_UIF) != 0u;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* ADC1                                                                      */
/* ------------------------------------------------------------------------- */

static void st_adc_configure(uint32_t extra_cr2)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR1 = 0u;
  ADC1->SMPR2 = (7u << 3); /* channel 1: 239.5 sampling cycles */
  ADC1->SQR1 = 0u;         /* one conversion in the sequence */
  ADC1->SQR3 = 1u;         /* first conversion: channel 1 */
  ADC1->CR2 = ADC_CR2_ADON | extra_cr2;
}

static int st_test_adc(uint32_t *code)
{
  st_adc_configure(0u);
  ADC1->SR = 0x1Fu;
  ADC1->CR2 |= ADC_CR2_SWSTART;

  *code = 0u;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((ADC1->SR & ADC_SR_EOC) != 0u)
    {
      *code = ADC1->DR & 0x0FFFu;
      int ok = (ADC1->SR & ADC_SR_EOC) == 0u; /* reading DR clears EOC */
      ADC1->CR2 = 0u;
      return ok;
    }
  }
  ADC1->CR2 = 0u;
  return 0;
}

/* ------------------------------------------------------------------------- */
/* DMA1                                                                      */
/* ------------------------------------------------------------------------- */

static uint32_t s_dma_source[4] = {0x11223344u, 0x55667788u, 0x99AABBCCu, 0xDDEEFF00u};
static uint32_t s_dma_destination[4];
static uint16_t s_dma_samples[4];

static int st_test_dma(uint32_t adc_code)
{
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  DMA1->IFCR = 0x0FFFFFFFu;

  /* 1) Memory to memory, four words on channel 6. */
  DMA1_Channel6->CCR = 0u;
  DMA1_Channel6->CPAR = (uint32_t)s_dma_source;
  DMA1_Channel6->CMAR = (uint32_t)s_dma_destination;
  DMA1_Channel6->CNDTR = 4u;
  DMA1_Channel6->CCR = DMA_CCR_MEM2MEM | DMA_CCR_MINC | DMA_CCR_PINC
                     | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_EN;

  int ok = 0;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if (DMA1_Channel6->CNDTR == 0u)
    {
      ok = 1;
      break;
    }
  }
  DMA1_Channel6->CCR = 0u;

  for (uint32_t index = 0u; index < 4u; index++)
  {
    if (s_dma_destination[index] != s_dma_source[index])
    {
      ok = 0;
      break;
    }
  }
  ok &= (DMA1->ISR & DMA_ISR_TCIF6) != 0u;

  /* 2) Automatic conversions into memory: ADC1 -> DMA1 channel 1, circular. */
  DMA1->IFCR = 0x0FFFFFFFu;
  st_adc_configure(0u);
  DMA1_Channel1->CCR = 0u;
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
  DMA1_Channel1->CMAR = (uint32_t)s_dma_samples;
  DMA1_Channel1->CNDTR = 4u;
  DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_EN;
  ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_SWSTART;

  st_delay_ms(2u);
  ok &= (DMA1->ISR & DMA_ISR_TCIF1) != 0u;
  for (uint32_t index = 0u; index < 4u; index++)
  {
    if ((uint32_t)s_dma_samples[index] != adc_code)
    {
      ok = 0;
      break;
    }
  }

  /* Leave the ADC and the controller idle again. */
  ADC1->CR2 = 0u;
  DMA1_Channel1->CCR = 0u;
  DMA1->IFCR = 0x0FFFFFFFu;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* Flash interface                                                           */
/* ------------------------------------------------------------------------- */

#define ST_FLASH_TEST_PAGE 0x0800FC00u

extern uint32_t _etext;

static int st_test_flash(void)
{
  /* Never erase the page the firmware is running from. */
  if (ST_FLASH_TEST_PAGE < (uint32_t)&_etext)
  {
    return 0;
  }

  volatile uint16_t *word = (volatile uint16_t *)ST_FLASH_TEST_PAGE;

  FLASH->KEYR = 0x45670123u;
  FLASH->KEYR = 0xCDEF89ABu;
  int ok = (FLASH->CR & FLASH_CR_LOCK) == 0u;

  /* Erase the last page of the 64 KiB bank, then program two half words. */
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
  FLASH->CR = FLASH_CR_PER;
  FLASH->AR = ST_FLASH_TEST_PAGE;
  FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;
  ok &= (FLASH->SR & FLASH_SR_EOP) != 0u;
  ok &= word[0] == 0xFFFFu;

  FLASH->SR = FLASH_SR_EOP;
  FLASH->CR = FLASH_CR_PG;
  word[0] = 0x1234u;
  word[1] = 0xABCDu;
  ok &= (FLASH->SR & FLASH_SR_EOP) != 0u;
  ok &= word[0] == 0x1234u;
  ok &= word[1] == 0xABCDu;

  FLASH->CR = FLASH_CR_LOCK;
  ok &= (FLASH->CR & FLASH_CR_LOCK) != 0u;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* CRC                                                                       */
/* ------------------------------------------------------------------------- */

static int st_test_crc(uint32_t *value)
{
  RCC->AHBENR |= RCC_AHBENR_CRCEN;

  /* The checksum can only be preloaded through the 32 bit view of IDR: the
     CMSIS header declares the register as 8 bit wide. Preloading it keeps the
     test repeatable when the core is restarted without a power-on reset. */
  *(volatile uint32_t *)((uint32_t)&CRC->IDR) = 0xFFFFFFFFu;
  CRC->DR = 0u;           /* feed one all-zero word */
  *value = CRC->DR;

  /* Reference result of the F1 calculation unit for that input. */
  return *value == 0xC704DD7B;
}

/* ------------------------------------------------------------------------- */
/* RTC, PWR and BKP                                                          */
/* ------------------------------------------------------------------------- */

#define ST_RTC_SOURCE_LSE 1u
#define ST_RTC_SOURCE_LSI 2u

static volatile uint32_t s_lse_ready;

static int st_test_rtc(uint32_t *source)
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  PWR->CR |= PWR_CR_DBP; /* backup domain writes */

  /* Ask the 32.768 kHz crystal for duty and give it a bounded head start. */
  RCC->BDCR |= RCC_BDCR_LSEON;
  for (uint32_t waited = 0u; waited < 2u; waited++)
  {
    if ((RCC->BDCR & RCC_BDCR_LSERDY) != 0u)
    {
      break;
    }
    st_delay_ms(1u);
  }
  s_lse_ready = (RCC->BDCR & RCC_BDCR_LSERDY) != 0u ? 1u : 0u;

  if (s_lse_ready != 0u)
  {
    *source = ST_RTC_SOURCE_LSE;
    RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_LSE;
  }
  else
  {
    /* No crystal (or still starting): clock the RTC from the internal LSI. */
    *source = ST_RTC_SOURCE_LSI;
    RCC->CSR |= RCC_CSR_LSION;
    for (uint32_t waited = 0u; waited < 5u; waited++)
    {
      if ((RCC->CSR & RCC_CSR_LSIRDY) != 0u)
      {
        break;
      }
      st_delay_ms(1u);
    }
    RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_LSI;
  }
  RCC->BDCR |= RCC_BDCR_RTCEN;

  /* Program the prescaler and the counter through the configuration mode. */
  RTC->CRL |= RTC_CRL_CNF;
  RTC->PRLH = 0u;
  RTC->PRLL = (*source == ST_RTC_SOURCE_LSE) ? 32767u : 39999u;
  RTC->CNTH = 0u;
  RTC->CNTL = 0x1234u;
  RTC->ALRH = 0u;
  RTC->ALRL = 0x2000u;
  RTC->CRL &= ~RTC_CRL_CNF;

  int ok = (RTC->CRL & RTC_CRL_RTOFF) != 0u;
  ok &= ((RTC->CNTH << 16) | RTC->CNTL) == 0x1234u;
  ok &= RTC->ALRL == 0x2000u;

  /* The divider moves at the RTC clock, which proves the counter is running. */
  uint32_t divider = RTC->DIVL;
  st_delay_ms(2u);
  ok &= RTC->DIVL != divider;
  return ok;
}

static int st_test_bkp(void)
{
  /* The backup domain accepts writes once PWR_CR.DBP is set. */
  PWR->CR |= PWR_CR_DBP;
  BKP->DR1 = 0x5A5Au;
  return (BKP->DR1 & 0xFFFFu) == 0x5A5Au;
}

static int st_test_pwr(void)
{
  PWR->CR |= PWR_CR_PVDE | (4u << 5); /* power voltage detector at 2.9 V */
  int ok = (PWR->CR & (PWR_CR_PVDE | PWR_CR_PLS)) == (PWR_CR_PVDE | (4u << 5));

  PWR->CR |= PWR_CR_DBP;
  ok &= (PWR->CR & PWR_CR_DBP) != 0u;

  /* The wake-up and standby flags are cleared by writing the CR bits. */
  PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
  ok &= (PWR->CSR & (PWR_CSR_WUF | PWR_CSR_SBF)) == 0u;
  return ok;
}

/* ------------------------------------------------------------------------- */
/* bxCAN (CAN1) — internal loopback frame                                     */
/* ------------------------------------------------------------------------- */

#define ST_CAN_ID 0x123u
#define ST_CAN_DLC 5u

static volatile uint32_t s_can_irq;

void st_can_rx_irq(void)
{
  s_can_irq++;
}

uint32_t st_can_irq_count(void)
{
  return s_can_irq;
}

static int st_test_can(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  st_gpio_af_pp(GPIOA, 12u);          /* CAN1_TX */
  st_gpio_input_pull(GPIOA, 11u, 1u); /* CAN1_RX */

  /* Enter initialization mode to program the bit timing. */
  CAN1->MCR = CAN_MCR_INRQ;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((CAN1->MSR & CAN_MSR_INAK) != 0u)
    {
      break;
    }
  }

  /* 500 kbit/s from PCLK1: 8 tq per bit, prescaler chosen for the clock tree
     that is actually running. The backend loopback lets the frame complete
     without a transceiver on the bus. */
  uint32_t prescaler = s_pclk1_hz / (500000u * 8u);
  if (prescaler == 0u)
  {
    prescaler = 1u;
  }
  CAN1->BTR = CAN_BTR_LBKM | (prescaler - 1u) | (3u << 16) | (2u << 20);

  /* One 32 bit mask filter for the standard identifier 0x123 into FIFO0. */
  CAN1->FMR |= CAN_FMR_FINIT;
  CAN1->FA1R = 0u;
  CAN1->FM1R = 0u;
  CAN1->FS1R = 1u;
  CAN1->FFA1R = 0u;
  CAN1->sFilterRegister[0].FR1 = ST_CAN_ID << 21;
  CAN1->sFilterRegister[0].FR2 = 0x7FFu << 21;
  CAN1->FA1R = 1u;
  CAN1->FMR &= ~CAN_FMR_FINIT;

  CAN1->IER = CAN_IER_FMPIE0;
  NVIC_SetPriority(CAN1_RX0_IRQn, 0x50u);
  NVIC_EnableIRQ(CAN1_RX0_IRQn);

  CAN1->MCR = 0u;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((CAN1->MSR & CAN_MSR_INAK) == 0u)
    {
      break;
    }
  }

  /* Arm mailbox 0 with the five byte payload "Hello". */
  CAN1->sTxMailBox[0].TDTR = ST_CAN_DLC;
  CAN1->sTxMailBox[0].TDLR = (uint32_t)'H' | ((uint32_t)'e' << 8)
                           | ((uint32_t)'l' << 16) | ((uint32_t)'l' << 24);
  CAN1->sTxMailBox[0].TDHR = (uint32_t)'o';
  CAN1->sTxMailBox[0].TIR = (ST_CAN_ID << 21) | CAN_TI0R_TXRQ;

  int ok = 0;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0u)
    {
      uint32_t identifier = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FFu;
      uint32_t length = CAN1->sFIFOMailBox[0].RDTR & 0xFu;
      uint32_t low = CAN1->sFIFOMailBox[0].RDLR;
      uint32_t high = CAN1->sFIFOMailBox[0].RDHR;
      CAN1->RF0R = CAN_RF0R_RFOM0; /* release the mailbox */

      ok = (identifier == ST_CAN_ID) && (length == ST_CAN_DLC);
      ok &= ((low & 0xFFu) == (uint32_t)'H') && (((low >> 8) & 0xFFu) == (uint32_t)'e');
      ok &= (((low >> 16) & 0xFFu) == (uint32_t)'l') && (((low >> 24) & 0xFFu) == (uint32_t)'l');
      ok &= ((high & 0xFFu) == (uint32_t)'o');
      break;
    }
  }
  ok &= s_can_irq > 0u;

  /* Back to normal mode: the application frames the bus with the transceiver. */
  CAN1->MCR = CAN_MCR_INRQ;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((CAN1->MSR & CAN_MSR_INAK) != 0u)
    {
      break;
    }
  }
  CAN1->BTR &= ~CAN_BTR_LBKM;
  CAN1->MCR = 0u;
  for (uint32_t guard = 0u; guard < 200000u; guard++)
  {
    if ((CAN1->MSR & CAN_MSR_INAK) == 0u)
    {
      break;
    }
  }
  return ok;
}

/* ------------------------------------------------------------------------- */
/* Watchdogs                                                                 */
/* ------------------------------------------------------------------------- */

static int st_test_iwdg(void)
{
  IWDG->KR = 0x5555u; /* unlock PR and RLR */
  IWDG->PR = 6u;      /* 40 kHz / 256 */
  IWDG->RLR = 0x0FFFu;

  int ok = (IWDG->PR & 0x7u) == 6u;
  ok &= (IWDG->RLR & 0xFFFu) == 0x0FFFu;
  ok &= (IWDG->SR & 0x3u) == 0x3u; /* both registers accepted */

  IWDG->KR = 0xCCCCu; /* start: the timeout is about 104 s with PR/RLR above */
  IWDG->KR = 0xAAAAu; /* reload */
  return ok;
}

static void st_wwdg_refresh(void)
{
  WWDG->CR = 0x7Fu; /* EWI disabled, T = 127, WDGA is kept by the hardware */
}

static int st_test_wwdg(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

  /* PCLK1 / 4096, early wake-up interrupt enabled, window 0x60. */
  WWDG->CFR = (0u << 7) | (1u << 9) | 0x60u;
  int ok = ((WWDG->CFR >> 7) & 0x3u) == 0u;
  ok &= (WWDG->CFR & (1u << 9)) != 0u;

  WWDG->CR = 0x7Fu | 0x80u; /* enable with the counter at its maximum */
  ok &= (WWDG->CR & 0x80u) != 0u;

  /* The early wake-up flag is set once the counter has counted down into the
     window, which proves the watchdog is really running. */
  int early_wakeup = 0;
  for (uint32_t waited = 0u; waited < 100u; waited++)
  {
    if ((WWDG->SR & 0x1u) != 0u)
    {
      early_wakeup = 1;
      WWDG->SR = 0x1u; /* write one to clear */
      break;
    }
    st_delay_ms(1u);
  }
  ok &= early_wakeup != 0;

  st_wwdg_refresh();
  return ok;
}

void st_service_watchdogs(void)
{
  st_wwdg_refresh();
  IWDG->KR = 0xAAAAu;
}

/* ------------------------------------------------------------------------- */
/* The report                                                                */
/* ------------------------------------------------------------------------- */

static const char *st_reset_cause(void)
{
  uint32_t csr = RCC->CSR;
  if ((csr & RCC_CSR_PORRSTF) != 0u)
  {
    return "POR";
  }
  if ((csr & RCC_CSR_PINRSTF) != 0u)
  {
    return "PIN";
  }
  if ((csr & RCC_CSR_SFTRSTF) != 0u)
  {
    return "SOFT";
  }
  if ((csr & RCC_CSR_IWDGRSTF) != 0u)
  {
    return "IWDG";
  }
  if ((csr & RCC_CSR_WWDGRSTF) != 0u)
  {
    return "WWDG";
  }
  if ((csr & RCC_CSR_LPWRRSTF) != 0u)
  {
    return "LOWPOWER";
  }
  return "NONE";
}

void st_run_selftest(const st_clock_t *clock)
{
  uint32_t adc_code = 0u;
  uint32_t crc_value = 0u;
  uint32_t rtc_source = ST_RTC_SOURCE_LSI;

  s_pclk1_hz = clock->pclk1_hz;
  /* APB2 is not divided and APB1 is /2 when the PLL runs, so both timer
     clocks equal the core clock in this configuration. */
  s_timer_hz = clock->hclk_hz;

  st_console_puts("GSIM-SELFTEST 1");
  st_console_newline();

  st_console_puts("CLK ");
  st_console_u32(clock->hclk_hz);
  st_console_puts((clock->pll_used != 0u) ? " HSE" : " HSI");
  st_console_newline();

  st_check("SYSTICK", st_test_systick());
  st_check("GPIO", st_test_gpio());
  st_check("BITBAND", st_test_bitband());
  st_check("EXTI", st_test_exti());

  st_check("USART1", st_test_usart1());
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  st_gpio_af_pp(GPIOA, 2u);           /* USART2_TX */
  st_gpio_input_pull(GPIOA, 3u, 1u);  /* USART2_RX */
  st_check("USART2", st_test_usart_port(USART2, RCC_APB1ENR_USART2EN, clock->pclk1_hz, (uint8_t)'2'));
  st_gpio_af_pp(GPIOB, 10u);          /* USART3_TX */
  st_gpio_input_pull(GPIOB, 11u, 1u); /* USART3_RX */
  st_check("USART3", st_test_usart_port(USART3, RCC_APB1ENR_USART3EN, clock->pclk1_hz, (uint8_t)'3'));

  st_check("SPI1", st_test_spi());
  st_check("I2C1", st_test_i2c());
  st_check("TIM1", st_test_tim1());
  st_check("TIM2", st_test_tim2());
  st_check("TIM3", st_test_tim3());
  st_check("TIM4", st_test_tim4());

  int adc_ok = st_test_adc(&adc_code);
  st_check_u32("ADC", adc_ok, adc_code);
  st_check("DMA", st_test_dma(adc_code));
  st_check("FLASH", st_test_flash());

  int crc_ok = st_test_crc(&crc_value);
  st_check_hex("CRC", crc_ok, crc_value);

  int rtc_ok = st_test_rtc(&rtc_source);
  st_check("RTC", rtc_ok);
  st_check("PWR", st_test_pwr());
  st_check("BKP", st_test_bkp());
  st_check("CAN", st_test_can());

  st_check("IWDG", st_test_iwdg());
  st_check("WWDG", st_test_wwdg());

  st_console_puts("ENV HSE=");
  st_console_puts((clock->hse_ready != 0u) ? "READY" : "ABSENT");
  st_console_puts(" LSE=");
  st_console_puts((s_lse_ready != 0u) ? "READY" : "WAIT");
  st_console_puts(" RTC=");
  st_console_puts((rtc_source == ST_RTC_SOURCE_LSE) ? "LSE" : "LSI");
  st_console_puts(" RESET=");
  st_console_puts(st_reset_cause());
  st_console_newline();

  st_console_puts("ONCHIP ");
  st_console_u32(s_checks - s_failures);
  st_console_puts("/");
  st_console_u32(s_checks);
  st_console_newline();

  /* The window watchdog was armed above: refresh it before the long flush so
     the remaining output cannot run into its window. */
  st_service_watchdogs();
  st_console_flush();
}
