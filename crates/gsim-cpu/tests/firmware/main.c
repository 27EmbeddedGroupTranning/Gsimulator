/* Phase 1 smoke firmware for Gsimulator.
 *
 * It exercises the peripherals that the simulator models in this phase, using
 * nothing but register accesses: USART1 (polled transmit), SysTick (interrupt),
 * EXTI0 triggered from software (interrupt), TIM3 (PWM setup), ADC1 (polled
 * single conversion) and both Cortex-M3 bit-band alias regions. The output is a
 * short ASCII report so that the host test can assert on it byte for byte.
 */

#include <stdint.h>

#define REG(address) (*(volatile uint32_t *)(address))

/* RCC */
#define RCC_APB2ENR REG(0x40021018U)
#define RCC_APB1ENR REG(0x4002101CU)

/* GPIO */
#define GPIOA_CRL REG(0x40010800U)
#define GPIOA_CRH REG(0x40010804U)

/* AFIO / EXTI */
#define AFIO_EXTICR1 REG(0x40010008U)
#define EXTI_IMR REG(0x40010400U)
#define EXTI_RTSR REG(0x40010408U)
#define EXTI_SWIER REG(0x40010410U)
#define EXTI_PR REG(0x40010414U)

/* USART1 */
#define USART1_SR REG(0x40013800U)
#define USART1_DR REG(0x40013804U)
#define USART1_BRR REG(0x40013808U)
#define USART1_CR1 REG(0x4001380CU)

/* TIM3 */
#define TIM3_CR1 REG(0x40000400U)
#define TIM3_CCMR1 REG(0x40000418U)
#define TIM3_CCER REG(0x40000420U)
#define TIM3_PSC REG(0x40000428U)
#define TIM3_ARR REG(0x4000042CU)
#define TIM3_CCR1 REG(0x40000434U)

/* ADC1 */
#define ADC1_SR REG(0x40012400U)
#define ADC1_CR1 REG(0x40012404U)
#define ADC1_CR2 REG(0x40012408U)
#define ADC1_SMPR2 REG(0x40012410U)
#define ADC1_SQR1 REG(0x4001242CU)
#define ADC1_SQR3 REG(0x40012434U)
#define ADC1_DR REG(0x4001244CU)

/* Cortex-M system control */
#define SYST_CSR REG(0xE000E010U)
#define SYST_RVR REG(0xE000E014U)
#define NVIC_ISER0 REG(0xE000E100U)
#define NVIC_IPR1 REG(0xE000E404U)

/* Cortex-M3 bit-band aliases, the way stm32f1xx.h defines them. */
#define PERIPH_BB(byte, bit) \
    REG(0x42000000U + (((byte) - 0x40000000U) * 32U) + ((bit) * 4U))
#define SRAM_BB(byte, bit) \
    REG(0x22000000U + (((byte) - 0x20000000U) * 32U) + ((bit) * 4U))

#define GPIOA_ODR REG(0x4001080CU)

#define USART_SR_TXE (1U << 7)
#define USART_CR1_TE (1U << 3)
#define USART_CR1_UE (1U << 13)

#define EXTI0_IRQN 6U

volatile uint32_t g_ticks;
volatile uint32_t g_exti_count;
volatile uint32_t g_bb_word;

void SysTick_Handler(void)
{
    g_ticks++;
}

void EXTI0_IRQHandler(void)
{
    EXTI_PR = 1U; /* acknowledge the line */
    g_exti_count++;
}

static void uart_putc(char character)
{
    while ((USART1_SR & USART_SR_TXE) == 0U) {
    }
    USART1_DR = (uint32_t)(unsigned char)character;
}

static void uart_puts(const char *text)
{
    while (*text != '\0') {
        uart_putc(*text++);
    }
}

static void uart_put_u32(uint32_t value)
{
    char digits[11];
    int count = 0;

    if (value == 0U) {
        uart_putc('0');
        return;
    }
    while (value > 0U && count < 10) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0) {
        uart_putc(digits[--count]);
    }
}

static void uart_init(void)
{
    /* USART1, GPIOA and AFIO clocks. */
    RCC_APB2ENR = RCC_APB2ENR | (1U << 14) | (1U << 2) | (1U << 0);

    /* PA9 alternate function push-pull, 50 MHz. */
    {
        uint32_t crh = GPIOA_CRH;
        crh &= ~(0xFU << 4);
        crh |= (0xBU << 4);
        GPIOA_CRH = crh;
    }

    /* 8 MHz HSI / 115200 baud. */
    USART1_BRR = 69U;
    USART1_CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void systick_init(void)
{
    SYST_RVR = 799U;   /* 100 us at 8 MHz */
    SYST_CSR = 0x7U;   /* core clock, interrupt enabled, counting */
}

static void exti_init(void)
{
    /* PA0: input with pull-down. */
    {
        uint32_t crl = GPIOA_CRL;
        crl &= ~0xFU;
        crl |= 0x8U;
        GPIOA_CRL = crl;
    }
    AFIO_EXTICR1 = 0U;      /* EXTI0 -> port A */
    EXTI_RTSR = EXTI_RTSR | 1U;
    EXTI_IMR = EXTI_IMR | 1U;

    NVIC_IPR1 = (NVIC_IPR1 & ~(0xFFU << 24)) | (0x40U << 24);
    NVIC_ISER0 = NVIC_ISER0 | (1U << EXTI0_IRQN);
}

static void tim3_init(void)
{
    RCC_APB2ENR = RCC_APB2ENR | (1U << 2); /* GPIOA */
    RCC_APB1ENR = RCC_APB1ENR | (1U << 1); /* TIM3 */

    /* PA6 as a 50 MHz alternate-function push-pull output: that is where
     * TIM3_CH1 drives the PWM waveform. */
    {
        uint32_t crl = GPIOA_CRL;
        crl &= ~(0xFU << 24);
        crl |= (0xBU << 24);
        GPIOA_CRL = crl;
    }

    TIM3_PSC = 7U;      /* 8 MHz / 8 = 1 MHz counter clock */
    TIM3_ARR = 999U;    /* 1 kHz period */
    TIM3_CCMR1 = 0x60U; /* PWM mode 1 with preload */
    TIM3_CCER = 0x1U;   /* channel 1 output enabled */
    TIM3_CCR1 = 250U;   /* 25 % duty cycle */
    TIM3_CR1 = 0x1U;    /* enable the counter */
}

static uint32_t adc_read_channel1(void)
{
    RCC_APB2ENR = RCC_APB2ENR | (1U << 9); /* ADC1 */

    ADC1_CR1 = 0U;
    ADC1_SMPR2 = 7U;    /* channel 1: 239.5 sampling cycles */
    ADC1_SQR1 = 0U;     /* sequence length 1 */
    ADC1_SQR3 = 1U;     /* first conversion: channel 1 */
    ADC1_CR2 = (1U << 0) | (1U << 22); /* ADON | SWSTART */

    while ((ADC1_SR & 0x2U) == 0U) {
    }
    return ADC1_DR & 0x0FFFU;
}

static void wait_ticks(uint32_t count)
{
    while (g_ticks < count) {
    }
}

/* Exercises both bit-band alias regions: setting and clearing a GPIO output
 * bit through the peripheral alias, and a single bit of an SRAM word through
 * the SRAM alias. Returns 0b1101 when everything behaved. */
static uint32_t bitband_probe(void)
{
    uint32_t set_direct;
    uint32_t set_alias;
    uint32_t cleared_alias;
    uint32_t sram_bit;

    PERIPH_BB(0x4001080CU, 5) = 1U; /* GPIOA->ODR bit 5 */
    set_direct = (GPIOA_ODR >> 5) & 1U;
    set_alias = PERIPH_BB(0x4001080CU, 5) & 1U;
    PERIPH_BB(0x4001080CU, 5) = 0U;
    cleared_alias = PERIPH_BB(0x4001080CU, 5) & 1U;

    g_bb_word = 0U;
    SRAM_BB((uint32_t)&g_bb_word, 17U) = 1U;
    sram_bit = (g_bb_word >> 17) & 1U;

    return (set_direct << 3) | (set_alias << 2) | (cleared_alias << 1) | sram_bit;
}

int main(void)
{
    uint32_t adc;

    uart_init();
    uart_puts("GSIM\n");

    systick_init();
    exti_init();

    wait_ticks(3U);
    uart_puts("T");
    uart_put_u32(g_ticks >= 3U ? 3U : 0U);
    uart_puts("\n");

    EXTI_SWIER = 1U; /* software request on EXTI0 */
    while (g_exti_count == 0U) {
    }
    uart_puts("E");
    uart_put_u32(g_exti_count);
    uart_puts("\n");

    tim3_init();
    uart_puts("M");
    uart_put_u32(TIM3_CCR1);
    uart_puts("\n");

    adc = adc_read_channel1();
    uart_puts("A");
    uart_put_u32(adc);
    uart_puts("\n");

    uart_puts("B");
    uart_put_u32(bitband_probe());
    uart_puts("\nOK\n");

    for (;;) {
    }
}
