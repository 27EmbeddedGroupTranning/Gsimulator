/* Minimal Cortex-M3 startup code: vector table, .data copy, .bss zeroing.
 * Written for -nostdlib, so it must not rely on any libc helper. */

#include <stdint.h>

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

int main(void);

void Reset_Handler(void);
void Default_Handler(void) { for (;;) { } }

void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector"), used))
void (*const vector_table[])(void) = {
    (void (*)(void)) & _estack, /* 0x00 initial stack pointer */
    Reset_Handler,              /* 0x04 reset */
    NMI_Handler,                /* 0x08 */
    HardFault_Handler,          /* 0x0C */
    Default_Handler,            /* 0x10 MemManage */
    Default_Handler,            /* 0x14 BusFault */
    Default_Handler,            /* 0x18 UsageFault */
    0, 0, 0, 0,                 /* 0x1C..0x28 reserved */
    Default_Handler,            /* 0x2C SVCall */
    Default_Handler,            /* 0x30 DebugMonitor */
    0,                          /* 0x34 reserved */
    Default_Handler,            /* 0x38 PendSV */
    SysTick_Handler,            /* 0x3C SysTick */
    /* IRQ 0.. */
    Default_Handler,            /* 0  WWDG */
    Default_Handler,            /* 1  PVD */
    Default_Handler,            /* 2  TAMPER */
    Default_Handler,            /* 3  RTC */
    Default_Handler,            /* 4  FLASH */
    Default_Handler,            /* 5  RCC */
    EXTI0_IRQHandler,           /* 6  EXTI0 */
    EXTI1_IRQHandler,           /* 7  EXTI1 */
    Default_Handler,            /* 8  EXTI2 */
    Default_Handler,            /* 9  EXTI3 */
    Default_Handler,            /* 10 EXTI4 */
    Default_Handler,            /* 11 DMA1 channel 1 */
    Default_Handler,            /* 12 DMA1 channel 2 */
    Default_Handler,            /* 13 DMA1 channel 3 */
    Default_Handler,            /* 14 DMA1 channel 4 */
    Default_Handler,            /* 15 DMA1 channel 5 */
    Default_Handler,            /* 16 DMA1 channel 6 */
    Default_Handler,            /* 17 DMA1 channel 7 */
    ADC1_2_IRQHandler,          /* 18 ADC1_2 */
    Default_Handler,            /* 19 */
    Default_Handler,            /* 20 */
    Default_Handler,            /* 21 */
    Default_Handler,            /* 22 */
    Default_Handler,            /* 23 EXTI9_5 */
    Default_Handler,            /* 24 TIM1 BRK */
    Default_Handler,            /* 25 TIM1 UP */
    Default_Handler,            /* 26 TIM1 TRG COM */
    Default_Handler,            /* 27 TIM1 CC */
    Default_Handler,            /* 28 TIM2 */
    TIM3_IRQHandler,            /* 29 TIM3 */
    Default_Handler,            /* 30 TIM4 */
    Default_Handler,            /* 31 I2C1 EV */
    Default_Handler,            /* 32 I2C1 ER */
    Default_Handler,            /* 33 I2C2 EV */
    Default_Handler,            /* 34 I2C2 ER */
    Default_Handler,            /* 35 SPI1 */
    Default_Handler,            /* 36 SPI2 */
    USART1_IRQHandler,          /* 37 USART1 */
};

void Reset_Handler(void)
{
    uint32_t *source = &_sidata;
    for (uint32_t *destination = &_sdata; destination < &_edata;) {
        *destination++ = *source++;
    }
    for (uint32_t *destination = &_sbss; destination < &_ebss;) {
        *destination++ = 0;
    }

    main();

    for (;;) {
    }
}
