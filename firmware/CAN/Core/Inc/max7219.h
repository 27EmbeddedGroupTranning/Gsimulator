/**
  ******************************************************************************
  * @file    max7219.h
  * @brief   MAX7219 8x8 LED 点阵驱动（SPI1 软件片选）
  ******************************************************************************
  * 接线（与 CubeMX 当前配置一致）：
  *   SCK       -> PA5  (SPI1_SCK)
  *   MOSI/DIN  -> PA7  (SPI1_MOSI)
  *   LOAD/CS   -> PA4  (SPI1_NSS，CubeMX 中 NSS 为软件管理，代码里配成推挽输出)
  *   VCC       -> 5V（模块自带稳压时可用 3.3V，视模块而定）
  *   GND       -> GND
  ******************************************************************************
  */
#ifndef __MAX7219_H
#define __MAX7219_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---- 片选引脚 ---- */
#define MAX7219_CS_PORT        GPIOA
#define MAX7219_CS_PIN         GPIO_PIN_4

/* 级联片数：单个 8x8 点阵填 1，4 合 1 模块填 4 */
#define MAX7219_CHAIN          1U

/* 亮度 0~15 */
#define MAX7219_INTENSITY      0x07U

/* ---- 行列方向修正 ----
   不同模块的 DIGIT0~DIGIT7 / bit7 物理位置并不统一，按实际显示结果调整：
     MAX7219_FLIP_VERTICAL   = 1 -> DIGIT0 是最下行（图案上下颠倒时置 1）
     MAX7219_FLIP_HORIZONTAL = 1 -> bit7 是最右列（图案左右镜像时置 1） */
#define MAX7219_FLIP_VERTICAL     0
#define MAX7219_FLIP_HORIZONTAL   0

/* ---- 寄存器地址 ---- */
#define MAX7219_REG_NOOP       0x00U
#define MAX7219_REG_DIGIT0     0x01U   /* 第 1 行，DIGIT0~DIGIT7 对应 8 行 */
#define MAX7219_REG_DECODE     0x09U
#define MAX7219_REG_INTENSITY  0x0AU
#define MAX7219_REG_SCANLIMIT  0x0BU
#define MAX7219_REG_SHUTDOWN   0x0CU
#define MAX7219_REG_DISPLAYTEST 0x0FU

void MAX7219_Init(SPI_HandleTypeDef *hspi);
void MAX7219_Reinit(void);
void MAX7219_SetIntensity(uint8_t level);
void MAX7219_DisplayTest(uint8_t enable);
void MAX7219_Clear(void);
void MAX7219_ShowImage(const uint8_t image[8]);
void MAX7219_ShowCheck(void);

/* SPI 传输失败次数，调试器里观察；正常应保持 0 */
extern volatile uint32_t MAX7219_ErrCount;
/* DOUT 回读值（接 DOUT→PA6 后有效），正常应为最近发出的一帧 */
extern volatile uint16_t MAX7219_Echo;

void MAX7219_EchoTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAX7219_H */
