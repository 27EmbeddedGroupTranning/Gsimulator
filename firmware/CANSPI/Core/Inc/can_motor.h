/**
  ******************************************************************************
  * @file    can_motor.h
  * @brief   CAN command frames for the gear motor of the virtual bench.
  ******************************************************************************
  * The part (gsim-devices::Motor) is a node of its own on the pair: it decodes
  * exactly one standard data frame and follows it.
  *
  *   identifier  0x200 (11 bit, data frame)
  *   data[0]     target speed, high byte
  *   data[1]     target speed, low byte
  *
  * The two bytes are a big endian signed 16 bit number in rpm: positive turns
  * the shaft one way, negative the other, zero lets it coast to a stop.
  ******************************************************************************
  */
#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Standard identifier the motor listens to. */
#define CAN_MOTOR_COMMAND_ID  0x200U

/* Highest speed the part follows, in rpm. */
#define CAN_MOTOR_MAX_RPM     3000

/* Leaves initialisation mode: call once, after MX_CAN_Init(). */
void CAN_Motor_Init(void);

/* Queues one command frame; returns 1 when it entered a mailbox. */
uint8_t CAN_Motor_SetSpeedRpm(int16_t rpm);
uint8_t CAN_Motor_Stop(void);

/* Frames accepted by a mailbox / frames refused (all mailboxes busy). */
extern volatile uint32_t CAN_Motor_Frames;
extern volatile uint32_t CAN_Motor_Errors;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_MOTOR_H */
