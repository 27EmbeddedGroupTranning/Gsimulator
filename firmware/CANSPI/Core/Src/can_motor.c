/**
  ******************************************************************************
  * @file    can_motor.c
  * @brief   Command frames for the CAN motor on the virtual bench.
  ******************************************************************************
  * CAN1 runs at 200 kbit/s (36 MHz PCLK1 / 36 / (1 + 2 + 2)); the bridge clocks
  * the wire from the same BTR, so the part sees exactly these timings. The
  * driver only transmits - the motor answers with the ACK slot of the frame it
  * decoded - so no acceptance filter is installed.
  ******************************************************************************
  */
#include "can_motor.h"
#include "can.h"

volatile uint32_t CAN_Motor_Frames = 0U;
volatile uint32_t CAN_Motor_Errors = 0U;

void CAN_Motor_Init(void)
{
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t CAN_Motor_SetSpeedRpm(int16_t rpm)
{
  CAN_TxHeaderTypeDef header = {0};
  /* The HAL always fills all eight data bytes, so the payload buffer has to be
     a full frame even though only the first two bytes carry the command. */
  uint8_t data[8] = {0};
  uint32_t mailbox = 0U;

  if (rpm > CAN_MOTOR_MAX_RPM)
  {
    rpm = CAN_MOTOR_MAX_RPM;
  }
  else if (rpm < -CAN_MOTOR_MAX_RPM)
  {
    rpm = -CAN_MOTOR_MAX_RPM;
  }

  data[0] = (uint8_t)((uint16_t)rpm >> 8);
  data[1] = (uint8_t)((uint16_t)rpm & 0xFFU);

  header.StdId = CAN_MOTOR_COMMAND_ID;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 2U;
  header.TransmitGlobalTime = DISABLE;

  /* With automatic retransmission disabled a mailbox is free again as soon as
     the frame left the wire (or was lost), so the next phase can ask again. */
  if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) != HAL_OK)
  {
    CAN_Motor_Errors++;
    return 0U;
  }

  CAN_Motor_Frames++;
  return 1U;
}

uint8_t CAN_Motor_Stop(void)
{
  return CAN_Motor_SetSpeedRpm(0);
}
