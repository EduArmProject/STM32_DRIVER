/**
  ******************************************************************************
  * @file    canfd_protocol.h
  * @author  Corentin_k_
  * @brief   Header file of FDCAN protocol module.
  *          This file contains function prototypes and definitions for the
  *          Flexible DataRate Controller Area Network (FDCAN) peripheral:
  *           + Initialization and filter configuration functions
  *           + Message transmission and reception functions (CAN-FD)
  *           + Queue and FIFO management functions
  ******************************************************************************
  */

#ifndef FDCAN_PROTOCOL_H
#define FDCAN_PROTOCOL_H

#include <stdint.h>
#include "main.h"

// Reference documents:
// https://community.st.com/stm32-mcus-60/how-to-use-fdcan-to-create-a-simple-communication-with-a-basic-filter-132821
// https://www.st.com/resource/en/application_note/an5348-introduction-to-fdcan-peripherals-for-stm32-mcus-stmicroelectronics.pdf

/** @defgroup CAN_Standard_Identifiers CAN Standard Identifiers Table
  * @{
  */
#define CAN_ID_ESTOP_BROADCAST    0x000U /*!< Emergency stop broadcast (highest priority ID) */
#define CAN_ID_SYNC_BROADCAST     0x001U /*!< Synchronization pulse broadcast */
#define CAN_ID_CMD_OFFSET         0x100U /*!< Base offset for incoming motor commands (0x100 + motor_add) */
#define CAN_ID_TELEMETRY_OFFSET   0x200U /*!< Base offset for outgoing telemetry frames (0x200 + motor_add) */
#define CAN_ID_FAULT_OFFSET       0x300U /*!< Base offset for fault / error frames (0x300 + motor_add) */
/**
  * @}
  */

/* External flags accessible by application / FSM */
extern volatile uint8_t can_estop_triggered;
extern volatile uint8_t can_sync_triggered;

/**
  * @brief  Configure FDCAN reception filters, start the module, and activate notifications.
  * @note   Must be called after Fetch_Motor_Address() so that motor_add is properly set.
  * @retval None
  */
void FDCAN_Config(void);

/**
  * @brief  Retrieve and process all pending messages from the specified Rx FIFO.
  * @param  RxFifo Rx FIFO location (FDCAN_RX_FIFO0 or FDCAN_RX_FIFO1)
  * @retval None
  */
void FDCAN_Receive_Msg(uint32_t RxFifo);

/**
  * @brief  Send a CAN-FD message (up to 64 bytes at 5 Mbit/s) via the Tx Queue.
  * @note   Messages with the lowest ID are transmitted first by the hardware scheduler.
  * @param  std_id Standard 11-bit identifier (e.g., CAN_ID_TELEMETRY_OFFSET + motor_add)
  * @param  pData  Pointer to data buffer to transmit
  * @param  len    Length of payload in bytes (0 to 64)
  * @retval HAL status (HAL_OK, HAL_BUSY, or HAL_ERROR)
  */
HAL_StatusTypeDef FDCAN_Send_Msg(uint32_t std_id, const uint8_t *pData, uint8_t len);

/**
  * @brief  Emergency stop callback invoked immediately upon receiving CAN_ID_ESTOP_BROADCAST.
  * @note   Weak definition provided; can be overridden in user application.
  * @retval None
  */
void Motor_stop(void);

/**
  * @brief  Message execution callback invoked when an operational CAN message is received.
  * @note   Weak definition provided; can be overridden in user application.
  * @retval None
  */
void execute_msg(void);

#endif /* FDCAN_PROTOCOL_H */
