/**
  ******************************************************************************
  * @file    canfd_protocol.c
  * @author  Corentin_k_
  * @brief   FDCAN protocol module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Flexible DataRate Controller Area Network
  *          (FDCAN) peripheral for EduArm motor node:
  *           + Initialization and filter configuration functions
  *           + Message transmission and reception functions (CAN-FD)
  *           + Queue and FIFO management functions
  ******************************************************************************
  */

#include "canfd_protocol.h"
#include "motor_can.h"
#include <stdio.h>
#include <string.h>
#include "fdcan.h"

/** @defgroup FDCAN_Private_Variables FDCAN Private Variables
  * @{
  */
static FDCAN_TxHeaderTypeDef TxHeader;
static FDCAN_RxHeaderTypeDef RxHeaderFifo0;
static FDCAN_RxHeaderTypeDef RxHeaderFifo1;
static uint8_t RxDataFifo0[64];             /*!< FIFO for normal message reception (motor control, etc.) */
static uint8_t RxDataFifo1[64];             /*!< FIFO for high-priority message reception (E-Stop, Sync) */

volatile uint8_t can_estop_triggered = 0U; /*!< Emergency stop flag */
volatile uint8_t can_sync_triggered  = 0U; /*!< Synchronization trigger flag */
static uint8_t   tx_msg_marker       = 0U; /*!< Message marker counter for Tx events */
/**
  * @}
  */

/**
  * @brief  Convert a raw payload length in bytes into an FDCAN DLC constant.
  * @param  len Data length in bytes (0 to 64)
  * @retval FDCAN Data Length Code (FDCAN_DLC_BYTES_x)
  */
static uint32_t FDCAN_BytesToDLC(uint8_t len)
{
    if (len == 0U)  return FDCAN_DLC_BYTES_0;
    if (len == 1U)  return FDCAN_DLC_BYTES_1;
    if (len == 2U)  return FDCAN_DLC_BYTES_2;
    if (len == 3U)  return FDCAN_DLC_BYTES_3;
    if (len == 4U)  return FDCAN_DLC_BYTES_4;
    if (len == 5U)  return FDCAN_DLC_BYTES_5;
    if (len == 6U)  return FDCAN_DLC_BYTES_6;
    if (len == 7U)  return FDCAN_DLC_BYTES_7;
    if (len == 8U)  return FDCAN_DLC_BYTES_8;
    if (len <= 12U) return FDCAN_DLC_BYTES_12;
    if (len <= 16U) return FDCAN_DLC_BYTES_16;
    if (len <= 20U) return FDCAN_DLC_BYTES_20;
    if (len <= 24U) return FDCAN_DLC_BYTES_24;
    if (len <= 32U) return FDCAN_DLC_BYTES_32;
    if (len <= 48U) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

/**
  * @brief  Convert an FDCAN Data Length Code (DLC) back to byte length.
  * @param  dlc FDCAN Data Length Code from Rx Header
  * @retval Payload size in bytes (0 to 64)
  */
static uint8_t FDCAN_DLCToBytes(uint32_t dlc)
{
    static const uint8_t dlc_table[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    uint8_t index = (uint8_t)(dlc >> 16);
    return (index < 16U) ? dlc_table[index] : 64U;
}

/**
  * @brief  Configure FDCAN1 hardware filters, start peripheral and prepare Tx header.
  * @retval None
  */
void FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    /* Filter 0: High-Priority Dual Filter for E-Stop & Sync broadcasts -> Route to FIFO1 */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0U;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
    sFilterConfig.FilterID1    = CAN_ID_ESTOP_BROADCAST;
    sFilterConfig.FilterID2    = CAN_ID_SYNC_BROADCAST;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Filter 1: Dedicated Motor Command Filter -> Route to FIFO0 */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 1U;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = CAN_ID_CMD_OFFSET + motor_add;
    sFilterConfig.FilterID2    = 0x7FFU; /* Exact 11-bit match */

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure global filter: reject non-matching standard & extended frames */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Overwrite oldest messages when FIFO0 is full */
    if (HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Internal timestamp counter configuration */
    if (HAL_FDCAN_EnableTimestampCounter(&hfdcan1, FDCAN_TIMESTAMP_INTERNAL) != HAL_OK)
    {
        Error_Handler();
    }

    /* Transmitter Delay Compensation (TDC) for high-speed bit rate switching */
    uint32_t tdc_offset = (1U + hfdcan1.Init.DataTimeSeg1) * hfdcan1.Init.DataPrescaler;
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, tdc_offset, 0U) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Activate interrupts for FIFO0, FIFO1, and bus-off error states */
    uint32_t active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                          FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                          FDCAN_IT_BUS_OFF              |
                          FDCAN_IT_ERROR_PASSIVE;

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, active_its, 0U) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module (node becomes active on the bus) */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Prepare default Tx Header for telemetry frames */
    TxHeader.Identifier          = CAN_ID_TELEMETRY_OFFSET + motor_add;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    TxHeader.FDFormat            = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0U;
}

/**
  * @brief  Send a CAN-FD message (up to 64 bytes at 5 Mbit/s) via the Tx Queue.
  * @note   Lowest CAN ID has highest priority and is transmitted first.
  * @param  std_id Standard 11-bit identifier (e.g., CAN_ID_TELEMETRY_OFFSET + motor_add)
  * @param  pData  Pointer to payload data buffer
  * @param  len    Length of payload in bytes (0 to 64)
  * @retval HAL status (HAL_OK, HAL_BUSY, or HAL_ERROR)
  */
HAL_StatusTypeDef FDCAN_Send_Msg(uint32_t std_id, const uint8_t *pData, uint8_t len)
{
    uint8_t buf_tx[64] = {0};

    if (len > 64U)
    {
        return HAL_ERROR;
    }

    /* Check if Tx Queue has free slots */
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        return HAL_BUSY; /* All 3 Tx Queue slots are full */
    }

    if ((pData != NULL) && (len > 0U))
    {
        memcpy(buf_tx, pData, len);
    }

    /* Ensure transmission parameters are strictly set */
    TxHeader.Identifier          = std_id & 0x7FFU;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_BytesToDLC(len);
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    TxHeader.FDFormat            = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = tx_msg_marker++;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buf_tx);
}

/**
  * @brief  Read and process all pending messages from the specified Rx FIFO.
  * @param  RxFifo Rx FIFO to drain (FDCAN_RX_FIFO0 or FDCAN_RX_FIFO1)
  * @retval None
  */
void FDCAN_Receive_Msg(uint32_t RxFifo)
{
    FDCAN_RxHeaderTypeDef *pHeader = (RxFifo == FDCAN_RX_FIFO0) ? &RxHeaderFifo0 : &RxHeaderFifo1;
    uint8_t *pData                 = (RxFifo == FDCAN_RX_FIFO0) ? RxDataFifo0    : RxDataFifo1;

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, RxFifo) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, RxFifo, pHeader, pData) == HAL_OK)
        {
            uint8_t payload_len = FDCAN_DLCToBytes(pHeader->DataLength);
            (void)payload_len;

            if (RxFifo == FDCAN_RX_FIFO1)
            {
                /* Priority broadcast messages */
                if (pHeader->Identifier == CAN_ID_ESTOP_BROADCAST)
                {
                    can_estop_triggered = 1U;
                    Motor_stop();
                }
                else if (pHeader->Identifier == CAN_ID_SYNC_BROADCAST)
                {
                    can_sync_triggered = 1U;
                }
            }
            else
            {
                /* Operational motor command message */
                execute_msg();
            }
        }
    }
}

/**
  * @brief  Rx FIFO 0 new message callback (Normal motor commands).
  * @param  hfdcan      Pointer to FDCAN handle
  * @param  RxFifo0ITs  Interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
    {
        FDCAN_Receive_Msg(FDCAN_RX_FIFO0);
    }
}

/**
  * @brief  Rx FIFO 1 new message callback (High-Priority E-Stop & Sync).
  * @param  hfdcan      Pointer to FDCAN handle
  * @param  RxFifo1ITs  Interrupt flags
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U)
    {
        FDCAN_Receive_Msg(FDCAN_RX_FIFO1);
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

/**
  * @brief  FDCAN Error and Status callback (Bus-Off recovery).
  * @param  hfdcan          Pointer to FDCAN handle
  * @param  ErrorStatusITs  Error interrupt flags
  * @retval None
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        can_estop_triggered = 1U;
        Motor_stop();

        HAL_FDCAN_Stop(hfdcan);
        if (HAL_FDCAN_Init(hfdcan) == HAL_OK)
        {
            FDCAN_Config();
        }
        else
        {
            Error_Handler();
        }
    }
}


/**
  * @brief  Default weak implementation of Motor_stop.
  * @note   Can be overridden in user application or motor FSM.
  * @retval None
  */
__weak void Motor_stop(void)
{
    /* To be implemented in motor controller / FSM */
}

/**
  * @brief  Default weak implementation of execute_msg.
  * @note   Decodes 8-byte command into command_received and toggles blue LED.
  * @retval None
  */
__weak void execute_msg(void)
{
    if (RxHeaderFifo0.DataLength == FDCAN_DLC_BYTES_8)
    {
        memcpy(&command_received, RxDataFifo0, sizeof(MotorCommand_t));
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

