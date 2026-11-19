#include "canfd_protocol.h"
#include "motor_can.h"
#include "fdcan.h"

void FDCAN_Config(void) {

    FDCAN_FilterTypeDef sFilterConfig;

    /* Configure Rx filter : accept only frames addressed to this motor */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_EXACT;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = motor_add;
    sFilterConfig.FilterID2    = 0x00;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                      FDCAN_REJECT,               // Non-matching Standard : reject
                                      FDCAN_REJECT,               // Non-matching Extended : reject
                                      FDCAN_FILTER_REMOTE_REJECT,
                                      FDCAN_FILTER_REMOTE_REJECT) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable interrupt on new message in RX FIFO0 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Prepare Tx Header (used for acknowledgment frames) */
    TxHeader.Identifier          = motor_add;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_FRAME_CLASSIC;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;
}
