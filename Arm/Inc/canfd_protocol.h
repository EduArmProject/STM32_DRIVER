#ifndef CANFD_PROTOCOL_H
#define CANFD_PROTOCOL_H

/* Configure the FDCAN filter, start the peripheral and prepare the Tx header.
 * Must be called after Fetch_Motor_Address() so that motor_add is valid. */
void FDCAN_Config(void);

#endif /* CANFD_PROTOCOL_H */
