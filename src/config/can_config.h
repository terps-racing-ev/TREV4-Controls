#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

#include "IO_CAN.h"

/* Channels */
#define CONTROLS_CAN_CHANNEL IO_CAN_CHANNEL_0
#define TELEMETRY_CAN_CHANNEL IO_CAN_CHANNEL_1

/* Baud Rate */
#define BAUD_RATE       500

// Buffer size for every rx message, will pretty much always be at 0 or 1
// since we poll (200hz) faster than we will receive (50hz)
#define RX_FIFO_BUFFER_SIZE 8

// Buffer size for an entire tx channel
#define TX_FIFO_BUFFER_SIZE 32


/* RX Message IDs */
#define CAN_ID_INV_STATUS       0x000000AB

/* TX Message IDs */
#define CAN_ID_TORQUE_COMMAND   0x000000C0
#define CAN_ID_APPS0            0x001ACCE0
#define CAN_ID_APPS1            0x001ACCE1


#endif