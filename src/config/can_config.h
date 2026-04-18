#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

#include "IO_CAN.h"
#include "util/units.h"

/* Channels */
#define CONTROLS_CAN_CHANNEL    IO_CAN_CHANNEL_0
#define DAQ_CAN_CHANNEL         IO_CAN_CHANNEL_1

/* Baud Rate */
#define BAUD_RATE       500

// Buffer size for every rx MESSAGE, will pretty much always be at 0 or 1
// since we poll (200hz) faster than we will receive (50hz)
#define RX_FIFO_BUFFER_SIZE 8

// Buffer size for an entire tx CHANNEL, higher since we will be sending
// out messages in bursts every cycle
#define TX_FIFO_BUFFER_SIZE 32

// TODO watch out for slower messages need to make multiple
#define MSG_TIMEOUT_US      MsToUs(1000)

/* TX Periods */
#define CAN_TX_RATE_5MS         MsToUs(5)
#define CAN_TX_RATE_10MS        MsToUs(10)
#define CAN_TX_RATE_100MS       MsToUs(100)
#define CAN_TX_RATE_1000MS      MsToUs(1000)

/* RX Message IDs */
#define CAN_ID_INV_STATUS       0x0AB // TODO do we need this

#define CAN_ID_INV_HIGH_SPEED   0x0B0

/* TX Message IDs */
#define CAN_ID_INV_TORQUE_COMMAND   0x0C0
#define CAN_ID_INV_READ_WRITE       0x0C1
#define CAN_ID_APPS_VOLTAGES        0x0D1ACCE0
#define CAN_ID_APPS_VALUES          0x0D1ACCE1
#define CAN_ID_VCU_SUMMARY          0x0D1001F0
#define CAN_ID_BSE                  0x0D100B5E
#define CAN_ID_VCU_SETTINGS         0x0D1000CF
#define CAN_ID_VCU_CONFIG_SET       0x0D1000CE
#define CAN_ID_EXIT_DRIVE_REASON    0x0D10DEAD


#endif
