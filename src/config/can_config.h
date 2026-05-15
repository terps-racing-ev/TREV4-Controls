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

#define MSG_TIMEOUT_US      MsToUs(5000)

/* TX Periods */
#define CAN_TX_RATE_NON_PERIODIC (0u)
#define CAN_TX_RATE_10MS         MsToCycles(10)
#define CAN_TX_RATE_100MS        MsToCycles(100)
#define CAN_TX_RATE_1000MS       MsToCycles(1000)

/* CAN channel recovery policy */
#define CAN_RECOVERY_BUS_OFF_TRIGGER_CYCLES      MsToCycles(20)   // Time CAN must stay bus-off before recovery starts
#define CAN_RECOVERY_NON_OK_TRIGGER_CYCLES       MsToCycles(100)  // Time CAN must remain non-OK before recovery starts
#define CAN_RECOVERY_COOLDOWN_CYCLES             MsToCycles(500)  // Wait between recovery attempts

/* RX Message IDs */
#define CAN_ID_INV_STATUS           0x0AB // TODO do we need this
#define CAN_ID_INV_HIGH_SPEED       0x0B0
#define CAN_ID_HVC_SUMMARY          0x004001F0
//                                      +---- 0 because the user sends it
//                                      |
//                                      V
#define CAN_ID_SET_VCU_CONFIG       0x000000CF

/* TX Message IDs */
#define CAN_ID_INV_TORQUE_COMMAND   0x0C0
#define CAN_ID_INV_READ_WRITE       0x0C1
#define CAN_ID_CONFIG               0x001000CF
#define CAN_ID_APPS_VOLTAGES        0x0D1ACCE0
#define CAN_ID_APPS_VALUES          0x0D1ACCE1
#define CAN_ID_VCU_SUMMARY          0x0D1001F0
#define CAN_ID_BSE                  0x0D100B5E
#define CAN_ID_DEAD_CAR             0x0D10DEAD
#define CAN_ID_CAN_HEALTH           0x0D100CA9
#define CAN_ID_CAN_HEALTH_FIFO      0x0D10F1F0

#endif
