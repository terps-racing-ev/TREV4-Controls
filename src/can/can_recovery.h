#ifndef CAN_RECOVERY_H
#define CAN_RECOVERY_H

#include "IO_Constants.h"
#include "IO_CAN.h"

/*
CAN Recovery was mostly AI generated with little caleb review.
hopefully this code never runs.
*/


typedef struct {
    ubyte2 non_ok_cycles;
    ubyte2 bus_off_cycles;
    ubyte2 cooldown_cycles;
    ubyte2 recovery_attempts;
    bool recovery_requested;
    bool can_ready;
} CAN_Recovery_State_t;

/**
 * Initialize CAN recovery state
 */
void CAN_Recovery_Init(void);

/**
 * Deinitialize CAN recovery state
 */
void CAN_Recovery_DeInit(void);

/**
 * Update recovery state based on CAN channel status
 * @param controls_status Status of the controls CAN channel
 * @param daq_status Status of the DAQ CAN channel
 */
void CAN_Recovery_UpdateRecoveryState(IO_ErrorType controls_status, IO_ErrorType daq_status);

/**
 * Run recovery procedure if needed
 * This should be called each cycle to check if recovery is needed and execute it
 * @param init_callback Callback to reinitialize CAN (typically CAN_Manager_Init)
 * @param deinit_callback Callback to deinitialize CAN (typically CAN_Manager_DeInit)
 */
void CAN_Recovery_RunRecovery(void (*init_callback)(void), void (*deinit_callback)(void));

/**
 * Get current recovery state
 * @return Pointer to recovery state structure
 */
const CAN_Recovery_State_t* CAN_Recovery_GetState(void);

/**
 * Check if CAN is ready for operation
 * @return True if CAN is ready, false otherwise
 */
bool CAN_Recovery_IsReady(void);

/**
 * Mark all health counters as faulted (used after recovery init fails)
 * @param set_tx_fault If true, set TX fault
 * @param set_rx_fault If true, set RX fault
 */
void CAN_Recovery_MarkHealthFault(bool set_tx_fault, bool set_rx_fault);

#endif // CAN_RECOVERY_H
