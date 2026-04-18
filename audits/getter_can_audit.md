# TREV4-Controls — Getter vs CAN TX Audit

Date: 2026-04-16

Scope:
- Codebase: `github/TREV4-Controls/src`
- “Getter” = functions that expose internal state/measurements (e.g. `*_GetData()`, `*_GetState()`, `*_IsActive()`, `RuntimeConfig_Get*()`, `CAN_Manager_Get*Data()`)
- “Sending as CAN messages” = values included in frames produced by `src/can/can_tx_pack.c` and scheduled by `src/can/can_manager.c`.

## CAN TX frames currently produced

From `src/can/can_manager.c` / `src/can/can_tx_pack.c`:

- `CAN_ID_APPS_VOLTAGES` (`CAN_TX_PackAPPSVoltages`)
- `CAN_ID_APPS_VALUES` (`CAN_TX_PackAPPSValues`)
- `CAN_ID_INV_TORQUE_COMMAND` (`CAN_TX_PackInvTorqueCommand`)
- `CAN_ID_INV_READ_WRITE` (`CAN_TX_PackInvReadWrite`)
- `CAN_ID_VCU_SUMMARY` (`CAN_TX_PackVCUSummary`)
- `CAN_ID_VCU_SETTINGS` (`CAN_TX_PackVCUSettings`)

Notes:
- `CAN_TX_PackVCUExitDriveReason` exists but is currently empty and not scheduled.

## Getter inventory and CAN coverage

### `APPS_GetData()` → `APPS_Data_t`
Files: `src/sensors/apps.h`, `src/sensors/apps.c`

Sent today:
- In `CAN_ID_APPS_VOLTAGES`: `apps1.raw_mv`, `apps1.filt_mv`, `apps2.raw_mv`, `apps2.filt_mv`
- In `CAN_ID_APPS_VALUES`: `apps1.value`, `apps2.value`, per-sensor flags (`valid/out_of_range/adc_err/stale`), and top-level flags (`valid`, `implausible`), plus `apps_value`

Not sent today (available internally, never packed into any TX frame):
- `APPS_Data_t.above_bap_threshold`
- `APPS_Data_t.below_bap_reestablish_threshold`

### `BSE_GetData()` → `BSE_Data_t`
Files: `src/sensors/bse.h`, `src/sensors/bse.c`

Sent today:
- None (no CAN TX frame uses `BSE_GetData()`)

Not sent today (entire struct is internal-only):
- `psi`, `brakes_engaged`, `hard_braking`
- `raw_mv`, `filt_mv`
- `valid`, `out_of_range`, `stale`, `adc_err`
- `last_update_us` (declared in header; currently not written in `BSE_Update()`)

### `TorqueController_GetData()` → `TorqueController_Data_T`
Files: `src/control/torque_controller.h`, `src/control/torque_controller.c`

Sent today:
- In `CAN_ID_INV_TORQUE_COMMAND`: `inv_torque_scaled`, `inv_direction`, `inv_enable`, `inv_speed_mode`

Not sent today:
- `apps_torque`
- `regen_torque`

### `StateMachine_GetState()` → `VCU_State_t`
Files: `src/state_machine.h`, `src/state_machine.c`

Sent today:
- In `CAN_ID_VCU_SUMMARY`: `VCU_State_t`

Not sent today:
- N/A (only returns the enum)

### `Buzzer_GetState()` → `Buzzer_State_t`
Files: `src/io/buzzer.h`, `src/io/buzzer.c`

Sent today:
- None

Not sent today:
- `BUZZER_STATE_*` (inactive / in_prog / done)

### `RTD_IsActive()` → `bool`
Files: `src/io/rtd.h`, `src/io/rtd.c`

Sent today:
- None

Not sent today:
- RTD switch state (debounced)

### `RuntimeConfig_*()`
Files: `src/settings/runtime_config.h`, `src/settings/runtime_config.c`

Sent today:
- `CAN_ID_VCU_SETTINGS` broadcasts one runtime parameter per message via `RuntimeConfig_GetNextBroadcastParam()`.
- Current parameter set is 1 item: `RUNTIME_PARAM_MAX_TORQUE`.

Gaps / caveats:
- `RuntimeConfig_GetMaxTorque()` is not directly packed, but the underlying parameter value is broadcast via `CAN_ID_VCU_SETTINGS` (as `param_id=1`).

### `CAN_Manager_Get*Data()` (RX mirror getters)
Files: `src/can/can_manager.h`, `src/can/can_manager.c`, `src/can/can_rx_unpack.h`, `src/can/can_rx_unpack.c`

These getters expose values received from CAN (controls bus RX). The VCU does **not** currently re-broadcast most of them in its own TX frames.

- `CAN_Manager_GetInverterHighSpeedData()` → `InverterHighSpeed_RX_Data_t`
  - Used in TX: `motor_speed` only (packed into `CAN_ID_VCU_SUMMARY` as abs(rpm))
  - Not re-broadcast by VCU TX: `torque_cmd`, `torque_feedback`, `dc_bus_voltage`

- `CAN_Manager_GetInverterStatusData()` → `InverterStatus_RX_Data_t`
  - Used in TX: none
  - Not re-broadcast by VCU TX: `run_faults`, `post_faults`
  - Also note: `CAN_RX_UnpackInverterStatus()` is TODO, so the struct is currently not populated.

- `CAN_Manager_GetHVCSummaryData()` → `HVCSummary_RX_Data_t`
  - Used in TX: none
  - Not re-broadcast by VCU TX: `sdc_ok`, `imd_ok`, `bms_ok`, `pack_voltage`, `pack_current`, `pack_soc`
  - Also note: `CAN_RX_UnpackHVCSummary()` currently hard-codes the OK flags TRUE and does not unpack voltage/current/SOC.

## Quick “what we have but aren’t sending” summary

Internal-only (no CAN TX coverage at all):
- All BSE data (`BSE_GetData()`)
- Buzzer state (`Buzzer_GetState()`)
- RTD active (`RTD_IsActive()`)
- Inverter Status RX data (not forwarded, and not unpacked)
- HVC Summary RX data (not forwarded; unpack stubbed)

Partially sent (some fields packed, others missing):
- APPS: missing BAP threshold flags
- Torque controller: missing `apps_torque` and `regen_torque`
- InverterHighSpeed RX: only speed is forwarded; torque and DC bus voltage are not
