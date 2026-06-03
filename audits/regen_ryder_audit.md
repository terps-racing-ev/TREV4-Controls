# TREV4-Controls — Ryder Regen Strategy Audit

Date: 2026-06-03

Scope:
- Strategy under review: `REGEN_STRATEGY_RYDER`
- Primary files reviewed:
  - `src/control/torque_controller.c`
  - `src/control/torque_controller.h`
  - `src/config/torque_config.h`
  - `src/config/runtime_config.c`
  - `src/config/runtime_config.h`
  - `src/can/can_tx.c`
  - `VCU.dbc`

## Strategy summary

Current Ryder behavior in `src/control/torque_controller.c` is:

- Regen only runs in `VCU_STATE_DRIVING` and only after the global regen enable, speed, and optional SOC gates pass.
- Ryder positive regen torque is computed as:
  - `front_table_torque = LookupRyderFrontTorque(front_psi)`
  - `balance_torque = CalculateRyderBalanceTorque(front_psi, rear_psi_x10)`
  - `requested_torque = min(balance_torque, front_table_torque, REGEN_MAX_TORQUE)`
- The inverter command is then the negated value: `-requested_torque`.
- Debug coverage is reasonably strong: `VCU_Regen_Debug` exposes status flags, block reason, pressures, front-table torque, balance torque, and final positive torque (`VCU.dbc:155-175`, `VCU.dbc:382`).

## Findings

### 1. High: Max-SOC gating fails open when HVC SOC is invalid

Evidence:
- `src/control/torque_controller.c:351` only blocks on high SOC when both conditions are true:
  - SOC gate is enabled
  - HVC SOC frame is valid
- If the HVC SOC frame is stale or missing, the strategy does not block regen at all and proceeds into the Ryder torque path.

Why this matters:
- The system default is to keep SOC gating enabled, but the current implementation only enforces it when the HVC SOC message is present.
- Any HVC SOC receive failure becomes a fail-open path for regen at high pack SOC.

Recommendation:
- If `REGEN_SOC_GATE_ENABLED` is on, treat invalid/stale HVC SOC as a blocking condition instead of allowing Ryder regen to proceed.
- Add a dedicated debug reason such as `REGEN_BLOCK_SOC_INVALID` so this is visible on-car.

### 2. Medium: `REGEN_MIN_TORQUE` is ignored entirely in Ryder mode

Evidence:
- The generic regen interpolation path reads `RUNTIME_PARAM_REGEN_MIN_TORQUE` in `src/control/torque_controller.c:131`.
- The Ryder path does not use that parameter anywhere. Its final clamp only applies `REGEN_MAX_TORQUE` in `src/control/torque_controller.c:235` and `src/control/torque_controller.c:254-255`.
- The parameter is still exposed as a runtime-configurable setting in `src/config/runtime_config.c:226` and has a default in `src/config/torque_config.h:38`.

Why this matters:
- Operators can tune `REGEN_MIN_TORQUE` over CAN and see it echoed in config traffic, but in the default Ryder strategy it has no effect.
- That is a contract mismatch between the tuning surface and the actual control law.

Recommendation:
- Either apply `REGEN_MIN_TORQUE` to Ryder output as a floor/entry clamp, or document and hide the parameter for Ryder mode.

### 3. Medium: Ryder mode bypasses all runtime BSE pressure threshold parameters

Evidence:
- The legacy regen helper reads front/rear min/max PSI thresholds in `src/control/torque_controller.c:286-289`.
- That same helper immediately exits for Ryder at `src/control/torque_controller.c:296-297`.
- The actual Ryder path only uses the compile-time lookup/equation plus the hard-coded `REGEN_RYDER_MAX_FRONT_PSI` guard (`src/control/torque_controller.c:226`, `src/config/torque_config.h:39-42`, `src/config/torque_config.h:50-58`).
- The bypassed PSI parameters are still runtime-configurable in `src/config/runtime_config.c:233-254` and still have defaults in `src/config/torque_config.h:39-42`.

Why this matters:
- In the default strategy, `REGEN_MIN_BSE_FRONT_PSI`, `REGEN_MIN_BSE_REAR_PSI`, `REGEN_MAX_BSE_FRONT_PSI`, and `REGEN_MAX_BSE_REAR_PSI` do not shape Ryder behavior at all.
- Anyone tuning regen from the dashboard can change those values and believe they are affecting Ryder when they are not.

Recommendation:
- Either incorporate those thresholds into the Ryder gating/clamping path, or clearly mark them as legacy-only and suppress them in Ryder-facing tooling.

### 4. Low: Missing rear data can be reported as `FRONT_INVALID`

Evidence:
- `src/control/torque_controller.c:211-213` returns `REGEN_BLOCK_FRONT_INVALID` when either `front_bse == NULL` or `mobo_power == NULL`.
- Rear-message freshness is handled separately as `REGEN_BLOCK_REAR_INVALID` in `src/control/torque_controller.c:221-223`, but a null rear pointer never reaches that branch.

Why this matters:
- This does not change the torque result, but it weakens the debug signal during bring-up and trackside troubleshooting.

Recommendation:
- Split the null checks so `mobo_power == NULL` reports `REGEN_BLOCK_REAR_INVALID`.

## Positive observations

- The top-level sign handling is correct for inverter regen: Ryder computes a positive magnitude and returns the negated torque command to the inverter path in `src/control/torque_controller.c:363-368`.
- Ryder is bounded by `REGEN_MAX_TORQUE`, which remains runtime-configurable in `src/control/torque_controller.c:235` and `src/config/runtime_config.c:219`.
- Telemetry coverage is strong enough to validate the control law live:
  - status and block reason: `VCU_Regen_Debug` mux 0 (`VCU.dbc:155-169`)
  - front/rear pressure and inverter command: mux 1 (`VCU.dbc:170-172`)
  - front-table, balance, and final torque: mux 2 (`VCU.dbc:173-175`)

## Recommended next steps

1. Make SOC gating fail-safe when enabled and HVC SOC is invalid.
2. Decide whether Ryder should honor the existing `REGEN_MIN_TORQUE` and BSE PSI threshold parameters.
3. If those parameters are intentionally legacy-only, remove or relabel them in dashboard tooling when Ryder is selected.
4. Fix the `FRONT_INVALID` vs `REAR_INVALID` debug attribution for null rear telemetry.