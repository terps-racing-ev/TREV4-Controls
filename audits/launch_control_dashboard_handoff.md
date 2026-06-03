# Launch Control Dashboard Handoff

## Purpose

This document describes the implemented launch-control system in the VCU firmware so another agent can build a control dashboard against the real behavior and CAN contract.

This handoff covers:

- Learning mode workflow
- Actual launch mode workflow
- Separation from traction control
- Runtime-configurable parameters and their mux ids
- Command messages and state-machine behavior
- Telemetry messages and payload layout
- Safety constraints and edge cases
- Recommended dashboard behavior

## High-Level Model

The VCU has a traction-related overlay state machine implemented in:

- `src/control/traction_control_state_machine.h`
- `src/control/traction_control_state_machine.c`

This TRC layer is separate from the main VCU state machine in `state_machine.c`.

Important separation rules:

- Learning mode and actual launch mode are separate TRC modes.
- Only one of those two modes can be active at a time.
- Standard traction control is a separate function from launch control.
- Launch control does **not** depend on traction control being enabled in EEPROM.
- Launch torque is always pedal-bounded, so launch control can never command more torque than the driver is requesting.

## Modes

TRC states are defined in `src/control/traction_control_state_machine.h`.

Current state values:

| Value | State |
|---|---|
| 0 | `TRC_STATE_OFF` |
| 1 | `TRC_STATE_ACTIVE` |
| 2 | `TRC_STATE_LAUNCH_IDLE` |
| 3 | `TRC_STATE_LAUNCH` |
| 4 | `TRC_STATE_LEARNING_IDLE` |
| 5 | `TRC_STATE_LEARNING_READY_RUN1` |
| 6 | `TRC_STATE_LEARNING_RUNNING_RUN1` |
| 7 | `TRC_STATE_LEARNING_COMPLETE_RUN1` |
| 8 | `TRC_STATE_LEARNING_READY_RUN2` |
| 9 | `TRC_STATE_LEARNING_RUNNING_RUN2` |
| 10 | `TRC_STATE_LEARNING_COMPLETE_RUN2` |
| 11 | `TRC_STATE_LEARNING_READY_RUN3` |
| 12 | `TRC_STATE_LEARNING_RUNNING_RUN3` |
| 13 | `TRC_STATE_LEARNING_PROCESSING` |
| 14 | `TRC_STATE_LEARNING_RESULTS_READY` |
| 15 | `TRC_STATE_LEARNING_ABORTED` |
| 16 | `TRC_STATE_FAULT` |

Meaning:

- `ACTIVE` is the existing traction-control state.
- `LAUNCH_IDLE` means actual launch mode has been entered, but not armed yet.
- `LAUNCH` means actual launch has been armed and is waiting for throttle or already running.
- `LEARNING_*` states perform the 3-run learning workflow.

## Core Workflow

### Learning Mode Workflow

The intended operator flow is:

1. Put the car in a valid driving-ready condition.
2. Send TRC command `ENTER_LEARNING`.
3. Send TRC command `ARM` to arm run 1.
4. Driver applies throttle.
5. Run 1 starts automatically once throttle threshold conditions are met.
6. When run 1 completes, read telemetry.
7. Send `ARM` again to arm run 2.
8. Driver applies throttle.
9. Run 2 starts automatically.
10. When run 2 completes, read telemetry.
11. Send `ARM` again to arm run 3.
12. Driver applies throttle.
13. Run 3 starts automatically.
14. When run 3 completes, the VCU processes the results and reports the computed best curve.

Important behavior details:

- The VCU does **not** use brake-plus-throttle to start learning runs.
- The VCU does **not** use an RPM threshold to start learning runs.
- A learning run starts only after it has already been armed by CAN and then the throttle threshold is crossed.

### Actual Launch Workflow

The intended operator flow is:

1. Finish the 3 learning runs.
2. Read back the learned telemetry and best-curve result from the VCU.
3. Upload the desired actual launch curve data into EEPROM-backed runtime params.
4. Set the actual launch curve source to `UPLOADED`.
5. Send TRC command `ENTER_LAUNCH`.
6. Send TRC command `ARM_LAUNCH`.
7. Driver applies throttle.
8. Actual launch begins.

Important behavior details:

- Actual launch mode and learning mode are separate.
- Actual launch must be explicitly entered and then explicitly armed.
- Actual launch starts only after it is armed and the throttle threshold is crossed.
- The uploaded curve is distinct from the learned `BEST_CURVE` result.

## Start Conditions

Both learning runs and actual launch use the same throttle-only start gate implemented by `LaunchStartReady()` in `src/control/traction_control_state_machine.c`.

Start conditions:

- VCU state must be `VCU_STATE_DRIVING`
- APPS must be valid
- Brakes must **not** be engaged
- APPS value must be greater than or equal to the configured compile-time threshold

Current compile-time threshold:

- `LAUNCH_START_PEDAL_PERCENT = 80`

This threshold is defined in `src/config/torque_config.h`.

## End Conditions

### Learning Run End Conditions

Implemented in `ServiceRunningRun()`:

- Reached `TRC_LAUNCH_END_RPM`
- Brakes engaged
- Timeout exceeded (`TRC_LAUNCH_TIMEOUT_MS`)

Abort condition:

- Slip exceeds `TRC_LAUNCH_MAX_SLIP_X1000`

### Actual Launch End Conditions

Implemented in `ServiceActualLaunch()`:

- Reached `TRC_LAUNCH_END_RPM`
- Brakes engaged
- Timeout exceeded (`TRC_LAUNCH_TIMEOUT_MS`)

Behavior after end:

- Normal end returns from `TRC_STATE_LAUNCH` to `TRC_STATE_LAUNCH_IDLE`
- Explicit exit or abort returns to `TRC_STATE_OFF`
- Leaving `VCU_STATE_DRIVING` cancels actual launch and returns to `OFF`

## Learned Curves vs Actual Launch Curve

There are now **two different kinds of curves** in the system.

### 1. Learning Curves

These are fixed compile-time curves used only by the 3 learning runs:

- Curve A: `LAUNCH_CURVE_A_TORQUE_NM`
- Curve B: `LAUNCH_CURVE_B_TORQUE_NM`
- Curve C: `LAUNCH_CURVE_C_TORQUE_NM`

They share compile-time motor-RPM breakpoints:

- `LAUNCH_CURVE_RPM_BREAKPOINTS`

These are used only for learning comparisons.

### 2. Actual Launch Curve

This is the curve used by real launch mode after the learning workflow.

It can be selected from four sources using `RUNTIME_PARAM_TRC_LAUNCH_ACTIVE_CURVE`:

| Value | Meaning |
|---|---|
| 0 | Use compile-time Curve A |
| 1 | Use compile-time Curve B |
| 2 | Use compile-time Curve C |
| 3 | Use uploaded EEPROM-backed actual curve |

Default:

- `TRC_LAUNCH_ACTIVE_CURVE_DEFAULT = 3`

When source `3` is selected, actual launch uses uploaded runtime-config curve points, not the learned A/B/C tables.

## Uploaded Actual Launch Curve Parameters

These params are EEPROM-backed runtime config values and are sent/received through the existing `SET_VCU_CONFIG` / `VCU_Config` mux mechanism.

Relevant enum ids are in `src/config/runtime_config.h`.

### Existing Launch Params

| Param id | Name |
|---|---|
| 22 | `TRC_LAUNCH_ENABLED` |
| 23 | `TRC_LAUNCH_END_RPM` |
| 24 | `TRC_LAUNCH_TIMEOUT_MS` |
| 25 | `TRC_LAUNCH_MAX_SLIP_X1000` |
| 26 | `TRC_LAUNCH_BEST_CURVE` |
| 27 | `TRC_LAUNCH_ACTIVE_CURVE` |
| 28 | `TRC_LAUNCH_RECOMMENDED_SLIP_X1000` |

### New Uploaded Curve Params

| Param id | Name |
|---|---|
| 29 | `TRC_LAUNCH_ACTUAL_RPM_0` |
| 30 | `TRC_LAUNCH_ACTUAL_RPM_1` |
| 31 | `TRC_LAUNCH_ACTUAL_RPM_2` |
| 32 | `TRC_LAUNCH_ACTUAL_RPM_3` |
| 33 | `TRC_LAUNCH_ACTUAL_RPM_4` |
| 34 | `TRC_LAUNCH_ACTUAL_TORQUE_0` |
| 35 | `TRC_LAUNCH_ACTUAL_TORQUE_1` |
| 36 | `TRC_LAUNCH_ACTUAL_TORQUE_2` |
| 37 | `TRC_LAUNCH_ACTUAL_TORQUE_3` |
| 38 | `TRC_LAUNCH_ACTUAL_TORQUE_4` |

Default uploaded curve:

- RPM defaults match the shared compile-time breakpoints: `0, 1000, 2000, 4000, 6000`
- Torque defaults match Curve A: `40, 50, 65, 80, 90`

Sanitization behavior in firmware:

- Uploaded RPM points are forced non-negative.
- Uploaded torque points are forced non-negative.
- Uploaded RPM breakpoints are forced monotonically non-decreasing during lookup.

This means the dashboard should still validate monotonically increasing RPM points before writing them, but the firmware will refuse obviously broken decreasing sequences by flattening them.

## Command Channel

TRC commands do **not** use a dedicated CAN RX id.

They are carried inside the existing `SET_VCU_CONFIG` message using reserved mux id:

- `SET_VCU_CONFIG_TRC_COMMAND_MUX = 240`

Message id:

- `CAN_ID_SET_VCU_CONFIG = 0x000000CF`

Command values:

| Value | Command |
|---|---|
| 0 | `NONE` |
| 1 | `SET_ACTIVE` |
| 2 | `SET_OFF` |
| 3 | `ENTER_LEARNING` |
| 4 | `ARM` |
| 5 | `ABORT` |
| 6 | `EXIT_LEARNING` |
| 7 | `ENTER_LAUNCH` |
| 8 | `ARM_LAUNCH` |
| 9 | `EXIT_LAUNCH` |

### Practical Command Sequences

#### Learning sequence

1. Send mux `240`, value `3` (`ENTER_LEARNING`)
2. Send mux `240`, value `4` (`ARM`) for run 1
3. Wait for run 1 to finish
4. Send mux `240`, value `4` (`ARM`) for run 2
5. Wait for run 2 to finish
6. Send mux `240`, value `4` (`ARM`) for run 3
7. Wait for `LEARNING_RESULTS_READY`

#### Actual launch sequence

1. Upload or confirm actual launch params
2. Set `TRC_LAUNCH_ACTIVE_CURVE = 3` if using uploaded curve
3. Send mux `240`, value `7` (`ENTER_LAUNCH`)
4. Send mux `240`, value `8` (`ARM_LAUNCH`)
5. Driver applies throttle

#### Exit/cancel sequences

- Send `5` (`ABORT`) to cancel a learning run or actual launch
- Send `6` (`EXIT_LEARNING`) to leave learning mode
- Send `9` (`EXIT_LAUNCH`) to leave actual launch mode
- Send `2` (`SET_OFF`) to force the TRC layer back to `OFF`

## Telemetry Messages

### 1. `VCU_TRC_State`

CAN id:

- `0x0D1008C0`

Sent every:

- `100 ms`

Purpose:

- High-level TRC state summary
- Learning/launch mode summary
- Best curve summary
- Grip scores A and B
- Recommended slip

Payload layout from `CAN_TX_PackVCUTrcState()`:

| Byte(s) | Meaning |
|---|---|
| 0 | `state` |
| 1 bits 0-1 | `current_run` |
| 1 bits 2-3 | `selected_curve` |
| 1 bit 4 | `learning_active` |
| 1 bit 5 | `launch_mode_active` |
| 1 bits 6-7 | `best_curve` |
| 2-3 | `grip_score_a_x1000` |
| 4-5 | `grip_score_b_x1000` |
| 6-7 | `recommended_slip_x1000` |

`selected_curve` meanings:

| Value | Meaning |
|---|---|
| 0 | Curve A |
| 1 | Curve B |
| 2 | Curve C |
| 3 | Uploaded actual-launch curve |

Important limitation:

- `VCU_TRC_State` does **not** carry grip score C because the frame is full.
- That is why the new `VCU_TRC_Run_Data` frame exists.

### 2. `VCU_TRC_Run_Data`

CAN id:

- `0x0D1008C1`

Sent every:

- `100 ms`

Purpose:

- Report run-specific learning results for all three runs.

Behavior:

- The frame multiplexes internally by cycling `RUN1 -> RUN2 -> RUN3 -> RUN1 ...`
- Full refresh of all three runs occurs every `300 ms`

Payload layout from `CAN_TX_PackVCUTrcRunData()`:

| Byte(s) | Meaning |
|---|---|
| 0 | `run_index` (`1`, `2`, `3`) |
| 1 bit 0 | `run_valid` (`sample_count > 0`) |
| 2-3 | `avg_slip_x1000` for that run |
| 4-5 | `peak_slip_x1000` for that run |
| 6-7 | `grip_score_x1000` for that run |

This is the frame the dashboard should use to display data from **all 3 runs**.

### 3. `VCU_Config`

CAN id:

- `0x001000CF`

Purpose:

- Broadcast runtime config params back out over CAN, including launch-related params and uploaded actual-launch curve points.

This means the dashboard can:

- Write a launch param with `SET_VCU_CONFIG`
- Listen for `VCU_Config`
- Verify the echoed/stored value

## Learning Metrics

The learning helper stores these metrics per run:

- `avg_slip_x1000`
- `peak_slip_x1000`
- `grip_score_x1000`
- sample count

Meaning of slip ratio:

- `1000 = 1.000 = nominal no-slip ratio`
- Greater than `1000` means more rear-wheel overspeed relative to front wheels

Grip score:

- Computed as a relative score, not an absolute friction coefficient
- Higher score is better
- Score is based on average slip, not acceleration

Important physics limitation:

- The firmware has no onboard accelerometer reference for absolute tire-road friction estimation
- Front undriven wheels are used as the ground-reference approximation
- This means the result is a **relative comparison** between tested curves, not absolute $\mu$

## Best-Curve Result

After run 3 completes, `LearningMode_ProcessResults()` computes:

- `best_curve`
- `recommended_slip_x1000`

Current heuristic:

- Prefer the most aggressive tested curve that stayed at or below `TRC_LAUNCH_MAX_SLIP_X1000`
- If no curve stayed under that safety ceiling, fall back conservatively

The best curve result is stored in:

- `RUNTIME_PARAM_TRC_LAUNCH_BEST_CURVE`

This is a **result**. It is not automatically the actual launch curve unless the operator chooses to use it.

## Actual Launch Torque Behavior

Actual launch torque is generated in `TractionControlSM_GetLaunchTorque()`.

Rules:

- If actual launch is armed but throttle has not started the run yet, no launch torque is applied.
- Once throttle start is detected, launch torque uses the selected source curve.
- If `selected_curve == 3`, the uploaded actual-launch curve is used.
- If `selected_curve` is `0`, `1`, or `2`, compile-time curves A/B/C are used.
- Final launch torque is clamped so it never exceeds the driver-requested pedal torque.

This pedal bound is critical for compliance and safety.

## Independence From Standard Traction Control

This is a hard requirement and the current implementation honors it.

Behavior:

- Standard traction control enable/disable is controlled by `RUNTIME_PARAM_TRACTION_CONTROL_ENABLED`
- Launch control mode does not require that param to be enabled
- Actual launch torque path is handled by the TRC state machine overlay and is independent of the normal traction-control PID enable bit

Dashboard implication:

- Do not disable or gray out launch-control UI just because traction control is disabled
- Treat traction control and launch control as separate operator functions

## Recommended Dashboard UX

### Dashboard sections

The dashboard should ideally have 4 separate sections.

#### 1. Learning Control

- Button: `Enter Learning`
- Button: `Arm Next Learning Run`
- Button: `Abort`
- Button: `Exit Learning`
- Live indicator of current learning state
- Live indicator of currently selected learning curve (A/B/C)

#### 2. Learning Results

- Best curve result
- Recommended slip
- Per-run cards or table for:
  - Run 1 avg slip
  - Run 1 peak slip
  - Run 1 grip score
  - Run 2 avg slip
  - Run 2 peak slip
  - Run 2 grip score
  - Run 3 avg slip
  - Run 3 peak slip
  - Run 3 grip score

#### 3. Actual Launch Curve Editor

- Selector for active curve source: A / B / C / Uploaded
- 5 editable RPM breakpoint fields
- 5 editable torque fields
- Readback/echo display of uploaded stored values
- Validation for monotonic RPM breakpoints

#### 4. Actual Launch Control

- Button: `Enter Launch`
- Button: `Arm Launch`
- Button: `Abort`
- Button: `Exit Launch`
- Live launch-state display
- Live indicator of whether launch mode is armed/active

### Guardrails the dashboard should implement

- Prevent arming learning or launch when the current TRC state is not compatible
- Show uploaded actual-launch curve params after readback, not just what the user typed
- If using uploaded curve source, verify all 10 uploaded fields before arming launch
- Prefer showing `selected_curve` names, not raw numeric ids
- Display run-data values in engineering units by converting `x1000` ratios to decimal

Example:

- `1100` should display as `1.100`

## Recommended Dashboard Polling / Subscription Strategy

Use these telemetry sources together:

- `VCU_TRC_State` for state, active mode flags, best curve, recommended slip, summary grip A/B
- `VCU_TRC_Run_Data` for run-specific avg/peak/grip across all 3 runs
- `VCU_Config` for readback of launch params and uploaded actual-launch curve values

Operationally:

- Treat `VCU_TRC_State` as the high-level status source
- Treat `VCU_TRC_Run_Data` as the authoritative per-run result source
- Treat `VCU_Config` as the authoritative EEPROM/runtime-param readback source

## Relevant Source Files

Primary implementation files:

- `src/control/traction_control_state_machine.h`
- `src/control/traction_control_state_machine.c`
- `src/control/learning_mode.h`
- `src/control/learning_mode.c`
- `src/control/torque_controller.c`

Runtime config and persistence:

- `src/config/runtime_config.h`
- `src/config/runtime_config.c`
- `src/config/torque_config.h`

CAN transport:

- `src/config/can_config.h`
- `src/can/can_manager.h`
- `src/can/can_manager.c`
- `src/can/can_tx.h`
- `src/can/can_tx.c`
- `scripts/generate_vcu_dbc.py`
- `VCU.dbc`

## Summary for the Dashboard Agent

The dashboard should assume the following:

- Learning mode and actual launch mode are separate operator workflows.
- Only one of them should be active at a time.
- Learning is driven by `ENTER_LEARNING` + repeated `ARM` commands.
- Actual launch is driven by `ENTER_LAUNCH` + `ARM_LAUNCH`.
- Start of either mode is throttle-only after arm; no brake+throttle initiation.
- Best-curve result is reported by the VCU but is not automatically forced into actual launch.
- The operator can upload a separate actual-launch curve and select `ACTIVE_CURVE = UPLOADED`.
- Standard traction control enable/disable is separate from launch control capability.

If a future dashboard agent needs a minimal implementation target, the minimum viable operator flow is:

1. Enter learning
2. Arm run 1
3. Read run data
4. Arm run 2
5. Read run data
6. Arm run 3
7. Read best curve and recommended slip
8. Upload actual-launch curve params
9. Set active launch curve source to `UPLOADED`
10. Enter launch
11. Arm launch
