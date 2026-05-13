# PLAN — CAN-Based APPS Calibration (Open/Pressed)

## Goal
Add CAN commands that calibrate the APPS sensors by capturing the *current* sensor readings:
- **Calibrate Open**: store current readings as **min mV**.
- **Calibrate Pressed**: store current readings as **max mV**.

Calibration must:
- Be **persistent** across power cycles.
- Be **blocked while driving** (safety gating).
- Be **robust** against invalid bounds (sanity checks + fallback to defaults).

## Summary of Approach
1. Add an EEPROM-backed storage for APPS bounds by extending `RuntimeConfig`.
2. Update `APPS_Update()` to use runtime-configured min/max (with sanitization).
3. Add a dedicated CAN RX command frame `CAN_ID_APPS_CALIBRATE`.
4. Implement `CAN_RX_UnpackAppsCalibrate()` that:
   - Rejects calibration while driving / RTD active.
   - Captures filtered mV readings (`appsX.filt_mv`).
   - Writes min/max via `RuntimeConfig_Set()` with range validation.

## Key Constants (specific)
- **CAN ID**: `CAN_ID_APPS_CALIBRATE = 0x000000D0` (extended frame, controls channel)
- **Minimum valid span**: `MIN_SPREAD_MV = 200` mV
  - Used to prevent divide-by-zero and unstable scaling.

---

## Phase 1 — Persisted storage (RuntimeConfig)

### Objective
Store these 4 values in EEPROM:
- APPS1 min mV
- APPS1 max mV
- APPS2 min mV
- APPS2 max mV

### Changes
1. Update enum
   - File: `src/config/runtime_config.h`
   - Add:
     - `RUNTIME_PARAM_APPS1_MIN_MV`
     - `RUNTIME_PARAM_APPS1_MAX_MV`
     - `RUNTIME_PARAM_APPS2_MIN_MV`
     - `RUNTIME_PARAM_APPS2_MAX_MV`
   - Keep `RUNTIME_PARAM_COUNT` last.

2. Extend stored data
   - File: `src/config/runtime_config.c`
   - Extend `RuntimeConfig_Data_t` with 4 `sbyte2` fields for those params.

3. Add descriptors
   - File: `src/config/runtime_config.c`
   - Add 4 entries to `param_descs[]`:
     - Defaults from `src/config/apps_config.h`:
       - `APPS_1_MIN_VOLTAGE`, `APPS_1_MAX_VOLTAGE`
       - `APPS_2_MIN_VOLTAGE`, `APPS_2_MAX_VOLTAGE`
     - Clamp range:
       - `min_value = 0`
       - `max_value = 5000`

### Notes
- `RuntimeConfig_Set()` already triggers EEPROM write (coalesced) and triggers config TX.
- `RuntimeConfig_Task()` already handles invalid EEPROM blobs by restoring defaults and writing them.

### Acceptance Criteria
- On first boot after flashing, bounds match today’s compile-time defaults.
- After calibration, bounds persist after power cycle.

---

## Phase 2 — Use runtime bounds in APPS scaling

### Objective
Replace compile-time APPS bounds usage in `APPS_Update()` with runtime-configured values.

### Changes
1. Read bounds each update
   - File: `src/sensors/apps.c`
   - Read min/max for each sensor from `RuntimeConfig`.

2. Sanitize bounds per sensor
Before calling `UpdateChannel()`:
- If `max_mv <= min_mv`, use compile-time defaults.
- If `(max_mv - min_mv) < MIN_SPREAD_MV`, use compile-time defaults.

### Rationale
`VoltageToPedalTravel()` divides by `(max_mv - min_mv)`. Sanitization guarantees this is safe.

### Acceptance Criteria
- With default runtime bounds, APPS behavior matches current behavior.
- With calibrated bounds, APPS maps near 0 (open) and near `APPS_RESOLUTION` (pressed).

---

## Phase 3 — CAN RX command: `CAN_ID_APPS_CALIBRATE`

### Objective
Provide a minimal, dedicated CAN message to trigger calibration.

### Message definition
- **CAN ID**: `0x000000D0`
- **Frame format**: extended (`IO_CAN_EXT_FRAME`)
- **Channel**: `CONTROLS_CAN_CHANNEL`
- **DLC**: at least 1

### Payload
- Byte0: `apps_cal_cmd`
  - `0` = CAL_OPEN (capture min)
  - `1` = CAL_PRESSED (capture max)
- Bytes1..7: reserved/ignored

### Integration points
1. Add CAN ID
   - File: `src/config/can_config.h`
   - Add `#define CAN_ID_APPS_CALIBRATE 0x000000D0`

2. Add RX message enum entry
   - File: `src/can/can_manager.h`
   - Add `CAN_RX_MSG_APPS_CALIBRATE`

3. Register RX message
   - File: `src/can/can_manager.c`
   - Add entry to `rx_messages[]`:
     - `.channel = CONTROLS_CAN_CHANNEL`
     - `.id_format = IO_CAN_EXT_FRAME`
     - `.id = CAN_ID_APPS_CALIBRATE`
     - `.timeout_us = MSG_TIMEOUT_US`
     - `.decode_fn = CAN_RX_UnpackAppsCalibrate`

4. Add handler
   - File: `src/can/can_rx.h`
     - Declare `void CAN_RX_UnpackAppsCalibrate(IO_CAN_DATA_FRAME* frame);`
   - File: `src/can/can_rx.c`
     - Implement handler.

### Acceptance Criteria
- Receiving this frame triggers calibration (only when allowed by gating and sanity checks).

---

## Phase 4 — Calibration logic details (gating + capture + validation)

### Safety gating (must-pass)
In `CAN_RX_UnpackAppsCalibrate()`:
1. Reject if `frame == NULL` or `frame->length < 1`.
2. Reject if EEPROM load not complete:
   - `RuntimeConfig_IsEepromLoadComplete() == FALSE`
3. Reject if driving / torque could be applied:
   - `StateMachine_GetState() == VCU_STATE_DRIVING` OR
   - `RTD_IsActive() == TRUE`

### Sensor-data gating
- Reject if either sensor has unusable ADC data:
  - `apps->apps1.adc_err || apps->apps1.stale || apps->apps2.adc_err || apps->apps2.stale`
- Do **not** require `out_of_range == FALSE` (bad bounds may cause out-of-range; calibration needs to fix that).

### Capture source
- Use filtered voltages:
  - APPS1: `apps->apps1.filt_mv`
  - APPS2: `apps->apps2.filt_mv`

### Apply semantics
- If `apps_cal_cmd == 0` (CAL_OPEN): set BOTH mins.
- If `apps_cal_cmd == 1` (CAL_PRESSED): set BOTH maxes.

### Validation against complementary bound
To avoid creating invalid bounds:
- Before setting a min, read current max.
- Before setting a max, read current min.
- Only accept if `max > min + MIN_SPREAD_MV`.

**Policy (specific):** all-or-nothing per command
- If either sensor would become invalid, reject the entire calibration command.

### Acceptance Criteria
- Calibration is ignored while driving.
- Calibration cannot create invalid min/max pairs.
- After calibration, scaled APPS behaves correctly.

---

## Observability
Existing CAN TX already supports validation:
- `CAN_ID_APPS_VOLTAGES`: verify filtered mV stabilization.
- `CAN_ID_APPS_VALUES`: verify scaling near 0 open / near resolution pressed.
- `CAN_ID_CONFIG`: after adding runtime params, bounds can be observed (mux cycles through params).

---

## Verification Procedure

### Build
- Build using the standard toolchain (example):
  - `make COMPILER=viper TARGET=TTC60`

### Bench test
1. Pedal **released**:
   - Watch `CAN_ID_APPS_VOLTAGES` until `apps1.filt_mv` and `apps2.filt_mv` settle.
   - Send `CAN_ID_APPS_CALIBRATE` with byte0 = `0`.
2. Pedal **fully pressed**:
   - Watch until filtered voltages settle.
   - Send `CAN_ID_APPS_CALIBRATE` with byte0 = `1`.

### Expected results
- Pedal released: `apps_value` near 0 (respecting deadzone).
- Pedal pressed: `apps_value` near `APPS_RESOLUTION`.
- Sensors no longer flagged out-of-range during normal operation.

### Persistence
- Power-cycle, confirm behavior remains calibrated.

### Negative tests
- Attempt calibration while driving/RTD active: must be ignored.
- Attempt calibration with stale/errored ADC: must be ignored.

---

## Implementation Checklist (file-by-file)

1. `src/config/can_config.h`
   - Add `CAN_ID_APPS_CALIBRATE`.

2. `src/can/can_manager.h`
   - Add `CAN_RX_MSG_APPS_CALIBRATE`.

3. `src/can/can_manager.c`
   - Register new RX message.

4. `src/can/can_rx.h`
   - Declare `CAN_RX_UnpackAppsCalibrate`.

5. `src/can/can_rx.c`
   - Implement handler, includes for:
     - `state_machine.h`
     - `io/rtd.h`
     - `sensors/apps.h`
     - `config/runtime_config.h`

6. `src/config/runtime_config.h`
   - Add 4 runtime param IDs.

7. `src/config/runtime_config.c`
   - Add 4 stored fields + param descriptors.

8. `src/sensors/apps.c`
   - Read runtime bounds + sanitize + pass to `UpdateChannel()`.
