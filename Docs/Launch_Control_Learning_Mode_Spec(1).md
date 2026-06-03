# Launch Control Learning Mode Automation Specification

**Project:** TREV4 Controls (UMD Terps Racing FSAE EV)  
**Target Hardware:** HYDAC TTC60 VCU (C firmware)  
**Branch:** `traction_control` (or create feature branch from it)  
**Goal:** Automate launch control characterization with minimal driver intervention. Perform two acceleration runs with different torque curves, measure wheel slip (motor speed vs front wheels), auto-calculate estimated μ (friction), and output results via CAN. Build foundation for later closed-loop PID traction control.

**Intended Audience:** AI coding agent / firmware engineer implementing this feature.  
**Date:** June 2026

---

## 1. Overview & Objectives

### Primary Goal
Implement a **semi-automated Launch Learning Mode** on the existing TTC60 VCU firmware that:
1. Driver triggers "Learning Mode" via CAN command or input.
2. VCU guides through **two straight-line acceleration runs** using two different pre-defined torque curves.
3. During each run, the VCU measures and logs **wheel slip ratio** (using motor RPM vs average front wheel RPM).
4. After both runs, the VCU automatically computes a simple **estimated μ** (or relative grip metric) and recommends the better curve / parameters.
5. Results are broadcast via CAN to dash, telemetry (CANedge2), and service tool.
6. Minimal human involvement beyond performing the two clean launches.

### Why This Matters
- Reduces setup time and human variability when characterizing tires/track grip.
- Provides quick on-car feedback for launch strategy.
- Creates data foundation for adaptive / closed-loop traction control (your later PID goal).
- Leverages existing wheel speed CAN messages (10 ms / 100 Hz — proven sufficient from Bosch M5 ABS usage in motorsport).

### Success Criteria
- State machine cleanly manages learning sequence with clear dash prompts.
- Slip calculation is accurate and robust to async CAN messages.
- μ estimation is simple but useful (tunable via runtime params).
- Safety interlocks prevent misuse on track.
- Code integrates cleanly with existing `traction_control.c`, `state_machine.c`, `torque_controller.c`, runtime config, and CAN layers.
- RAM/CPU usage remains well within TTC60 limits (use running stats, not full buffers where possible).

---

## 2. Requirements

### Functional Requirements
- **FR1**: Enter/Exit Learning Mode via CAN message (or fallback button/pedal sequence).
- **FR2**: Support two torque curves (Curve A = conservative, Curve B = aggressive). Curves selectable or pre-loaded; applied during respective runs.
- **FR3**: Detect run start (vehicle speed > threshold + torque request > threshold) and run end (speed target reached, brake applied, or timeout).
- **FR4**: Compute slip ratio in real time:
  ```
  slip_ratio = (motor_rpm * gear_ratio * tire_radius - v_front_avg) / v_front_avg
  ```
  (Handle units, sign, and low-speed cases gracefully.)
- **FR5**: During "high torque phase" of each run, accumulate statistics: peak slip, average slip, sample count, optionally longitudinal accel correlation.
- **FR6**: After Run 2, compute estimated μ (or grip score) for each curve and determine "winner" or recommended scaling.
- **FR7**: Output results via CAN (status, prompts, final μ values, recommended params).
- **FR8**: Automatically trigger detailed logging on CANedge2 during learning runs (via CAN flag).
- **FR9**: Store learned values in runtime config / EEPROM for persistence across power cycles.

### Non-Functional Requirements
- **Performance**: Slip calculation + stats update at 10–20 ms loop rate. State machine responsive.
- **Robustness**: Handle missing/invalid wheel speed messages, sensor noise, async CAN data. Graceful degradation.
- **Safety**: Multiple interlocks (brake pressed, RTD off, excessive slip, timeout, driver abort). Learning mode must not increase risk.
- **Maintainability**: Modular, well-commented C code following existing style. Use existing patterns (runtime params, CAN RX structs, etc.).
- **Testability**: Bench-testable with simulated CAN data; on-car validation with conservative curves first.

### Safety & Constraints (Critical)
- Learning mode **disabled by default** and requires explicit enable + confirmation.
- Max torque limited during learning runs (configurable, start conservative).
- Abort on: brake pedal, RTD off, vehicle speed > safe limit, slip > dangerous threshold, timeout.
- No autonomous driving — driver must initiate and control each launch.
- Clear visual/audible feedback via dash lights + CAN text/status messages.
- All learned values are **suggestions** — driver/team still validates before competition use.

---

## 3. Current Codebase Analysis (Key Files)

From `traction_control` branch analysis:

**Existing Strengths (Build On These)**
- `src/control/traction_control.c/h` — Already has PID slip limiting, front wheel RPM via `CAN_RX_GetFrontLeftRpmData()` / Right, runtime param access (`GetParam`, `GetParamX1000`), `ResetPid()`, torque limiting.
- `src/control/torque_controller.c/h` — Torque request handling.
- `src/state_machine.c/h` — Central FSM — **extend this**.
- `src/main.c` — Clean init + main loop structure.
- `src/config/runtime_config.*` + `torque_config.*` — Perfect for tunable learning params (slip targets, μ thresholds, curve scalings).
- CAN layer (`can/can_manager.h`, `can/can_rx.h`, `VCU.dbc`) — Already receiving wheel speeds and motor data.
- Power limiting and sensor layers.

**Gaps to Fill**
- No dedicated learning states or sequence logic.
- No run statistics / μ estimation.
- No "test curve" application during learning.
- Limited onboard result output.

**Strongly Recommended Architecture**: 
- Create a **new dedicated Traction Control State Machine** (`src/control/traction_control_state_machine.c/h`).
- The learning mode becomes a sub-system called from within the TRC state machine.
- Existing `traction_control.c` logic (PID, slip limiting) should be refactored to live under TRC state control.
- `learning_mode.c/h` is a focused helper (stats + μ calc) invoked by the TRC state machine during learning states.
- Leave the original `state_machine.c/h` almost untouched — it continues to manage only core `VCU_STATE`.
- Result: Clean separation with `VCU_TRC_STATE` as the independent traction/launch layer status message.

---

## 4. Proposed Architecture (Updated: Layered State Machines)

**Design Principle**: Keep the core vehicle functionality clean and independent.  
- **Core VCU State Machine** (`VCU_STATE` message) — handles bare functionality: RTD, drive modes, faults, basic torque request, safety interlocks. This remains the primary state machine in `state_machine.c/h`.
- **Traction Control State Machine** (`VCU_TRC_STATE` message) — **new independent secondary state machine** that runs as an overlay/layer on top. It owns all traction-related logic, including the closed-loop PID (future) and the Launch Control Learning Mode.

Launch Control Learning becomes a **sub-mode inside the Traction Control layer** (not polluting the core VCU states). This gives clean separation, easier enable/disable of traction features, and safer boundaries.

### Recommended Structure

```
main.c (main loop @ 10-20 ms)
│
├── CoreStateMachine_Update()          // Existing state_machine.c/h → VCU_STATE
│   └── Handles: RTD, DriveMode, Faults, basic torque path, safety
│
├── TractionControl_StateMachine_Update()   // NEW: traction_control_state_machine.c/h → VCU_TRC_STATE
│   └── Owns: TRC_OFF / TRC_ACTIVE / TRC_LEARNING_*
│       └── Inside LEARNING states: calls LearningMode_Update()
│
├── TorqueController_Update()
│   └── If (TRC_State == TRC_ACTIVE or LEARNING) → apply traction limiting / learning curve
│       Else → pass-through or basic limiting
│
└── CAN + Runtime Config
    ├── VCU_STATE     (existing - core)
    └── VCU_TRC_STATE (new - traction layer status + learning prompts/results)
```

**Benefits of this approach**
- Core safety states stay simple and auditable.
- Traction features (including learning) can be completely disabled or developed independently.
- Future closed-loop PID lives naturally inside the TRC state machine.
- Learning mode states (`LEARNING_IDLE`, `READY_RUN1`, `RUNNING_RUN1`, `COMPLETE_RUN1`, `READY_RUN2`, `RUNNING_RUN2`, `PROCESSING`, `RESULTS_READY`) live cleanly under `TRC_LEARNING_*`.
- Easier to add more traction modes later (e.g., `TRC_RACE`, `TRC_WET`, `TRC_ENDURANCE`).

**Data Flow During Learning (now under TRC layer)**
1. CAN command or dash menu → Core state machine stays in normal drive → TRC_State transitions to `TRC_LEARNING_IDLE`
2. Select curve + confirm → `TRC_LEARNING_READY_RUN1` → Dash: "TRC Learning – Curve A – Ready for Run 1"
3. Driver launches (core state still normal) → Detect start → `TRC_LEARNING_RUNNING_RUN1` → Collect slip stats
4. Run ends → `TRC_LEARNING_COMPLETE_RUN1` → Prompt for Run 2
5. Repeat for Run 2 (`TRC_LEARNING_RUNNING_RUN2`)
6. Both runs done → `TRC_LEARNING_PROCESSING` → Calculate μ1/μ2 + best curve
7. `TRC_LEARNING_RESULTS_READY` → Broadcast `VCU_TRC_STATE` with results + store in runtime config
8. Driver exits learning → TRC_State returns to `TRC_ACTIVE` or `TRC_OFF`

The core `VCU_STATE` never needs to know about learning runs — it just sees normal torque requests with optional limiting applied by the TRC layer.

---

## 5. Detailed Implementation Plan (Step-by-Step for Agent)

### Step 1: Define New States & Data Structures (Layered Approach)

**Core state machine (`src/state_machine.h`)** — **Do NOT add learning states here.** Keep it focused on bare VCU functionality.

**New file: `src/control/traction_control_state_machine.h`** (create)

```c
typedef enum {
    TRC_STATE_OFF = 0,
    TRC_STATE_ACTIVE,              // Normal closed-loop traction (future PID)
    TRC_STATE_LEARNING_IDLE,
    TRC_STATE_LEARNING_READY_RUN1,
    TRC_STATE_LEARNING_RUNNING_RUN1,
    TRC_STATE_LEARNING_COMPLETE_RUN1,
    TRC_STATE_LEARNING_READY_RUN2,
    TRC_STATE_LEARNING_RUNNING_RUN2,
    TRC_STATE_LEARNING_PROCESSING,
    TRC_STATE_LEARNING_RESULTS_READY,
    TRC_STATE_LEARNING_ABORTED,
    TRC_STATE_FAULT
} TrcState_t;

typedef struct {
    TrcState_t current_state;
    // ... other TRC-specific data
} TrcStateMachine_Data_t;
```

**File:** `src/control/learning_mode.h` (new helper, called by TRC state machine)

```c
#ifndef LEARNING_MODE_H
#define LEARNING_MODE_H

#include <stdint.h>
#include <stdbool.h>
#include "IO_Types.h"   // or your ubyte/float types

typedef struct {
    bool     active;
    uint8_t  current_run;           // 0=none, 1=run1, 2=run2
    uint8_t  selected_curve;        // 0=A (conservative), 1=B (aggressive)
    float    peak_slip[2];
    float    avg_slip[2];
    uint32_t sample_count[2];
    float    estimated_mu[2];
    float    best_curve_score;      // or index
    uint32_t run_start_time[2];
    float    start_speed[2];
    float    end_speed[2];
    // Add more as needed (torque integral, etc.)
} LaunchLearning_Data_t;

extern LaunchLearning_Data_t g_learning_data;

void LearningMode_Init(void);
void LearningMode_Update(sbyte2 motor_rpm, sbyte2 requested_torque_nm);
void LearningMode_Reset(void);
bool LearningMode_IsActive(void);
uint8_t LearningMode_GetRecommendedCurve(void);
float LearningMode_GetEstimatedMu(uint8_t curve_idx);

#endif
```

### Step 2: Implement Core Logic in `learning_mode.c`

**Key Functions**

```c
#include "learning_mode.h"
#include "traction_control.h"      // reuse slip calc if possible
#include "can/can_rx.h"
#include "config/runtime_config.h"
#include "state_machine.h"         // for state queries if needed

static LaunchLearning_Data_t g_learning_data = {0};

void LearningMode_Init(void) {
    g_learning_data = (LaunchLearning_Data_t){0};
}

void LearningMode_Reset(void) {
    LearningMode_Init();
    // Optionally clear runtime params
}

bool LearningMode_IsActive(void) {
    return g_learning_data.active;
}

// Simple slip ratio (reuse/extend from traction_control.c)
static float CalculateSlipRatio(sbyte2 motor_rpm, float front_left_rpm, float front_right_rpm) {
    if (front_left_rpm < 1.0f && front_right_rpm < 1.0f) return 0.0f; // avoid div0 at standstill
    
    float v_front = (front_left_rpm + front_right_rpm) * 0.5f; // avg front wheel speed (rpm or scaled)
    // TODO: Convert motor_rpm → wheel-equivalent speed using gear ratio + tire radius
    float v_rear_equiv = motor_rpm * GetParamX1000(PARAM_GEAR_RATIO) * GetParamX1000(PARAM_TIRE_RADIUS_FACTOR);
    
    float slip = (v_rear_equiv - v_front) / v_front;
    return (slip > 0.0f) ? slip : 0.0f; // only positive drive slip
}

static bool IsHighTorquePhase(sbyte2 requested_torque) {
    return requested_torque > GetParam(PARAM_LEARNING_MIN_TORQUE_NM);
}

void LearningMode_Update(sbyte2 motor_rpm, sbyte2 requested_torque_nm) {
    if (!g_learning_data.active) return;

    const FrontWheelRpm_RX_Data_t* fl = CAN_RX_GetFrontLeftRpmData();
    const FrontWheelRpm_RX_Data_t* fr = CAN_RX_GetFrontRightRpmData();
    
    float slip = CalculateSlipRatio(motor_rpm, fl ? fl->rpm : 0, fr ? fr->rpm : 0);

    uint8_t run = g_learning_data.current_run - 1; // 0 or 1 index

    // Detect run start (simple threshold example)
    if (/* current state == RUNNING && */ motor_rpm > 500 && requested_torque_nm > 50) {
        if (g_learning_data.sample_count[run] == 0) {
            g_learning_data.run_start_time[run] = IO_RTC_GetTimeStamp();
            g_learning_data.start_speed[run] = /* current vehicle speed */;
        }
    }

    if (IsHighTorquePhase(requested_torque_nm) && g_learning_data.current_run > 0) {
        // Update running stats (memory efficient)
        if (slip > g_learning_data.peak_slip[run]) {
            g_learning_data.peak_slip[run] = slip;
        }
        // Running average
        uint32_t n = g_learning_data.sample_count[run];
        g_learning_data.avg_slip[run] = (g_learning_data.avg_slip[run] * n + slip) / (n + 1);
        g_learning_data.sample_count[run]++;
    }

    // TODO: Add run-end detection logic here or in state machine
    // (brake pressed, speed target reached, timeout)
}
```

**μ Estimation (Simple but Effective)**

```c
static float EstimateMu(uint8_t curve_idx) {
    // Placeholder — tune from real track data
    // Example: Higher peak slip + good accel = higher grip indication
    float peak = g_learning_data.peak_slip[curve_idx];
    float avg  = g_learning_data.avg_slip[curve_idx];
    
    // Very basic model (replace with better correlation to longitudinal accel or known curves)
    float grip_score = 1.0f / (1.0f + peak * 2.0f) + (avg < 0.15f ? 0.3f : 0.0f);
    return grip_score; // or map to real μ (0.6–1.2 range)
}

void LearningMode_ProcessResults(void) {
    g_learning_data.estimated_mu[0] = EstimateMu(0);
    g_learning_data.estimated_mu[1] = EstimateMu(1);
    
    // Simple winner selection
    if (g_learning_data.estimated_mu[1] > g_learning_data.estimated_mu[0]) {
        g_learning_data.best_curve_score = 1;
    } else {
        g_learning_data.best_curve_score = 0;
    }
    
    // Store in runtime config for persistence
    RuntimeConfig_SetI32(PARAM_LEARNING_MU_RUN1, (sbyte2)(g_learning_data.estimated_mu[0] * 1000));
    RuntimeConfig_SetI32(PARAM_LEARNING_MU_RUN2, (sbyte2)(g_learning_data.estimated_mu[1] * 1000));
    RuntimeConfig_SetI32(PARAM_LEARNING_BEST_CURVE, g_learning_data.best_curve_score);
}
```

### Step 3: Extend State Machine

In `state_machine.c`:

- Add transitions for learning states.
- On entering `READY_RUN1` / `READY_RUN2`: Send CAN status "Ready for Run X – Curve Y – Launch when ready".
- On `RUNNING_*`: Apply selected torque curve (or scale existing map).
- On run complete: Move to next state, call `LearningMode_ProcessResults()` when both done.
- Add abort path that resets everything safely.

### Step 4: Integrate with Torque Path

In `torque_controller.c` or `traction_control.c`:

```c
sbyte2 final_torque = requested;

if (LearningMode_IsActive()) {
    // During learning: apply test curve or light traction limiting only
    if (/* in running state */) {
        final_torque = ApplyLearningCurve(requested, g_learning_data.selected_curve);
    }
} else {
    final_torque = TractionControl_ApplyLimit(requested, motor_rpm);
}
```

### Step 5: CAN Messages (Recommended Layered Approach)

**Keep existing:**
- `VCU_STATE` — core vehicle state (RTD, drive mode, faults, etc.)

**Add new dedicated message:**
- `VCU_TRC_STATE` (TX, ~10-20 Hz or on change) — contains the entire traction control layer status:
  - `trc_state` (enum: OFF / ACTIVE / LEARNING_IDLE / LEARNING_READY_RUN1 / ... / RESULTS_READY / FAULT)
  - `learning_active` (bool)
  - `current_run` (0/1/2)
  - `selected_curve` (0/1)
  - `prompt_id` or short status text
  - When in RESULTS_READY: embed or follow with results fields (`mu_run1_x1000`, `mu_run2_x1000`, `best_curve`, `recommended_slip_target_x1000`, etc.)

**Optional separate command message:**
- `VCU_TRC_Command` (RX) — for enabling/disabling TRC layer, entering learning, selecting curve, abort, etc.

This keeps `VCU_STATE` clean while giving the traction layer its own independent, queryable state that the dash, telemetry, and service tool can consume without parsing core vehicle logic.

### Step 6: Safety & Abort Logic

- Central abort function that:
  - Sets torque to 0 or safe limp
  - Resets learning data
  - Returns state machine to normal drive state
  - Logs event
- Check brake pedal (existing BSE sensor) in every learning state.
- Timeout per run (configurable, e.g., 15–20 s).
- Max slip guard.

### Step 7: Dash / Driver Feedback

Send clear status messages via CAN:
- "LEARNING MODE ACTIVE"
- "RUN 1 READY – CURVE A – GO"
- "RUN 1 COMPLETE – Data OK"
- "RUN 2 READY – CURVE B – GO"
- "PROCESSING..."
- "RESULTS: μ1=0.81 μ2=0.87 → Use Curve B, target slip 0.12"

Use existing lights/buzzer for critical states if needed.

---

## 6. New / Modified Files Summary (Layered Design)

| File | Action | Purpose |
|------|--------|---------|
| `src/control/traction_control_state_machine.c` | **Create** | New independent TRC state machine (owns TRC states + calls learning) |
| `src/control/traction_control_state_machine.h` | **Create** | `TrcState_t` enum + TRC state machine API |
| `src/control/learning_mode.c` | **Create** | Helper: slip stats, running averages, μ estimation, result processing |
| `src/control/learning_mode.h` | **Create** | Public API for the learning helper |
| `src/state_machine.h` | **Do NOT modify** (or only minor) | Keep focused on core `VCU_STATE` |
| `src/state_machine.c` | **Minimal or no change** | Core vehicle states only |
| `src/control/traction_control.c` | **Modify / refactor** | Move existing PID logic under TRC state machine control |
| `src/control/torque_controller.c` | **Modify** (light) | Check TRC state before applying limits or learning curves |
| `VCU.dbc` | **Modify** | Add `VCU_TRC_STATE` message (and optional `VCU_TRC_Command`) |
| `src/config/runtime_config.h` | **Modify** | Add `PARAM_TRC_*` and `PARAM_LEARNING_*` tunables |
| `src/main.c` | **Modify** (small) | Call both `CoreStateMachine_Update()` and `TractionControl_StateMachine_Update()` every loop |

---

## 7. Testing Strategy

**Bench / SIL**
- Use existing CAN simulation or script to inject wheel speed + motor RPM messages.
- Verify state transitions, stats accumulation, result calculation.
- Test abort paths and safety interlocks.

**On-Car (Low Risk)**
1. Start with **very conservative** torque curves and low power limit.
2. Perform learning sequence on known flat surface.
3. Verify dash prompts, CANedge2 logging trigger, and final results broadcast.
4. Compare VCU-computed μ against post-process of detailed CANedge2 logs (GNSS + IMU + wheel speeds).
5. Validate no unintended torque intervention in normal drive mode.

**Metrics to Log**
- Slip traces for both runs
- Peak/avg slip per run
- Estimated μ values
- Time from launch to end speed
- Any aborts or faults

---

## 8. Future Work (Closed-Loop PID)

Once learning is solid:
- Use learned μ or best slip target as input to adaptive PID in `traction_control.c`.
- Gain scheduling based on estimated grip.
- Torque vectoring or curve-specific maps.
- Auto-update target slip from recent learning runs.

---

## 9. Implementation Notes & Style

- Follow existing code style (naming, types like `sbyte2`, `float4`, `ubyte*`, `GetParam*` helpers).
- Prefer **running statistics** over large arrays to conserve RAM.
- Comment all magic numbers and reference runtime params.
- Use `IO_RTC_GetTimeStamp()` for timing.
- Keep functions small and testable.
- Add asserts or bounds checks in debug builds.

---

## 10. Open Questions for Clarification (if needed)

- Exact definition of the two torque curves (maps vs simple scaling factors)?
- Preferred μ estimation formula once you have real data?
- Dash display capabilities (text strings vs numeric IDs)?
- Any existing longitudinal accel source (IMU via CANedge2 or separate sensor)?

---

**This specification is ready for an AI coding agent.**  
Copy the entire content above into your agent prompt along with relevant snippets from your current `traction_control.c`, `state_machine.h`, and `VCU.dbc` if you want the agent to generate production-ready code diffs.

You can now hand this file directly to your agent (or future self) and say:  
**"Implement according to the detailed spec in Launch_Control_Learning_Mode_Spec.md. Start by creating the new learning_mode module and extending the state machine."**

Good luck with the implementation — this will be a powerful tool for the team! 

**File saved to:** `/home/workdir/artifacts/Launch_Control_Learning_Mode_Spec.md`