# Launch Control System Audit
**Date:** 2026-06-10
**Branch:** traction_control
**Auditor:** GitHub Copilot

---

## 1. System Overview

The launch control system implements a **time-based torque curve** for standing-start acceleration. Since the car no longer has front wheel-speed sensors, launch torque follows a fixed, preprogrammed torque-vs-time profile instead of slip-based control.

### State Machine

```
DISARMED ──[CAN ARM cmd]──► ARMED ──[off-brakes + APPS > 25%]──► LAUNCHING
    ▲                                                               │
    └───────────── [brake / APPS < 5% / !DRIVING] ─────────────────┘
```

### Torque Curve

| Phase | Duration | Torque |
|-------|----------|--------|
| Off-the-line hold | 0 – 200 ms | `torque_offtheline` (100 Nm) |
| Parabolic ramp | 200 ms – 3.0 s | `torque_init + t² × (torque_final − torque_init) / duration²` |
| Flat hold | > 3.0 s | `torque_final` (183 Nm) |

Output is always **capped by the driver's pedal-mapped torque**, preserving APPS / FSAE plausibility.

### Integration Points

| Component | Interaction |
|-----------|-------------|
| **main.c** | Calls `LaunchControl_Update()` before `TorqueController_Update()` |
| **torque_controller.c** | Calls `LaunchControl_GetTorque()` – launch overrides pedal torque when active |
| **can_rx.c** | Receives ARM/DISARM commands via `SET_VCU_CONFIG` mux 240 |
| **can_tx.c** | Broadcasts launch state, elapsed time, curve torque for telemetry |
| **runtime_config.c** | All 7 launch params are EEPROM-backed with min/max clamping |
| **traction_control.c** | Telemetry refreshed during launch but **torque limit is NOT applied** (by design) |

---

## 2. Findings

### FINDING 1 — Torque Discontinuity at Off-the-Line Boundary
**Severity:** Medium  
**Location:** `launch_control.c:42-50` (`ComputeCurveTorque`)

At the transition from the off-the-line hold to the parabolic ramp, torque jumps in a single 10 ms cycle from `torque_offtheline` (100 Nm) to approximately `torque_init` (150 Nm) — a **~50 Nm step**.

```
t = 0.199 s → 100 Nm  (offtheline hold)
t = 0.200 s → ~150 Nm (parabolic curve start)
```

This is because the parabolic formula uses **absolute elapsed time** `t_s`, not time relative to the off-the-line window. At `t = offtheline_s`, the curve evaluates to:

$$T = T_{\text{init}} + \frac{t_{\text{offtheline}}^2 \cdot (T_{\text{final}} - T_{\text{init}})}{t_{\text{duration}}^2}$$

For default values this is ~150.15 Nm (negligible difference from `torque_init`), but the **jump from 100 → 150 Nm is real**. On a fast-responding EV motor this could cause a noticeable jerk at the 200 ms mark.

**Suggestion:** If a smoother transition is desired, either:
- Use `t_curve = t_s - offtheline_s` so the parabola starts exactly at `torque_init` with zero slope, or
- Add a short ramp (e.g., 50 ms) from `torque_offtheline` to `torque_init` at the boundary.

---

### FINDING 2 — No Traction Control During Launch
**Severity:** Medium (by design, but worth documenting)  
**Location:** `torque_controller.c:503-508`

When launch is active, `TractionControl_ApplyLimit()` is called with the launch torque but its **return value is discarded**:

```c
(void)TractionControl_ApplyLimit(launch_torque, inv_data->motor_speed);
limited_torque = launch_torque;  // TC output ignored
```

This means **there is zero anti-spin protection during launch** — the most wheel-spin-prone phase of a run. The comment in the code explains this is intentional ("so the limiter does not fight the launch curve"), and it makes sense given the lack of front wheel sensors.

**Suggestion:** If front sensors are ever restored, or if a rear-only slip detection strategy is viable, consider re-enabling TC during launch. For now, document this as a known limitation in the driver briefing.

---

### FINDING 3 — No Validation: `torque_init` vs `torque_final`
**Severity:** Low  
**Location:** `runtime_config.c:304-323`

The runtime config allows `launch_torque_init` (0–230) and `launch_torque_final` (0–230) to be set independently. There is **no cross-parameter validation** ensuring `torque_init ≤ torque_final`. If `init > final`, the parabolic curve **decreases** over time — mathematically valid but almost certainly a misconfiguration.

Similarly, there is no validation that `offtheline_time_ms ≤ curve_duration_ms`. If off-the-line time exceeds the curve duration, the ramp is skipped entirely and torque jumps directly from `torque_offtheline` to `torque_final`.

**Suggestion:** Add a post-set sanity check (or just document the expected relationship in the DBC/comments).

---

### FINDING 4 — No Maximum Launch Duration Safety Limit
**Severity:** Low  
**Location:** `launch_control.c:137-156` (`LaunchControl_Update`, `LAUNCH_STATE_LAUNCHING`)

There is no watchdog or maximum duration limit on an active launch. The launch stays active until the driver brakes, lifts below 5% APPS, or the car leaves DRIVING. In practice this is fine (the curve flattens at `torque_final` after 3 s), but an indefinitely long launch could thermally stress the motor/inverter.

**Suggestion:** Consider adding a configurable maximum launch duration (e.g., 10 s) that auto-disarms as a safety backstop.

---

### FINDING 5 — `LaunchControl_Init()` Not Called Directly in `main.c`
**Severity:** Low (maintainability)  
**Location:** `main.c` vs `torque_controller.c:460`

Every other module (`APPS_Init`, `BSE_Init`, `StateMachine_Init`, etc.) is initialized explicitly in `main.c`. `LaunchControl_Init()` is only called **indirectly** from inside `TorqueController_Init()`. This is a hidden dependency — if someone reorders or removes `TorqueController_Init()`, launch control silently fails to initialize.

**Suggestion:** Add an explicit `LaunchControl_Init()` call in `main.c` alongside the other module inits, and remove the call from `TorqueController_Init()`.

---

### FINDING 6 — Documentation Drift in Header Comment
**Severity:** Informational  
**Location:** `launch_control.h:12-16`

The header comment references `LAUNCH_OFFTHELINE_TIME_S` (seconds) but the actual define is `LAUNCH_OFFTHELINE_TIME_MS_DEFAULT` (milliseconds). The code uses the correct millisecond define. Also, the comment says "curve value at t = 0 s" for `torque_init`, but due to Finding 1 the curve actually evaluates at `t = offtheline_s`, not `t = 0`.

---

## 3. Safety Analysis

| Safety Property | Status | Notes |
|----------------|--------|-------|
| **FSAE APPS plausibility preserved** | ✅ Pass | Curve torque is always pedal-bounded in `GetTorque()` |
| **Brake-throttle cut honored** | ✅ Pass | Checked before `LaunchControl_GetTorque()` in torque controller |
| **APPS validity required** | ✅ Pass | `apps_ok` checked at trigger and during launch |
| **Brake ends launch** | ✅ Pass | `brakes_on` checked every cycle |
| **State loss ends launch** | ✅ Pass | `!driving` transitions to DISARMED |
| **Cannot re-arm mid-launch** | ✅ Pass | ARM command only accepted from DISARMED state |
| **CAN command validation** | ✅ Pass | Invalid command values hit the `default` no-op case |
| **Runtime param bounds** | ✅ Pass | All 7 launch params have min/max clamping in `runtime_config.c` |
| **Division by zero protection** | ✅ Pass | `duration_s` minimum is 100 ms (protected by runtime config) |
| **Null pointer check** | ✅ Pass | `GetTorque()` validates `out_torque != NULL` |
| **Overflow protection** | ✅ Pass | Curve torque clamped to `[0, max_torque]`; `elapsed_ms` saturated to 65535 in CAN TX |
| **Traction control during launch** | ⚠️ Known gap | See Finding 2 — by design, no TC during launch |
| **Maximum launch duration** | ⚠️ No limit | See Finding 4 |

---

## 4. Code Quality

| Aspect | Assessment |
|--------|-----------|
| **State machine clarity** | ✅ Clean, well-documented, easy to follow |
| **Separation of concerns** | ✅ Launch logic is isolated in its own module |
| **Runtime tunability** | ✅ All key parameters are EEPROM-backed and CAN-settable |
| **Telemetry** | ✅ State, armed, active, elapsed_ms, curve_torque all broadcast over CAN |
| **Float math** | ✅ All casts are safe for the parameter ranges; proper rounding (+0.5f) |
| **Consistent patterns** | ✅ Follows same `GetParam()` / `GetTorque()` / `GetData()` patterns as other modules |

---

## 5. Summary

The launch control system is **well-designed and safe for competition use**. The core logic is clean, the safety invariants (pedal bounding, brake cut, APPS validity, state transitions) are all correctly enforced, and the runtime configurability is thorough.

The most actionable finding is the **torque discontinuity at the off-the-line boundary** (Finding 1) — a ~50 Nm step in a single cycle that could cause a drivetrain jerk. The remaining findings are low-severity design considerations or documentation cleanup.

### Recommended Actions
1. **Review the offtheline → ramp transition** (Finding 1) — decide if the ~50 Nm step is acceptable or if a smoother handoff is needed
2. **Add `LaunchControl_Init()` to `main.c`** (Finding 5) — quick fix, improves maintainability
3. **Document the no-TC-during-launch limitation** (Finding 2) — ensure the team is aware
4. **Update header comment** (Finding 6) — minor doc fix
