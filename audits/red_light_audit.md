# Red Light (Red Car) Behavior Audit

**Audited:** 2026-06-12  
**Branch:** traction_control  
**Primary source files:** [`src/io/lights.c`](../src/io/lights.c), [`src/state_machine.c`](../src/state_machine.c), [`src/can/can_rx.c`](../src/can/can_rx.c)

---

## Overview

"Red car" is the FSAE term for the TSSI (Tractive System Status Indicator) showing red — indicating an active high-voltage fault. The VCU tracks this with a single latching boolean `red_car` in `lights.c`. The physical outputs are:

| Signal | Pin | Behavior |
|--------|-----|----------|
| `TSSI_RED_PIN` | `IO_DO_00` | Blinks 2 Hz when `red_car == TRUE`, off otherwise |
| `TSSI_GREEN_PIN` | `IO_DO_01` | Solid on when `red_car == FALSE`, off otherwise |

Green and red are always logical complements for the car-state indicator — green is not a separate indicator with its own logic.

---

## Fault Triggers

The red car flag latches `TRUE` when **any** of the following is detected in `Lights_Update()` ([`lights.c:53`](../src/io/lights.c#L53)):

```c
else if (!hvc_valid || !hvc->imd_ok || !hvc->bms_ok) {
    red_car = TRUE;
}
```

### 1. IMD Fault (`imd_ok == FALSE`)

- Source: `HVC_SUMMARY` CAN message, CAN ID `0x004001F0`, extended frame on `CONTROLS_CAN_CHANNEL`
- Bit encoding: byte 0, bit 1 — **active-low** (bit set = fault, `imd_ok = (data[0] & 0x02) == 0`)
- The IMD hardware opens the shutdown circuit independently; this flag is the VCU's CAN-reported view of that event

### 2. BMS Fault (`bms_ok == FALSE`)

- Source: same `HVC_SUMMARY` message
- Bit encoding: byte 0, bit 2 — **active-low** (`bms_ok = (data[0] & 0x04) == 0`)
- Same pattern as IMD: BMS opens the SDC in hardware; this is the CAN-reported status

### 3. Lost HVC Communication (`hvc_valid == FALSE`)

- Triggered when no `HVC_SUMMARY` frame has been received within **5000 ms** (`MSG_TIMEOUT_US` in [`can_config.h:22`](../src/config/can_config.h#L22))
- Implemented in `CAN_Manager_ProcessRxMessages()` ([`can_manager.c:574`](../src/can/can_manager.c#L574)):
  ```c
  if (IO_RTC_GetTimeUS(msg->last_rx_timestamp) > msg->timeout_us) {
      msg->data_vld = FALSE;
  }
  ```
- Also triggers if the CAN bus itself is in recovery (`!CAN_Recovery_IsReady()`), which forces **all** message validity flags false simultaneously

### Bit Encoding Detail

The raw `io_summary_flags` byte from the HVC uses active-low logic. A `1` in a bit position means the corresponding device is **faulted**:

| Bit | Mask | Signal | `ok` when |
|-----|------|--------|-----------|
| 0 | `0x01` | SDC | bit is `0` |
| 1 | `0x02` | IMD | bit is `0` |
| 2 | `0x04` | BMS | bit is `0` |

---

## Grace Periods and Timers

### Startup Grace Period — 5000 ms

- Constant: `RED_CAR_STARTUP_GRACE_US = MsToUs(5000)` ([`dio_config.h:19`](../src/config/dio_config.h#L19))
- Timer started in `Lights_Init()` ([`lights.c:25`](../src/io/lights.c#L25))
- Checked each cycle: `in_red_car_grace = (IO_RTC_GetTimeUS(red_car_grace_start) < 5000ms)`
- **Effect:** During the first 5 seconds after VCU boot, `red_car` is **force-cleared** every cycle, regardless of any fault signals. Any latch that forms is immediately cleared the next cycle as long as the grace is active.
- **Interaction with HVC timeout:** The HVC message timeout (`MSG_TIMEOUT_US`) is also 5000 ms, and its countdown starts from the same boot moment (`CAN_Manager_Init()` initializes `last_rx_timestamp` for all messages). This means:
  - If HVC starts sending within 5 s, its timeout won't fire after grace expires.
  - If HVC has sent **nothing** by t = 5 s, both timers expire simultaneously — the grace ends and `hvc_valid` goes false in the same cycle, immediately latching red car.

### Blink Period — 250 ms

- Constant: `TSSI_BLINK_PERIOD_US = MsToUs(250)` ([`dio_config.h:18`](../src/config/dio_config.h#L18))
- Purely cosmetic — controls the LED toggle rate, not the fault logic
- The `red_light_state` toggle happens on `IO_RTC_GetTimeUS(last_blink) > 250ms`
- When `red_car` goes `FALSE`, `red_light_state` is immediately forced `FALSE` (LED off, not mid-blink)

---

## Latching Behavior

The `red_car` flag is **fully latching** — it does not clear when the fault clears. The full priority chain in `Lights_Update()` each 10 ms cycle:

```
Priority 1 (highest): always_green || in_red_car_grace  → force red_car = FALSE
Priority 2:           !hvc_valid || !imd_ok || !bms_ok  → latch red_car = TRUE
Priority 3:           hvc->sdc_ok                        → clear red_car = FALSE
Priority 4 (default): none of the above                 → hold current red_car value
```

### What "latching" means in practice

- Once a BMS or IMD fault is seen, `red_car` becomes `TRUE`.
- If the fault flag clears (e.g. a transient glitch), `red_car` does **not** clear automatically. The code falls to Priority 3: it only clears if `sdc_ok` is simultaneously `TRUE`.
- **Why:** When a real BMS/IMD fault fires, the SDC opens — so `sdc_ok` is `FALSE`. The fault flag and SDC-open state arrive together. As the fault clears (or after a reset), the SDC must be explicitly re-closed before the VCU will let the red car condition go.
- The holding case (Priority 4) catches the window where faults have cleared but the SDC has not yet re-confirmed closed. The red car state is held until the SDC handshake completes.

### Clear Conditions

| Condition | Mechanism | Notes |
|-----------|-----------|-------|
| SDC confirmed OK | `hvc->sdc_ok == TRUE` with no active fault | Normal post-fault recovery path |
| Debug `ALWAYS_GREEN` bit set | Bit 9 of `RUNTIME_PARAM_DEBUG_DEFINES` | Force-clears every cycle; overrides all faults |
| Startup grace active | First 5000 ms after boot | Force-clears every cycle |

There is **no power-cycle-only latch** — clearing the fault source and getting SDC re-closed is sufficient to clear the red car state at runtime.

---

## Debug Overrides

Both overrides live in `RUNTIME_PARAM_DEBUG_DEFINES` (an i16 bitmask settable over CAN via `SET_VCU_CONFIG`).

### `DEBUG_BIT_ALWAYS_GREEN` (bit 9, `1 << 9`)

- Read in `Lights_Update()` ([`lights.c:47`](../src/io/lights.c#L47))
- Forces `red_car = FALSE` every cycle (Priority 1, alongside startup grace)
- Also reaches into `CAN_RX_GetHVCSummaryData()` and overrides `imd_ok = TRUE`, `bms_ok = TRUE` on the effective copy returned to all consumers ([`can_rx.c:57–60`](../src/can/can_rx.c#L57))
- **Net effect:** Any code consuming `HVC_SUMMARY` through `CAN_RX_GetHVCSummaryData()` sees no faults, and Lights sees no faults either. Double-layered override.

### `DEBUG_BIT_IGNORE_SDC` (bit 7, `1 << 7`)

- Only applies inside `CAN_RX_GetHVCSummaryData()` ([`can_rx.c:52–54`](../src/can/can_rx.c#L52))
- Forces `sdc_ok = TRUE` on the effective data copy
- Does **not** force `imd_ok` or `bms_ok` — faults can still latch red car
- Effect: the clear condition (Priority 3) is always satisfied, so once the fault flag itself clears, red car will immediately clear too — the SDC "must re-close" hold is bypassed

---

## State Machine Integration

`red_car` is queried by the state machine via `Lights_isRedCar()` ([`lights.h`](../src/io/lights.h)).

### Hard Fault Definition

```c
// state_machine.c:48
const bool hard_fault = (!apps->valid || !bse->valid || !hvc_summary->sdc_ok || is_red_car);
```

`is_red_car` is one of four conditions that constitute a `hard_fault`. A hard fault blocks the `NOT_READY → DRIVING` transition and immediately kicks the car out of `PLAYING_RTD_SOUND` or `DRIVING` states into `VCU_STATE_HARD_FAULT`.

### DEAD_CAR CAN Message

On the **rising edge** of `is_red_car` (transition from not-red to red), `dead_car_tx_pending` is set ([`state_machine.c:60–62`](../src/state_machine.c#L60)):

```c
if (!was_red_car && is_red_car) {
    dead_car_tx_pending = TRUE;
}
```

This sends a single `DEAD_CAR` CAN frame (ID `0x0D10DEAD`) on the DAQ channel. It is non-periodic — sent once per fault transition, not continuously.

Additionally, when the state machine itself transitions to `VCU_STATE_HARD_FAULT` from `DRIVING` or `PLAYING_RTD_SOUND`, it also sets `dead_car_tx_pending = TRUE` independently ([`state_machine.c:99, 86`](../src/state_machine.c#L99)). These two paths can both fire for the same fault event, but `StateMachine_DeadCarTxTrigger()` is a consume-on-read flag, so the second set just means the message is sent once.

### State Transitions Involving Red Car

```
VCU_STATE_NOT_READY
  └─ ready_to_drive && !hard_fault && !bap_fault  →  VCU_STATE_PLAYING_RTD_SOUND
     (red car blocks this transition)

VCU_STATE_PLAYING_RTD_SOUND
  └─ hard_fault  →  VCU_STATE_HARD_FAULT

VCU_STATE_DRIVING
  └─ hard_fault  →  VCU_STATE_HARD_FAULT

VCU_STATE_HARD_FAULT
  └─ !rtd_active  →  VCU_STATE_NOT_READY
     (RTD switch release is the only exit; red car alone does NOT hold this state)
```

**Important:** `VCU_STATE_HARD_FAULT` exits when `rtd_active == FALSE` — not when `red_car` clears. This means a driver must release and re-press the RTD switch after a fault clears before the car can go ready-to-drive again. If the RTD switch stays engaged while the fault clears, the state machine loops:

```
HARD_FAULT (rtd held) → stays HARD_FAULT (even if red_car clears)
HARD_FAULT (rtd released) → NOT_READY → can attempt PLAYING_RTD_SOUND
```

---

## Execution Order

`Lights_Update()` runs **before** `StateMachine_Update()` every cycle ([`main.c:103–104`](../src/main.c#L103)):

```c
Lights_Update();       // red_car latching resolved here
StateMachine_Update(); // reads Lights_isRedCar() — sees current cycle's result
```

This means the state machine always operates on the freshest red car determination, with zero cycle lag.

The CAN RX processing (`CAN_Manager_ProcessRxMessages()`) also runs before both, so HVC message validity is current when Lights evaluates it.

---

## End-to-End Fault Scenario Walkthrough

### Scenario A: BMS Fault During Driving

1. HVC detects BMS fault → opens SDC → sets bit 2 in `HVC_SUMMARY` byte 0
2. VCU receives frame → `bms_ok = FALSE`, `sdc_ok = FALSE` (SDC opened)
3. `Lights_Update()`: not in grace, `bms_ok == FALSE` → **`red_car = TRUE`** (Priority 2)
4. TSSI red light begins blinking at 2 Hz
5. `StateMachine_Update()`: `hard_fault = TRUE` → `current_state = VCU_STATE_HARD_FAULT`, `dead_car_tx_pending = TRUE`
6. `CAN_Manager_ProcessTxMessages()`: DEAD_CAR frame sent once on DAQ bus

**Recovery:**
7. BMS fault clears → HVC closes SDC → `bms_ok = TRUE`, `sdc_ok = TRUE`
8. `Lights_Update()`: not in grace, `bms_ok` OK — falls to Priority 3: `sdc_ok == TRUE` → **`red_car = FALSE`**
9. TSSI green resumes
10. `StateMachine_Update()`: `hard_fault = FALSE` — but state is still `HARD_FAULT`
11. Driver releases RTD → `rtd_active = FALSE` → state transitions to `NOT_READY`
12. Driver re-presses RTD + holds brake → state machine can proceed to `PLAYING_RTD_SOUND`

### Scenario B: Lost HVC Communication

1. HVC CAN goes silent (cable pulled, HVC powered off, etc.)
2. After **5000 ms** without an `HVC_SUMMARY` frame: `data_vld = FALSE`
3. `Lights_Update()`: `!hvc_valid` → **`red_car = TRUE`** (Priority 2)
4. Same state machine consequences as Scenario A
5. Recovery: HVC resumes sending → `data_vld` flips `TRUE` on next received frame
6. If `imd_ok` and `bms_ok` are OK and `sdc_ok` is TRUE in that frame → `red_car` clears same cycle

### Scenario C: Boot Sequence

1. `Lights_Init()` called — starts `red_car_grace_start` and `last_rx_timestamp` for all CAN messages simultaneously
2. For the first 5000 ms: `in_red_car_grace = TRUE` → `red_car = FALSE` forced every cycle
3. If HVC is already sending: `hvc_valid` becomes `TRUE` during the grace window; no red car after grace expires
4. If HVC is **not yet** sending at t = 5000 ms: grace expires and HVC timeout expires in the same cycle → red car immediately latches

---

## Known Quirks and Design Notes

1. **Grace = HVC timeout duration.** Both are 5000 ms and start at the same moment. The grace period precisely covers the window where the HVC might not yet have sent its first frame. There is no "gap" between grace ending and the HVC timeout window — they are designed to align.

2. **Fault-flag vs. SDC gap.** The latching logic correctly handles a timing edge: if an IMD/BMS fault fires and clears very quickly (sub-cycle transient), the code could reach Priority 3 (`sdc_ok` check) before latching. However, a real hardware fault opens the SDC — so `sdc_ok` will also be `FALSE`, and Priority 3 will not clear the latch. Only a truly instantaneous spurious glitch that never propagated to the SDC would be transparent.

3. **`red_car` is not a VCU state.** The comment in the code notes this: `was_red_car` is a separate tracking variable precisely because red car doesn't map to a `VCU_State_t` enum value. A future refactor might integrate it as a state.

4. **Green output is the complement of `red_car`, not of `red_light_state`.** Green turns off immediately when `red_car` latches — it does not wait for the blink timer. The green-off / red-starts-blinking transition is instantaneous from the car state perspective.

5. **CAN bus recovery invalidates everything.** `!CAN_Recovery_IsReady()` forces `data_vld = FALSE` for **all** RX messages, not just HVC. So a CAN bus issue (bus-off, etc.) triggers the same `!hvc_valid` path as a lost HVC and causes red car.

6. **`ALWAYS_GREEN` operates at two layers.** It patches the effective HVC summary struct returned by `CAN_RX_GetHVCSummaryData()` in addition to directly suppressing the red car latch in Lights. Any other module that reads `imd_ok`/`bms_ok` also sees the overridden values — there is no path where fault flags leak past this debug bit.
