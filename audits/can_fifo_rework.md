# CAN FIFO Rework Notes

## Purpose

This note documents the CAN FIFO problems found while adding regen-related receive messages, SOC receive support, and the `VCU_CAN_Readback` debug transmit message. It explains what was wrong with the previous FIFO allocation strategy, what the current system does differently, and why the new design should not compromise critical VCU behavior such as inverter torque command transmission.

## Short Version

The TTC60 CAN driver appears to have a limited number of hardware FIFO/message-object handles. The old CAN manager allocated one receive FIFO per logical RX message, then allocated TX FIFOs after all RX FIFOs. After adding more RX messages, the firmware could exceed the available hardware handles. The most visible symptom was that Controls standard TX still worked, but Controls extended TX and DAQ extended TX disappeared or became unreliable.

The current design reserves all TX FIFOs first, then reduces RX handle usage by sharing one hardware RX FIFO for related non-critical Controls extended telemetry messages. Critical or timing-sensitive paths keep dedicated FIFOs.

Current handle usage is:

| Handle type | Count | Purpose |
| --- | ---: | --- |
| TX FIFO | 3 | Controls STD TX, Controls EXT TX, DAQ EXT TX |
| RX FIFO | 6 | Inverter status, inverter high speed, shared Controls telemetry, SET VCU config, front left RPM, front right RPM |
| Total | 9 | Leaves margin under the observed TTC60 limit of about 10 handles |

## What Was Wrong With The Old System

The old system treated every logical RX message as needing its own hardware FIFO:

- `INV_STATUS`
- `INV_HIGH_SPEED`
- `HVC_SUMMARY`
- `HVC_SOC`
- `HVC_VSENSE`
- `MOBO_POWER_TELEMETRY`
- `SET_VCU_CONFIG`
- `FRONT_LEFT_RPM`
- `FRONT_RIGHT_RPM`

That is 9 RX FIFOs. The system also needs 3 TX FIFOs:

- Controls STD TX
- Controls EXT TX
- DAQ EXT TX

That makes 12 hardware FIFO/message-object handles. The TTC60 target did not behave correctly with that many. The driver calls did not make the failure obvious in normal behavior because `IO_CAN_ConfigFIFO` return values are not currently checked during initialization.

The failure pattern matched handle exhaustion:

- Controls standard TX messages still appeared, such as inverter torque command `0x0C0` and current limit `0x202`.
- Extended TX messages disappeared, including DAQ traffic and Controls extended config readback.
- Disabling the new `VCU_CAN_Readback` transmit message did not fully solve the problem, because the underlying issue was handle count, not only bus load.

The old allocation order also made the failure more dangerous for observability. RX FIFOs were allocated before TX FIFOs, so adding receive messages could starve later TX FIFO allocation.

## Current FIFO Allocation Strategy

The new system has two important rules:

1. Configure TX FIFOs first.
2. Share RX FIFOs only where message criticality and traffic patterns make that acceptable.

TX FIFOs are configured first in this order:

| FIFO | Channel | Frame type | Purpose |
| --- | --- | --- | --- |
| `controls_tx_std_fifo_handle` | Controls | Standard | Inverter command, inverter read/write, inverter current limit |
| `controls_tx_ext_fifo_handle` | Controls | Extended | VCU config readback and any Controls extended TX |
| `daq_tx_ext_fifo_handle` | DAQ | Extended | DAQ telemetry, CAN health, traction control debug, readback |

Then RX FIFOs are configured for only the messages that own hardware FIFOs:

| Logical RX message | Hardware FIFO owner | Filter type | Reason |
| --- | --- | --- | --- |
| `CAN_RX_MSG_INV_STATUS` | Itself | Exact STD ID | Dedicated inverter status path |
| `CAN_RX_MSG_INV_HIGH_SPEED` | Itself | Exact STD ID | Dedicated inverter feedback path used by torque/speed logic |
| `CAN_RX_MSG_HVC_SUMMARY` | Shared telemetry FIFO | Masked Controls EXT ID | Owns HVC/MOBO telemetry FIFO |
| `CAN_RX_MSG_HVC_SOC` | `HVC_SUMMARY` FIFO | Software-dispatched by ID | Saves a hardware FIFO |
| `CAN_RX_MSG_HVC_VSENSE` | `HVC_SUMMARY` FIFO | Software-dispatched by ID | Saves a hardware FIFO |
| `CAN_RX_MSG_MOBO_POWER_TELEMETRY` | `HVC_SUMMARY` FIFO | Software-dispatched by ID | Saves a hardware FIFO |
| `CAN_RX_MSG_SET_VCU_CONFIG` | Itself | Exact Controls EXT ID | Dedicated config command path |
| `CAN_RX_MSG_FRONT_LEFT_RPM` | Itself | Exact DAQ EXT ID | Dedicated wheel speed input |
| `CAN_RX_MSG_FRONT_RIGHT_RPM` | Itself | Exact DAQ EXT ID | Dedicated wheel speed input |

The shared Controls telemetry FIFO uses mask `0x1F9FFE1F`. It is configured using `CAN_ID_HVC_SUMMARY` as the base ID and is intended to accept these known Controls extended telemetry messages:

- `CAN_ID_HVC_SUMMARY` / `0x004001F0`
- `CAN_ID_HVC_SOC` / `0x004001F4`
- `CAN_ID_HVC_VSENSE` / `0x004001F7`
- `CAN_ID_MOBO_POWER_TELEMETRY` / `0x00200010`

After a frame is read from a shared FIFO, the software dispatcher checks the frame ID and calls the matching decode function. Unknown IDs read through a shared FIFO are ignored.

Exact-match FIFOs do not do software ID redispatch. They call their configured decoder directly. This matters because the CAN driver may not populate every metadata field, such as `id_format`, on reads. For exact hardware filters, the hardware already did the match, so a second software metadata check can silently drop valid frames.

## Why `SET_VCU_CONFIG` Got Its Own FIFO Again

One intermediate fix put all Controls extended RX messages into a single broad FIFO. That restored config writes, but made single-message config sets inconsistent because the config command shared a queue with HVC/MOBO telemetry. If telemetry traffic filled or churned the queue, a single config set frame could be delayed or dropped. Sending the set frame repeatedly made it more likely that one would survive.

The current design fixes that by giving `SET_VCU_CONFIG` an exact dedicated FIFO again. This keeps one-shot config messages from competing with telemetry traffic, while still saving handles by sharing HVC/MOBO telemetry.

## Impact On Critical VCU Functionality

### Inverter Command TX

The inverter torque command path is protected by the new allocation order. `CAN_TX_MSG_INV_TORQUE_COMMAND` uses Controls standard TX ID `0x0C0` and the `controls_tx_std_fifo_handle`. That FIFO is configured first, before any RX FIFO. The current RX sharing strategy does not touch this TX path.

The other Controls standard inverter-related TX messages also use the same early-reserved Controls STD TX FIFO:

- `CAN_TX_MSG_INV_READ_WRITE` on `0x0C1`
- `CAN_TX_MSG_INV_CURRENT_LIMIT` on `0x202`

So the most critical outbound Controls standard messages are not dependent on the shared telemetry RX FIFO and are not starved by added receive messages.

### Inverter Feedback RX

The inverter receive paths still have dedicated exact RX FIFOs:

- `CAN_RX_MSG_INV_STATUS`
- `CAN_RX_MSG_INV_HIGH_SPEED`

These are not part of the shared Controls telemetry FIFO. `INV_HIGH_SPEED` feeds torque feedback, motor speed, and DC bus voltage decoding. Keeping it exact and dedicated avoids extra dispatch latency and avoids competition with HVC/MOBO telemetry.

### Config Command RX

`SET_VCU_CONFIG` now has an exact dedicated Controls extended RX FIFO. This is important for reliability because config commands may be sent as a single frame and should not be forced to compete with periodic telemetry in the same receive queue.

Config commands are not part of the hard real-time inverter torque command loop, but they affect runtime behavior. Giving them a dedicated FIFO restores the original expectation that one valid config frame should be enough.

### HVC, SOC, VSense, And Rear Brake Pressure

These messages share a Controls extended telemetry FIFO. They are important, but they are not sent to the inverter as direct commands. They are decoded into cached state and protected by validity timeouts. If one of these messages stops arriving, its `data_vld` flag eventually clears after `MSG_TIMEOUT_US`.

Current behavior using those validity flags includes:

- Regen SOC cutoff only applies when HVC SOC data is valid.
- Rear brake pressure regen input is only used when MOBO power telemetry is valid.
- HVC summary validity is visible to downstream logic through `CAN_Manager_RX_Data_Valid`.

The tradeoff is intentional: these telemetry messages can tolerate software dispatch and timeout-based validity more safely than the inverter command TX path or config command RX path can tolerate dropped frames.

### Wheel Speed RX

Front left and front right RPM messages stay on dedicated DAQ extended RX FIFOs. Traction control depends on wheel speed validity, so these were not folded into the shared Controls telemetry FIFO.

## Performance Considerations

The software dispatch cost is small. It only applies to frames received through the shared Controls telemetry FIFO, and it loops over the small `rx_messages` table to find a matching ID. The torque command TX path does not pay this dispatch cost.

The more important performance concern is FIFO queue contention. The current design avoids queue contention for critical paths:

- Inverter TX uses a dedicated TX FIFO.
- Inverter feedback RX uses dedicated exact RX FIFOs.
- Config set RX uses a dedicated exact RX FIFO.
- Wheel speed RX uses dedicated exact RX FIFOs.

The only shared queue is for HVC/MOBO telemetry. If that FIFO overflows, CAN health telemetry should expose it through the FIFO health counters.

## Remaining Risks And Things To Watch

The main remaining risk is that the shared Controls telemetry mask may admit additional unrelated Controls extended IDs in the future. Unknown IDs are ignored by software, but they can still consume entries in the shared FIFO. If the Controls bus gains more traffic that matches `0x1F9FFE1F`, we may need to tighten the mask or create another shared group.

Watch these diagnostics after adding future CAN messages:

- `VCU_CAN_Health_FIFO` mux for the shared telemetry FIFO owner, currently `RX_HVC_SUMMARY`.
- `VCU_CAN_Health_FIFO` mux for `RX_SET_VCU_CONFIG`.
- `fifo_full_count`, `overflow_count`, and `other_error_count`.
- Loss of `data_vld` on HVC, MOBO, or wheel speed messages.

## Guidance For Future CAN Additions

Do not add one hardware RX FIFO per new message by default. On TTC60, assume hardware FIFO/message-object handles are scarce.

Use this priority order when adding messages:

1. Reserve TX FIFOs first.
2. Keep inverter command TX and inverter feedback RX dedicated.
3. Keep one-shot command/config RX messages dedicated if they must work from a single frame.
4. Keep traction-control-critical wheel speed RX dedicated unless proven safe otherwise.
5. Share periodic telemetry RX messages when they are cached, timeout-protected, and safe to software-dispatch.
6. Check build and hardware behavior after every handle-count change.

If another critical RX message is added, it should probably receive a dedicated exact FIFO. If that pushes the handle count too high, the right fix is to merge lower-criticality telemetry groups more carefully, not to share critical control traffic.

## Bottom Line

The old system was simple but did not scale on TTC60 because it assumed unlimited hardware FIFOs. The new system reserves critical TX first, keeps critical command/feedback/config paths dedicated, and shares only lower-criticality periodic telemetry. This should improve reliability of critical VCU functionality while still allowing the new regen, SOC, rear brake pressure, and readback features to coexist within the target's CAN hardware limits.