# AVCC Electronics

4-node ESP32 system for autonomous vascular compression research: 1 master + 3 slaves coordinating 12-channel waveform output over **ESP-NOW** (not WiFi).

Architecture spec: `AVCC_Architecture.tex` (authoritative, 77-page reference).

## Layout

| Directory | Role |
|---|---|
| `Master_Pressure_Servo_Electrical/` | Coordinator. Wi-Fi AP `AVCC_Master 5` @ 192.168.4.1, web UI, state machine (IDLE → WAIT_ACKS → RUNNING) |
| `Electrical_Receiver_V2/` | 12-ch DAC slave (±15V via 8-bit parallel bus). Dual-core: `waveformTask` spin-waits for sub-ms edge timing, `loop()` handles ESP-NOW |
| `Pressure_Receiver_V2/` | 12-ch solenoid GPIO slave. Two pulses per electrical period. Per-channel auto-stop |
| `Servo_Receiver_V2/` | 12-ch servo slave. Ramped motion (11 ms/deg up, 3.67 ms/deg down). **2 Hz hard cap** — rejects higher-frequency requests with nack |
| `BICEP_MAC_Address/` | Utility: prints ESP32 STA MAC at 115200 baud (must set `WIFI_STA` mode first or it reads as zeros) |
| `Test_on_Wifi/` | Utility: Wi-Fi AP smoke test |

## Wire protocol

- `Msg` v3 (36 B): period_us, pulse_us, start_delay_us, phase_offset_us, duration_ms. v1 (24 B) auto-detected for backward compat.
- Flow: `CMD_SET` → `CMD_START` → `CMD_KEEPALIVE` every 150 ms. **Failsafe:** slaves auto-stop if no keepalive for 500 ms.
- Reliability: per-channel sequence guards, run-ID reset on master reboot, targeted retransmit (3 retries @ 600 ms).
- Status uplink is **1 Hz** (intentionally halved from 2 Hz to reduce traffic).
- Ack/Status types: `0xA1` / `0xA2`.

## Gotchas

- **Slave MACs are hard-coded** in master (around lines 36–38); an older set is commented out just above. Verify these match your hardware before flashing.
- **Electrical pin remap**: logical channels 2↔3, 6↔7, 10↔11 are swapped to physical pins. Don't "fix" this — it's deliberate.
- **No String allocations** in master web UI — HTML is streamed to avoid heap fragmentation. Don't reintroduce `String +=` patterns.
- **Boot auto-start grace period** is ~3 sec by default.
- Master was refactored from per-loop `applyOneStep()` to `burstAllSets()` + `burstAllStarts()`. Don't revert.

## Not a git repo

No version control here. Be conservative with destructive edits — there's no `git checkout` to fall back on.
