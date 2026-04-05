# Project: multiple-imu

## Hardware
- MCU: Seeed Studio XIAO ESP32S3 (master), XIAO ESP32C3 (slaves)
- IMU: Adafruit BNO085, I2C @ 0x4A, GPIO6=SDA GPIO7=SCL, 400kHz
- Master USB port: /dev/cu.usbmodem1101  (MAC: 90:70:69:11:DE:74, ESP32S3, 8MB flash)
- Slave 1 USB port: /dev/cu.usbmodem21301 (MAC: 58:8C:81:A7:B6:20, node_id=0x20)
- Slave 2: second XIAO ESP32C3, flashed same firmware (node_id = last byte of its MAC)
- ESP-IDF: v6.1-dev, channel 1

## Build & Flash
- Aliases: `build-master` `flash-master` `monitor-master` `build-slave` `flash-slave` `monitor-slave`
- Flash fail "Serial data stream stopped" → hold BOOT + press RESET on board, then flash immediately
- Flash fail "device not configured" → press RESET once (no BOOT) to exit download mode
- Firmware changes need rebuild+flash; Python/HTML changes need bridge restart only

## Packet & Frame Formats
- `imu_packet_t`: 50 bytes packed — node_id(u8) calib(u8) timestamp_us(u32) qi/qj/qk/qr(f32×4) quat_acc(f32) ax/ay/az(f32×3) gx/gy/gz(f32×3)
- IMU frame to PC: `[0xAA][0x55][50B imu_packet_t][4B timestamp_ms][CRC8]` = 57 bytes
- Node-event frame to PC: `[0xAA][0xBB][node_id][state][mac×6][CRC8]` = 11 bytes
- Beacon (slave→master, ESP-NOW): `[0xB0][node_id][state][pad]` = 4 bytes
- Command (master→slave, ESP-NOW): `[cmd][param][pad×2][arg(u32)]` = 8 bytes
- Python struct: IMU=`<BBIfffffffffff` (50B) + timestamp_ms=`<I` (4B) = 54B payload
- CRC8 poly=0x07, init=0x00
- GOTCHA: PC epoch ms (~1.7 trillion in 2026) overflows uint32_t. Master sends ms-since-sync as uint32, bridge reconstructs absolute epoch: `sync_epoch_ms + offset_ms`

## Node States
- `NODE_IDLE=0`: booted, BNO085 not initialized, sends beacon every 1s
- `NODE_CONNECTED=1`: BNO085 ready, not streaming, beacon every 5s
- `NODE_STREAMING=2`: actively sending IMU packets at configured rate

## Commands (master→slave via ESP-NOW)
- `CMD_RATE=0x01` param=Hz — change sample rate (10–200Hz)
- `CMD_START=0x02` — begin streaming
- `CMD_STOP=0x03` — pause streaming
- `CMD_CALIB=0x04` — if STREAMING/CONNECTED: quaternion tare (reset orientation to zero); if IDLE: full BNO085 reinit
- `CMD_SYNC=0x05` arg=master_timestamp_us — time sync offset
- `CMD_CONNECT=0x06` — trigger BNO085 init, go CONNECTED
- `CMD_DISCONNECT=0x07` — stop streaming, go back to IDLE

## Serial Commands (bridge→master over USB, text)
- `RATE:<hz>\n` `START:<node_id>\n` `STOP:<node_id>\n` `CALIB:<node_id>\n`
- `SYNC\n` `CONNECT:<node_id>\n` `DISCONNECT:<node_id>\n`
- `S<epoch_ms>\n` — PC time sync: master stores epoch ref + local clock, stamps IMU frames
- node_id=255 means broadcast to all slaves

## Slave Key Facts
- node_id = last byte of own WiFi MAC (set at boot, no Kconfig needed — all boards use identical firmware)
- `CONFIG_SLAVE_MASTER_MAC` in sdkconfig must match master's WiFi MAC (currently `90:70:69:11:DE:74`)
- BNO085 init deferred to CMD_CONNECT (not at boot) — runs in main task, not in ESP-NOW callback
- `g_connect_requested` / `g_calib_requested` / `g_tare_requested` flags set in callback, handled in main loop
- `g_time_offset_us` (int32) applied to timestamp: `(uint32_t)esp_timer_get_time() + (uint32_t)g_time_offset_us`
- Quaternion tare: `g_ref_qi/qj/qk/qr` stores reference orientation; `quat_apply_tare()` computes relative quaternion via Hamilton product
- Connection loss detection: ESP-NOW send callback tracks consecutive failures; after 20 fails → reset to IDLE and resume beaconing

## Master Key Facts
- Discovers slaves from 4-byte beacons; adds as ESP-NOW send-peer on first beacon
- Emits 11-byte node-event frame to PC on EVERY beacon (not just state change) — ensures bridge.py stays in sync even if started after master
- `slave_table[MAX_SLAVES=8]` protected by `slave_mutex`
- `send_node_event()` called OUTSIDE mutex to avoid priority inversion with `send_cmd()`
- Master target: ESP32-S3 (set via `idf.py set-target esp32s3`); console on USB_SERIAL_JTAG (not UART)

## Bridge (pc/bridge.py)
- Start: `cd pc && python3 bridge.py --port /dev/cu.usbmodem1101` — opens browser at http://localhost:8080/index.html
- HTTP server :8080 serves pc/ directory; WebSocket server :8765
- Serial reader dispatches on 2nd magic byte: 0x55=IMU frame, 0xBB=node-event frame
- HDF5 opened with mode `'w'` (fresh each run) — avoids SWMR lock errors
- `reconnect_serial()` defined but not yet wired into exception handler (pending)
- Recording: buffers rows in memory; on stop, sends CSV over WebSocket for browser download (no filesystem picker needed)
- Broadcasts `master_status` message every 1s (serial port, connection state, ESP-NOW channel)
- IMPORTANT: only bridge.py should hold the master serial port — close CLion monitor before starting

## UI (pc/index.html)
- Toss Design System: dark mode, sans-serif, no borders (bg-color layers), blue accent #3182f6, large radius
- Layout: sidebar (master status, device cards, sample rate, filter, recording) + main area (header bar + scrollable IMU grid)
- Each streaming IMU gets a row: 3D board (+ time display + euler) + accel plot + gyro plot, all horizontal
- 3D board: flat PCB shape with colored axis arrows (X=red, Y=green, Z=blue), rotated by quaternion
- Throttled render loop: 30fps max, 6 canvases per frame, dirty flag per node
- Ring buffers: Float32Array with head pointer (no GC from push/shift)
- Header bar chips: Serial status, Connected/Streaming counts, ESP-NOW channel, live pkt/s counter
- Butterworth filter: 2nd-order IIR biquad, display-only (raw data always saved). Options: None / Low Pass / High Pass with cutoff slider
- Recording: filename input + Start/Stop button; on stop, browser triggers native CSV download
- Plot Y-axis: auto-scale with minimum floor (accel ±20 m/s², gyro ±10 rad/s) — never shrinks below floor
- Never call `renderNodeList()` inside `requestAnimationFrame` — destroys button DOM at 60fps, swallows clicks
- Never use `ResizeObserver` on a canvas you set `.width/.height` on — creates resize feedback loop

## What's Done ✅
- BNO085 I2C, SHTP, all 3 reports (rotation vector + accel + gyro) confirmed on hardware
- ESP-NOW unicast slave→master, master→slave commands, all confirmed working
- Full state machine: IDLE→CONNECTED→STREAMING with beacon-based discovery
- Bridge: serial reader, HDF5+CSV logging, WebSocket broadcast, HTTP dashboard
- Master migrated from ESP32-C3 → ESP32-S3 (XIAO ESP32S3): USB_SERIAL_JTAG console, 8MB flash
- Slave `CONFIG_SLAVE_MASTER_MAC` updated to S3 master MAC; both slaves flashed
- Master emits node-event on every beacon (not just first discovery) for bridge sync
- Slave connection loss detection: 20 consecutive ESP-NOW send fails → reset to IDLE
- Slave quaternion tare: CMD_CALIB during streaming captures current orientation as zero reference
- UI: full rewrite with Toss Design System (dark mode, Toss palette, no borders)
- UI: multi-IMU grid (each node = 3D board + accel plot + gyro plot, up to 8 rows)
- UI: 3D IMU board visualization with colored XYZ axis arrows
- UI: master status bar with live pkt/s throughput counter
- UI: per-node sidebar cards with Connect/Start/Stop/Calib/Disconnect buttons
- UI: recording with in-memory buffering → browser native CSV download on stop
- UI: 2nd-order Butterworth filter (None / Low Pass / High Pass) with cutoff slider, display-only
- Bridge: master_status broadcast (serial port, connection state, ESP-NOW channel)
- Bridge: removed tkinter dependency; recording uses memory buffer + WebSocket download
- Time sync: PC epoch → master → 57-byte frame with timestamp_ms → bridge CSV/HDF5
- UI: sync status chip in header, per-node elapsed time display, horizontal IMU row layout
- UI: plot Y-axis minimum scale floor (auto-scale that never shrinks below threshold)

## What's Done Today (Apr 3, 2026) 📅
1. **Time Sync (M1-M5):** Master parses `S<epoch_ms>\n` from serial, stores sync reference (`g_sync_epoch_ms` + `g_sync_master_us`), extends IMU frame to 57 bytes with `uint32_t timestamp_ms` before CRC, logs sync status
2. **Time Sync (B1-B5):** Bridge handles `sync_manual` WS cmd → sends `S<epoch_ms>\n` to master, parses 57-byte frame (54B payload), adds `timestamp_ms` column to CSV/HDF5, auto-syncs on `record_start`, sends `sync_ack` to GUI, reports `synced`/`sync_age_s` in `master_status`
3. **Time Sync (G1-G4):** Fixed `sendSync()` (single `sync_manual` cmd), added sync status chip in header (green/amber with HH:MM:SS), handles `sync_ack` + `master_status` sync fields, auto-triggers sync on record start if not synced

## What Was Done (Apr 2, 2026)
1. Web GUI full rewrite: Toss design system, multi-IMU grid, 3D board + axes, throttled renderer
2. bridge.py: master_status broadcast, in-memory recording + browser download, removed tkinter
3. Master C3→S3 migration: set-target, USB console, 8MB flash, sdkconfig verified
4. Slave: updated master MAC, connection loss detection, quaternion tare on Calib
5. Master: node-event on every beacon (fixes bridge missing initial discovery)
6. UI: Butterworth filter controls (None/LP/HP + cutoff Hz slider)
7. UI: live pkt/s counter in header bar

## What's Pending ❌
- **Slaves not showing in dashboard** — master sees 2 slaves (confirmed via monitor: `slaves=2`) but bridge.py may not be parsing node-event frames reliably when mixed with ESP_LOGI text on same USB CDC stream. Debug serial reader or suppress text logging on master.
- Wire `reconnect_serial()` into bridge exception handler
- End-to-end pipeline test: connect → stream → record → stop → download CSV
- Verify Butterworth filter with real IMU data (tested with simulated data only)
- Investigate noisy/square-wave plot data — may be BNO085 I2C or SHTP parsing issue
- CLion CMake profiles: update master to `IDF_TARGET=esp32s3`, set `ESPPORT` env var per config
- perfboard HTML docs still reference "XIAO ESP32C3" for master (cosmetic, low priority)

## 🕐 Time Synchronization — Implemented (Apr 3, 2026)

Full plan: PC → master serial → master timestamps slave packets at reception → CSV gets absolute wall-clock timestamps.
No slave firmware changes needed for v1. Accuracy ≈ ±1–2 ms.

### bridge.py (B)
- [x] **B1** Handle `sync_manual` WebSocket command: compute `t_epoch_ms = int(time.time()*1000)`, send `S{t_epoch_ms}\n` over serial, store as `sync_epoch_ms`
- [x] **B2** Auto-sync on `record_start` (always ensure a clock reference before capture begins)
- [x] **B3** Parse `timestamp_ms` from extended IMU frame (frame grows 53→57 bytes; new `uint32_t` at byte offset 52)
- [x] **B4** Add `timestamp_ms` column to CSV output (absolute epoch ms)
- [x] **B5** Send `{ type: 'sync_ack', epoch_ms }` WebSocket message to GUI after serial send

### master/main/main.c (M)
- [x] **M1** Parse serial SYNC command `S<digits>\n` in USB serial RX loop
- [x] **M2** Store sync reference: `g_sync_epoch_ms` (uint64_t) + `g_sync_master_us = esp_timer_get_time()` at parse moment
- [x] **M3** Extend forwarded IMU frame: append `uint32_t timestamp_ms` before CRC → frame becomes 57 bytes `[0xAA][0x55][54B][CRC8]`
- [x] **M4** Compute timestamp on forward: `ts = g_sync_epoch_ms + (esp_timer_get_time() - g_sync_master_us) / 1000`; zero-fill if not yet synced
- [x] **M5** Add `synced: bool` + `sync_age_s: float` to the periodic `master_status` serial broadcast

### slave/main/main.c (S) — v1: no changes needed
- [ ] **S2** *(v2, optional)* Receive ESP-NOW SYNC frame `[0xAA][0xCC][8B master_us][CRC8]` from master; store `slave_offset_us = master_us − esp_timer_get_time()`; apply in pack loop for sub-ms accuracy

### pc/index.html (G)
- [x] **G1** Fix `sendSync()`: remove duplicate send, keep only `{ cmd: 'sync_manual' }`
- [x] **G2** Add sync status chip in header: green `Synced HH:MM:SS` or amber `Not synced`
- [x] **G3** Handle `sync_ack` WebSocket message: update chip text + store `syncEpochMs`
- [x] **G4** Auto-trigger sync when user clicks "Start Recording" if not yet synced

### Implementation order
```
M1 → M2 → M3 → M4 → M5   (one master firmware build + flash)
B1 → B3 → B4 → B2 → B5   (bridge.py, depends on M3 frame size change)
G1 → G2 → G3 → G4         (GUI, depends on B5 sync_ack message)
S2                          (optional v2 after everything else verified)
```

### Verification checklist
- [ ] Flash master → serial monitor → send `S1712345678000` manually → no crash, `g_sync_epoch_ms` set
- [ ] Start bridge → click Sync button → confirm `sync_ack` in browser console
- [ ] Stream + record → stop → open CSV → `timestamp_ms` column is monotonically increasing
- [ ] Two slaves streaming simultaneously → `timestamp_ms` values within 5 ms of each other

---

# Response Style

## Before starting work
- Briefly state what you understood the objective to be
- List the steps you plan to take (2-5 lines max)
- Then proceed immediately — do not wait for approval unless destructive

## While working
- One line per action taken, present tense ("Updating bridge.py to reconnect on serial error")
- If something unexpected is found mid-task, note it inline and keep going

## After finishing
- Short summary of what changed and why
- Flag anything that still needs attention

## General
- Do not re-explain known architecture or context already in memory
- Prefer direct fixes over instructions telling the user to fix it themselves
- Never ask for confirmation on non-destructive changes
