# Multiple-IMU

Distributed multi-node IMU telemetry system for real-time motion capture and data recording.

## Architecture

```
BNO085 ──I2C──> Slave (ESP32-C3) ──ESP-NOW──> Master (ESP32-S3) ──USB──> Bridge (Python) ──WS──> Browser
```

- **Master** (XIAO ESP32-S3): Discovers slaves, aggregates IMU data, forwards framed packets to PC over USB
- **Slaves** (XIAO ESP32-C3 x N): BNO085 IMU sensor nodes streaming orientation and acceleration via ESP-NOW
- **Bridge** (`pc/bridge.py`): Python WebSocket bridge — serial reader, HDF5/CSV logging, time sync
- **Dashboard** (`pc/index.html`): Real-time browser UI with 3D visualization, plots, recording, filtering

## Hardware

| Component | Spec |
|-----------|------|
| Master MCU | Seeed Studio XIAO ESP32-S3 |
| Slave MCU | Seeed Studio XIAO ESP32-C3 (up to 8) |
| IMU Sensor | Adafruit BNO085 (I2C, 0x4A) |
| Wireless | ESP-NOW, Channel 1 |
| I2C Pins | SDA=GPIO6, SCL=GPIO7, 400 kHz |

All slaves run **identical firmware** — node identity is derived from the last byte of each board's WiFi MAC address.

## Quick Start

### Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) v5.x+
- Python 3.10+ with packages: `pyserial`, `websockets`, `h5py`, `numpy`

### Flash Firmware

```bash
# Master (ESP32-S3)
cd master
idf.py set-target esp32s3
idf.py build flash monitor -p /dev/cu.usbmodem1101

# Slave (ESP32-C3) — same firmware for all slaves
cd slave
idf.py set-target esp32c3
idf.py build flash monitor -p /dev/cu.usbmodem21301
```

### Start Dashboard

```bash
# With MCU connected (auto-detects port)
./start-ui.sh

# Or specify port
./start-ui.sh /dev/cu.usbmodem1101

# UI only (no MCU needed)
./start-ui.sh --no-serial
```

The dashboard opens at `http://localhost:8080/index.html`. Select the serial port from the sidebar dropdown if not auto-connected.

### Shell Aliases

Add to `~/.zshrc`:

```bash
alias imu='python3 /path/to/multiple-imu/pc/bridge.py'
alias imu-ui='python3 /path/to/multiple-imu/pc/bridge.py --no-serial'
```

## Node States

```
IDLE ──CMD_CONNECT──> CONNECTED ──CMD_START──> STREAMING
  ^                       |                        |
  |       CMD_DISCONNECT  |      CMD_STOP          |
  +<──────────────────────+<───────────────────────+
```

| State | Description | Beacon Interval |
|-------|-------------|-----------------|
| IDLE | Booted, BNO085 off | 1 s |
| CONNECTED | BNO085 initialized, ready | 5 s |
| STREAMING | Sending IMU packets at 10-200 Hz | 5 s |

## Packet Formats

### IMU Frame (57 bytes, Master -> PC)

```
[0xAA][0x55][50B imu_packet_t][4B timestamp_ms][CRC8]
```

| Offset | Size | Field | Type |
|--------|------|-------|------|
| 0-1 | 2 | Magic bytes | 0xAA 0x55 |
| 2 | 1 | node_id | uint8 |
| 3 | 1 | calib | uint8 |
| 4 | 4 | timestamp_us | uint32 |
| 8 | 16 | qi, qj, qk, qr | float x4 |
| 24 | 4 | quat_acc | float |
| 28 | 12 | ax, ay, az | float x3 |
| 40 | 12 | gx, gy, gz | float x3 |
| 52 | 4 | timestamp_ms | uint32 |
| 56 | 1 | CRC8 | poly=0x07 |

### Other Frames

- **Beacon** (4B, slave -> master): `[0xB0][node_id][state][pad]`
- **Node Event** (11B, master -> PC): `[0xAA][0xBB][node_id][state][mac x6][CRC8]`
- **Command** (8B, master -> slave): `[cmd][param][pad x2][arg(u32)]`

## Time Synchronization

Three-layer clock sync: **PC -> Master -> Slaves**

1. **PC sends epoch**: `S<epoch_ms>\n` over serial
2. **Master stores reference**: `g_sync_epoch_ms` + `g_sync_master_us = esp_timer_get_time()`
3. **Master stamps frames**: `ts_ms = (esp_timer_get_time() - g_sync_master_us) / 1000`
4. **Master syncs slaves**: Broadcasts `CMD_SYNC` with `arg = esp_timer_get_time()`
5. **Slave computes offset**: `g_time_offset_us = master_ts - slave_ts`
6. **Bridge reconstructs**: `absolute_ms = sync_epoch_ms + ts_offset_ms`

### Sync Error Thresholds

| Range | Quality |
|-------|---------|
| < 5 ms | Excellent |
| 5-10 ms | Good |
| 10-20 ms | Warning — consider re-sync |
| > 20 ms | Bad — re-sync required |

## Dashboard Features

- **3D Board Visualization**: Per-node quaternion-driven 3D PCB with XYZ axis arrows
- **Accel/Gyro Plots**: Scrolling time-series with auto-scaling Y-axis
- **Butterworth Filter**: Low pass and high pass (display-only, raw data always saved)
- **Recording**: In-memory buffering with browser CSV download, auto time-sync on start
- **Time Sync Monitor**: Overlay showing per-node latency, inter-node sync error, sample rates
- **Node Naming**: Double-click node name to rename, persisted in `pc/node_names.json`
- **Port Selection**: Scan and connect to serial ports from the UI

## Project Structure

```
multiple-imu/
  master/
    main/main.c          # ESP32-S3 coordinator firmware
  slave/
    main/main.c          # ESP32-C3 sensor node firmware
    main/imu_packet.h    # Shared packet struct (50 bytes)
    components/
      bno085/             # BNO085 SHTP I2C driver
      bno055/             # BNO055 driver (legacy, unused)
  pc/
    bridge.py             # WebSocket bridge + serial reader
    index.html            # Dashboard UI
    docs.html             # Technical documentation
    node_names.json       # Persisted node name mappings
  start-ui.sh             # Launch script
  CLAUDE.md               # Development context & notes
```

## Documentation

Full technical documentation with diagrams, equations, and byte layouts:

- Run `imu` or `imu-ui`, then open `http://localhost:8080/docs.html`
- Or open `pc/docs.html` directly in a browser

## License

MIT
