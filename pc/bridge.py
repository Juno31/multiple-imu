#!/usr/bin/env python3
"""
IMU WebSocket bridge.

Reads 53-byte framed packets from master USB CDC, streams JSON to browser
via WebSocket, and forwards text commands from browser to MCU via serial.

Serial command format (written to serial port):
  RATE:<hz>\\n      — change sample rate for all slaves
  START:<node_id>\\n — resume streaming (255 = all)
  STOP:<node_id>\\n  — pause streaming  (255 = all)
  CALIB:<node_id>\\n — reinitialize BNO085

WebSocket messages (server → client):
  {"type":"imu",  "node_id":1, "ts":..., "qi":..., ...}
  {"type":"nodes","nodes":[{"id":1,"active":true,"last_seen":0.3},...]}
  {"type":"ack",  "cmd":"rate"}

WebSocket messages (client → server):
  {"cmd":"rate",  "value":100}
  {"cmd":"start", "node_id":1}   (node_id 255 = all)
  {"cmd":"stop",  "node_id":1}
  {"cmd":"calib", "node_id":1}
"""

import asyncio
import csv
import json
import struct
import time
import threading
import argparse
import webbrowser
import pathlib
from http.server import HTTPServer, SimpleHTTPRequestHandler
import functools

import h5py
import numpy as np
import serial
import serial.tools.list_ports
import websockets

import io

# Directory containing this script (serves index.html and imu_data.h5 from here)
HERE = pathlib.Path(__file__).parent

# ── Constants ────────────────────────────────────────────────────────────────
DEFAULT_PORT      = '/dev/cu.usbmodem101'
DEFAULT_BAUD      = 115200
DEFAULT_HDF5      = str(HERE / 'imu_data.h5')
DEFAULT_CSV       = str(HERE / 'imu_data.csv')
DATA_DIR          = HERE.parent / 'data'          # recordings saved here
DEFAULT_WS_PORT   = 8765
DEFAULT_HTTP_PORT = 8080

MAGIC0      = 0xAA
MAGIC1      = 0x55
PAYLOAD_LEN = 54          # 50B imu_packet + 4B timestamp_ms
FRAME_TOTAL = 57          # [0xAA][0x55][54B payload][CRC8]
IMU_STRUCT  = '<BBIfffffffffff'   # 50 bytes: node_id..gz
TS_MS_FMT   = '<I'                # 4 bytes: uint32 timestamp_ms
CRC8_POLY   = 0x07

# Node-event frame: [0xAA][0xBB][node_id(1)][state(1)][mac(6)][CRC8] = 11 bytes
NODE_EVT_MAGIC1   = 0xBB
NODE_EVT_PAYLOAD  = 8    # node_id + state + mac(6)
NODE_EVT_TOTAL    = 11   # 2 magic + 8 payload + 1 CRC
NODE_STATES       = {0: 'idle', 1: 'connected', 2: 'streaming'}

FIELDS = [
    'node_id', 'calib', 'timestamp_us',
    'qi', 'qj', 'qk', 'qr', 'quat_acc',
    'ax', 'ay', 'az', 'gx', 'gy', 'gz',
    'timestamp_ms',
]

# ── Node names (MAC → friendly name, persisted to JSON) ─────────────────────
NODE_NAMES_FILE = str(HERE / 'node_names.json')
node_names: dict = {}   # "XX:XX:XX:XX:XX:XX" → "Left Arm"

def load_node_names():
    global node_names
    try:
        with open(NODE_NAMES_FILE, 'r') as f:
            node_names = json.load(f)
        print(f'[names] Loaded {len(node_names)} name(s) from {NODE_NAMES_FILE}')
    except (FileNotFoundError, json.JSONDecodeError):
        node_names = {}

def save_node_names():
    with open(NODE_NAMES_FILE, 'w') as f:
        json.dump(node_names, f, indent=2)
    print(f'[names] Saved {len(node_names)} name(s)')

# ── Shared state ─────────────────────────────────────────────────────────────
clients: set       = set()
nodes:   dict      = {}    # node_id → {last_seen, active}
ser_ref: serial.Serial | None = None
data_queue: asyncio.Queue | None = None

# Recording state (accessed from serial thread + asyncio)
rec_lock = threading.Lock()
rec: dict = {'active': False, 'rows': []}  # rows = list of tuples buffered in memory

# Time sync state
sync_epoch_ms: int = 0     # PC wall-clock epoch ms sent to master
sync_time: float   = 0.0   # time.time() when sync was sent


# ── Helpers ──────────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def open_hdf5(path: str):
    # Always create fresh — avoids stale SWMR lock from previous sessions
    f = h5py.File(path, 'w', libver='latest')
    if 'imu' not in f:
        dt = np.dtype([
            ('node_id',      np.uint8),  ('calib',        np.uint8),
            ('timestamp_us', np.uint32),
            ('qi',  np.float32), ('qj',  np.float32),
            ('qk',  np.float32), ('qr',  np.float32),
            ('quat_acc',     np.float32),
            ('ax',  np.float32), ('ay',  np.float32), ('az',  np.float32),
            ('gx',  np.float32), ('gy',  np.float32), ('gz',  np.float32),
            ('timestamp_ms', np.uint32),
        ])
        f.create_dataset('imu', shape=(0,), maxshape=(None,), dtype=dt, chunks=True)
    f.swmr_mode = True
    return f


# ── Serial helpers ────────────────────────────────────────────────────────────
def list_serial_ports() -> list[dict]:
    """Return available serial ports as [{port, desc, hwid}, ...]."""
    return [
        {'port': p.device, 'desc': p.description, 'hwid': p.hwid}
        for p in serial.tools.list_ports.comports()
    ]


def connect_serial(port: str, baud: int) -> bool:
    """Open serial port, start reader thread. Returns True on success."""
    global ser_ref
    if ser_ref and ser_ref.is_open:
        try:
            ser_ref.close()
        except Exception:
            pass

    try:
        ser_ref = serial.Serial(port, baud, timeout=0.1)
        print(f'[serial] connected to {port}')

        # Start reader thread (only if HDF5/CSV are ready)
        if _hf_ref and _csv_writer_ref and _loop_ref and data_queue:
            threading.Thread(
                target=serial_reader,
                args=(ser_ref, _hf_ref, _csv_writer_ref, _csv_file_ref,
                      _loop_ref, data_queue),
                daemon=True,
            ).start()
        return True
    except serial.SerialException as e:
        print(f'[serial] failed to open {port}: {e}')
        ser_ref = None
        return False


def disconnect_serial():
    """Close the current serial port."""
    global ser_ref
    if ser_ref and ser_ref.is_open:
        try:
            ser_ref.close()
            print('[serial] disconnected')
        except Exception:
            pass
    ser_ref = None


def reconnect_serial(port: str, baud: int) -> serial.Serial:
    """Block until the serial port comes back, then return a fresh Serial."""
    while True:
        try:
            s = serial.Serial(port, baud, timeout=0.1)
            print(f'[serial] reconnected to {port}')
            return s
        except serial.SerialException:
            time.sleep(1.0)

# Refs kept so connect_serial() can start the reader thread later
_hf_ref = None
_csv_writer_ref = None
_csv_file_ref = None
_loop_ref = None



# ── Serial reader (background thread) ────────────────────────────────────────
def serial_reader(ser: serial.Serial, hf, csv_writer, csv_file,
                  loop: asyncio.AbstractEventLoop, queue: asyncio.Queue):
    ds    = hf['imu']
    total = 0
    bad   = 0

    while True:
        try:
            b = ser.read(1)
            if not b or b[0] != MAGIC0:
                continue
            b2 = ser.read(1)
            if not b2:
                continue

            # ── IMU data frame [0xAA][0x55] ──────────────────────────────
            if b2[0] == MAGIC1:
                rest = ser.read(FRAME_TOTAL - 2)
                if len(rest) != FRAME_TOTAL - 2:
                    continue

                payload = rest[:PAYLOAD_LEN]
                crc_rx  = rest[PAYLOAD_LEN]
                if crc8(payload) != crc_rx:
                    bad += 1
                    continue

                imu_vals      = struct.unpack(IMU_STRUCT, payload[:50])
                ts_offset_ms  = struct.unpack(TS_MS_FMT, payload[50:54])[0]
                # Reconstruct absolute epoch ms (bridge holds full 64-bit sync ref)
                ts_ms = (sync_epoch_ms + ts_offset_ms) if sync_epoch_ms else 0
                vals  = imu_vals + (ts_ms,)

                # HDF5
                row = np.array([vals], dtype=ds.dtype)
                ds.resize(ds.shape[0] + 1, axis=0)
                ds[-1] = row
                total += 1
                if total % 50 == 0:
                    hf.flush()
                    csv_file.flush()

                # Continuous CSV
                csv_writer.writerow(vals)

                # Recording buffer (if active)
                with rec_lock:
                    if rec['active']:
                        rec['rows'].append(vals)

                msg = dict(zip(FIELDS, vals))
                msg['type'] = 'imu'
                msg['timestamp_ms'] = ts_ms

                node_id = int(vals[0])
                if node_id not in nodes:
                    nodes[node_id] = {
                        'last_seen': time.time(), 'active': True,
                        'state': 'streaming', 'mac': '',
                    }
                else:
                    nodes[node_id]['last_seen'] = time.time()
                    nodes[node_id]['state']     = 'streaming'

                asyncio.run_coroutine_threadsafe(queue.put(json.dumps(msg)), loop)

            # ── Node-event frame [0xAA][0xBB] ────────────────────────────
            elif b2[0] == NODE_EVT_MAGIC1:
                rest = ser.read(NODE_EVT_TOTAL - 2)
                if len(rest) != NODE_EVT_TOTAL - 2:
                    continue

                payload = rest[:NODE_EVT_PAYLOAD]   # node_id+state+mac
                crc_rx  = rest[NODE_EVT_PAYLOAD]
                if crc8(payload) != crc_rx:
                    bad += 1
                    continue

                node_id   = payload[0]
                state_int = payload[1]
                mac_str   = ':'.join(f'{x:02X}' for x in payload[2:8])
                state_str = NODE_STATES.get(state_int, 'idle')

                if node_id not in nodes:
                    nodes[node_id] = {
                        'last_seen': time.time(), 'active': False,
                        'state': state_str, 'mac': mac_str,
                    }
                else:
                    nodes[node_id]['state'] = state_str
                    nodes[node_id]['mac']   = mac_str
                    if state_str == 'streaming':
                        nodes[node_id]['active'] = True
                    elif state_str == 'idle':
                        nodes[node_id]['active'] = False

                event = json.dumps({
                    'type':    'node_event',
                    'node_id': int(node_id),
                    'state':   state_str,
                    'mac':     mac_str,
                })
                asyncio.run_coroutine_threadsafe(queue.put(event), loop)
                print(f'[node] node={node_id} state={state_str} mac={mac_str}')

        except Exception as e:
            print(f'[serial] {e}')
            time.sleep(0.1)


# ── WebSocket helpers ─────────────────────────────────────────────────────────
async def broadcast(message: str):
    if not clients:
        return
    await asyncio.gather(*(c.send(message) for c in list(clients)),
                         return_exceptions=True)


async def queue_broadcaster(queue: asyncio.Queue):
    while True:
        msg = await queue.get()
        await broadcast(msg)


async def node_status_task():
    while True:
        await asyncio.sleep(1.0)
        now = time.time()
        status = {
            'type': 'nodes',
            'nodes': [
                {
                    'id':        k,
                    'active':    v['active'],
                    'last_seen': round(now - v['last_seen'], 1),
                    'state':     v.get('state', 'idle'),
                    'mac':       v.get('mac', ''),
                    'name':      node_names.get(v.get('mac', ''), ''),
                }
                for k, v in nodes.items()
            ],
        }
        await broadcast(json.dumps(status))

        # Master/bridge status (every tick alongside nodes)
        master_status = {
            'type':             'master_status',
            'serial_port':      ser_ref.port if ser_ref else '',
            'serial_connected': bool(ser_ref and ser_ref.is_open),
            'ws_clients':       len(clients),
            'espnow_channel':   1,
            'synced':           sync_epoch_ms > 0,
            'sync_age_s':       round(time.time() - sync_time, 1) if sync_epoch_ms > 0 else -1,
        }
        await broadcast(json.dumps(master_status))


# ── Time sync helper ─────────────────────────────────────────────────────────
async def do_sync(websocket):
    """Send S<epoch_ms> to master (PC→master sync) + SYNC (master→slaves sync).
    This resets all clocks to 0 relative to sync moment."""
    global sync_epoch_ms, sync_time
    if ser_ref and ser_ref.is_open:
        sync_epoch_ms = int(time.time() * 1000)
        sync_time = time.time()
        # 1. PC epoch → master (sets master's time reference)
        ser_ref.write(f'S{sync_epoch_ms}\n'.encode())
        print(f'[serial→] S{sync_epoch_ms}')
        # 2. Master → all slaves (broadcasts CMD_SYNC via ESP-NOW)
        ser_ref.write(b'SYNC\n')
        print('[serial→] SYNC')
        await websocket.send(json.dumps({
            'type': 'sync_ack',
            'epoch_ms': sync_epoch_ms,
        }))


# ── WebSocket connection handler ──────────────────────────────────────────────
async def handle_client(websocket):
    clients.add(websocket)
    print(f'[ws] client connected  ({len(clients)} total)')
    try:
        async for message in websocket:
            try:
                cmd = json.loads(message)
            except json.JSONDecodeError:
                continue

            c = cmd.get('cmd', '')
            line = None

            if c == 'rate':
                hz = int(cmd.get('value', 100))
                line = f'RATE:{hz}\n'

            elif c == 'start':
                nid = int(cmd.get('node_id', 255))
                line = f'START:{nid}\n'
                if nid in nodes:
                    nodes[nid]['active'] = True
                elif nid == 255:
                    for v in nodes.values():
                        v['active'] = True

            elif c == 'stop':
                nid = int(cmd.get('node_id', 255))
                line = f'STOP:{nid}\n'
                if nid in nodes:
                    nodes[nid]['active'] = False
                elif nid == 255:
                    for v in nodes.values():
                        v['active'] = False

            elif c == 'calib':
                nid = int(cmd.get('node_id', 255))
                line = f'CALIB:{nid}\n'

            elif c == 'connect':
                nid = int(cmd.get('node_id', 0))
                line = f'CONNECT:{nid}\n'

            elif c == 'disconnect':
                nid = int(cmd.get('node_id', 0))
                line = f'DISCONNECT:{nid}\n'
                if nid in nodes:
                    nodes[nid]['state']  = 'idle'
                    nodes[nid]['active'] = False

            elif c == 'sync_manual':
                # B1: PC time sync — send S<epoch_ms> to master
                await do_sync(websocket)

            elif c == 'list_ports':
                ports = list_serial_ports()
                connected_port = ser_ref.port if (ser_ref and ser_ref.is_open) else None
                await websocket.send(json.dumps({
                    'type': 'ports',
                    'ports': ports,
                    'connected': connected_port,
                }))
                continue   # no serial write needed

            elif c == 'serial_connect':
                port = cmd.get('port', '')
                baud = int(cmd.get('baud', DEFAULT_BAUD))
                ok = connect_serial(port, baud)
                await websocket.send(json.dumps({
                    'type': 'serial_status',
                    'connected': ok,
                    'port': port if ok else None,
                }))
                continue

            elif c == 'serial_disconnect':
                disconnect_serial()
                await websocket.send(json.dumps({
                    'type': 'serial_status',
                    'connected': False,
                    'port': None,
                }))
                continue

            elif c == 'get_names':
                await websocket.send(json.dumps({
                    'type': 'node_names',
                    'names': node_names,
                }))
                continue

            elif c == 'set_name':
                mac = cmd.get('mac', '')
                name = cmd.get('name', '').strip()
                if mac:
                    if name:
                        node_names[mac] = name
                    else:
                        node_names.pop(mac, None)
                    save_node_names()
                    # Broadcast updated names to all clients
                    await broadcast(json.dumps({
                        'type': 'node_names',
                        'names': node_names,
                    }))
                continue

            elif c == 'record_start':
                # B2: Auto-sync before recording
                await do_sync(websocket)
                await asyncio.sleep(0.05)
                # Start buffering in memory
                with rec_lock:
                    rec['rows'] = []
                    rec['active'] = True
                print('[rec] Started (buffering in memory)')
                await websocket.send(json.dumps({'type': 'record_started', 'file': 'recording...'}))

            elif c == 'record_stop':
                # 1. Stop buffering
                with rec_lock:
                    rec['active'] = False
                    rows = list(rec['rows'])
                    rec['rows'] = []
                # 2. Build CSV string with relative time_s column
                buf = io.StringIO()
                w = csv.writer(buf)
                w.writerow(FIELDS + ['time_s'])
                t0 = rows[0][-1] if rows else 0  # first row's timestamp_ms
                for r in rows:
                    rel_s = round((r[-1] - t0) / 1000.0, 4) if t0 else 0
                    w.writerow(list(r) + [rel_s])
                csv_text = buf.getvalue()
                # 3. Generate default filename
                fname = cmd.get('filename', '').strip()
                if not fname:
                    fname = time.strftime('%y%m%d_%H%M%S') + '.csv'
                if not fname.lower().endswith('.csv'):
                    fname += '.csv'
                print(f'[rec] Stopped — {len(rows)} rows, sending to browser as {fname}')
                # 4. Send CSV to browser for download
                await websocket.send(json.dumps({
                    'type': 'record_stopped',
                    'file': fname,
                    'csv': csv_text,
                    'rows': len(rows),
                }))

            if line and ser_ref and ser_ref.is_open:
                ser_ref.write(line.encode())
                print(f'[serial→] {line.strip()}')
                await websocket.send(json.dumps({'type': 'ack', 'cmd': c}))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)
        print(f'[ws] client disconnected ({len(clients)} total)')


# ── HTTP server (serves index.html from the pc/ directory) ───────────────────
def start_http_server(port: int):
    handler = functools.partial(SimpleHTTPRequestHandler, directory=str(HERE))
    # Silence the per-request log noise
    handler.log_message = lambda *a: None
    httpd = HTTPServer(('localhost', port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()


# ── Entry point ───────────────────────────────────────────────────────────────
async def main_async(args):
    global ser_ref, data_queue
    global _hf_ref, _csv_writer_ref, _csv_file_ref, _loop_ref

    loop       = asyncio.get_event_loop()
    _loop_ref  = loop
    data_queue = asyncio.Queue()

    load_node_names()

    hf = open_hdf5(args.hdf5)
    _hf_ref = hf

    csv_file   = open(args.csv, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(FIELDS)   # header row
    _csv_writer_ref = csv_writer
    _csv_file_ref   = csv_file
    print(f'CSV     → {args.csv}')
    print(f'HDF5    → {args.hdf5}')

    # Try to connect to serial port if provided (non-fatal if it fails)
    if not args.no_serial:
        ok = connect_serial(args.port, args.baud)
        if not ok:
            print(f'[serial] Could not open {args.port} — use the dashboard to select a port')

    start_http_server(args.http_port)

    url = f'http://localhost:{args.http_port}/index.html'
    print(f'Dashboard → {url}')
    webbrowser.open(url)

    async with websockets.serve(handle_client, 'localhost', args.ws_port):
        await asyncio.gather(
            queue_broadcaster(data_queue),
            node_status_task(),
        )


def main():
    parser = argparse.ArgumentParser(description='IMU WebSocket bridge')
    parser.add_argument('--port',      default=DEFAULT_PORT)
    parser.add_argument('--baud',      type=int, default=DEFAULT_BAUD)
    parser.add_argument('--hdf5',      default=DEFAULT_HDF5)
    parser.add_argument('--csv',       default=DEFAULT_CSV)
    parser.add_argument('--ws-port',   type=int, default=DEFAULT_WS_PORT)
    parser.add_argument('--http-port', type=int, default=DEFAULT_HTTP_PORT)
    parser.add_argument('--no-serial', action='store_true',
                        help='Start dashboard without serial connection (UI-only mode)')
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print('\nBridge stopped.')


if __name__ == '__main__':
    main()
