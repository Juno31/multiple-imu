#!/usr/bin/env python3
"""
IMU receiver: reads 53-byte framed packets from master USB CDC,
verifies CRC8, decodes imu_packet_t, appends to HDF5.

Frame format: [0xAA][0x55][50 bytes imu_packet_t][CRC8] = 53 bytes total
imu_packet_t: node_id(u8) calib(u8) timestamp_us(u32) qi qj qk qr quat_acc ax ay az gx gy gz (11×f32)
Struct fmt:   '<BBIfffffffffff'  → 1+1+4+44 = 50 bytes ✓
"""

import argparse
import struct
import time
import serial
import h5py
import numpy as np

MAGIC0      = 0xAA
MAGIC1      = 0x55
PAYLOAD_LEN = 50
FRAME_TOTAL = 53        # 2 magic + 50 payload + 1 CRC
STRUCT_FMT  = '<BBIfffffffffff'   # 50 bytes
CRC8_POLY   = 0x07


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC8_POLY) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def open_hdf5(path: str):
    f = h5py.File(path, 'a', libver='latest')
    if 'imu' not in f:
        dt = np.dtype([
            ('node_id',      np.uint8),
            ('calib',        np.uint8),
            ('timestamp_us', np.uint32),
            ('qi',           np.float32),
            ('qj',           np.float32),
            ('qk',           np.float32),
            ('qr',           np.float32),
            ('quat_acc',     np.float32),
            ('ax',           np.float32),
            ('ay',           np.float32),
            ('az',           np.float32),
            ('gx',           np.float32),
            ('gy',           np.float32),
            ('gz',           np.float32),
        ])
        f.create_dataset('imu', shape=(0,), maxshape=(None,), dtype=dt, chunks=True)
    f.swmr_mode = True
    return f


def sync_frame(ser: serial.Serial):
    """Scan for [0xAA][0x55] magic then read payload+CRC."""
    b = ser.read(1)
    if not b or b[0] != MAGIC0:
        return None
    b = ser.read(1)
    if not b or b[0] != MAGIC1:
        return None
    rest = ser.read(FRAME_TOTAL - 2)   # 51 bytes: 50 payload + 1 CRC
    if len(rest) != FRAME_TOTAL - 2:
        return None
    return rest


def main():
    parser = argparse.ArgumentParser(description='IMU packet receiver')
    parser.add_argument('--port', default='/dev/cu.usbmodem101')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--hdf5', default='imu_data.h5')
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud → {args.hdf5}")
    print("Ctrl+C to stop\n")

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    hf  = open_hdf5(args.hdf5)
    ds  = hf['imu']

    total = 0
    bad   = 0
    last_print = time.time()
    last_pkt   = None

    try:
        while True:
            rest = sync_frame(ser)
            if rest is None:
                continue

            payload = rest[:PAYLOAD_LEN]
            crc_rx  = rest[PAYLOAD_LEN]

            if crc8(payload) != crc_rx:
                bad += 1
                continue

            try:
                vals = struct.unpack(STRUCT_FMT, payload)
            except struct.error:
                bad += 1
                continue

            row = np.array([vals], dtype=ds.dtype)
            ds.resize(ds.shape[0] + 1, axis=0)
            ds[-1] = row
            total  += 1
            last_pkt = vals

            now = time.time()
            if now - last_print >= 0.5:
                hf.flush()
                v = last_pkt
                print(f"[node={v[0]} calib=0x{v[1]:02X} t={v[2]/1e6:.3f}s] "
                      f"q=({v[3]:.3f},{v[4]:.3f},{v[5]:.3f},{v[6]:.3f}) "
                      f"acc=({v[8]:.2f},{v[9]:.2f},{v[10]:.2f}) m/s2 "
                      f"gyro=({v[11]:.2f},{v[12]:.2f},{v[13]:.2f}) rad/s "
                      f"| rx={total} bad={bad}")
                last_print = now

    except KeyboardInterrupt:
        print(f"\nDone. {total} packets → {args.hdf5}, {bad} bad frames.")
    finally:
        hf.close()
        ser.close()


if __name__ == '__main__':
    main()
