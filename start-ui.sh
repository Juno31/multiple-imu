#!/usr/bin/env bash
# Start the IMU WebSocket bridge + dashboard
# Usage:
#   ./start-ui.sh                        # auto-detect or select port in UI
#   ./start-ui.sh /dev/cu.usbmodem1101   # connect to specific port

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR/pc" || exit 1

if [ -n "$1" ]; then
  exec python3 bridge.py --port "$1"
else
  exec python3 bridge.py --no-serial
fi
