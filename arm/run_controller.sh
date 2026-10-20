#!/usr/bin/env bash
# Terminal 3: low-level UR5e controller (ZMQ commands on 5555, state on 5556). UR in Remote Control, PC on 192.168.1.10.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
exec ./impedance_controller "${1:-pc_ur_new.json}"
