#!/usr/bin/env bash
# Camera + robot base calibration. Start ./arm/run_controller.sh first; run ./calibration/setup.sh once per PC.
# Args go to robot_base_calibration.py, e.g.  ./calibration/run_calibration.sh --name ur5e --serial_number 138322253845
#   ./calibration/run_calibration.sh --list   prints the serials of the connected RealSense cameras
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$HERE/.venv/bin/python"
if [ ! -x "$PY" ]; then
    echo "No $PY: run $HERE/setup.sh first." >&2
    exit 1
fi
if [ "${1:-}" = "--list" ]; then
    exec "$PY" -c "
import pyrealsense2 as rs
devices = rs.context().devices
for d in devices:
    print(d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name))
if len(devices) == 0:
    print('No RealSense camera found (USB 3 port? udev rules? see calibration/README.md)')
"
fi
exec "$PY" "$HERE/robot_base_calibration.py" "$@"
