#!/usr/bin/env bash
# Creates calibration/.venv with OpenCV (contrib), pyrealsense2, scipy and pyzmq. Run once per PC.
# Needs python3-venv. pyrealsense2 2.53 ships x86-64 wheels for Python 3.7-3.11 (Ubuntu 22.04 has 3.10).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -m venv "$HERE/.venv"
"$HERE/.venv/bin/pip" install --upgrade pip
"$HERE/.venv/bin/pip" install -r "$HERE/requirements.txt"
"$HERE/.venv/bin/python" -c "import cv2, cv2.aruco, pyrealsense2, scipy, zmq; print('OpenCV', cv2.__version__)"
echo "Done. Run the calibration with: $HERE/run_calibration.sh --name ur5e --serial_number <camera serial>"
