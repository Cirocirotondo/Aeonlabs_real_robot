#!/usr/bin/env bash
# Terminal 2: ZMQ <-> ROS 2 bridge (state on 5563, commands on 5562). Start it after the driver.
# Uses the system python3 (the one ROS 2 Humble is built for), not a venv.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec python3 "$(dirname "${BASH_SOURCE[0]}")/dg5f_policy_ros_bridge.py" "$@"
