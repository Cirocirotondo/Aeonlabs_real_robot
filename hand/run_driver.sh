#!/usr/bin/env bash
# Terminal 1: DG5F driver (ros2_control + per-joint position PID). Hand powered (24 V) and on the 169.254.186.x link.
# Extra args are passed to ros2 launch, e.g.  ./run_driver.sh delto_ip:=169.254.186.72
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py "$@"
