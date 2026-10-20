Subset of https://github.com/tesollodelto/dg5f_ros2 (commit 51cec8a, main) — only dg5f_driver and
dg5f_description. dg5f_gz and dg5f_moveit_config were left out: they are not needed to drive the hand.

Local change versus upstream: dg5f_driver/config/dg5f_right_pid_all_controller.yaml has the per-joint
PID gain p raised from 1.5 to 2.0 on all 20 joints (tuned on the real hand). Keep it if you re-sync with upstream.
