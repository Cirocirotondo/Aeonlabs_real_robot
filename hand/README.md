# Hand: Tesollo DG5F (right) — ROS 2 driver + ZMQ bridge

Everything needed to drive the hand lives in this folder:

| Path | What it is |
|---|---|
| `ws/src/dg_common/` | Tesollo's `dg_hardware` and `dg_tcp_comm` packages (ros2_control hardware plugin + TCP protocol) |
| `ws/src/dg5f_ros2/` | `dg5f_driver` (launch files, URDF, controller config) and `dg5f_description` (meshes, xacro macro) |
| `dg5f_policy_ros_bridge.py` | ZMQ <-> ROS 2 bridge: publishes joint state on port 5563, takes targets on port 5562 |
| `build.sh` | builds the workspace into `ws/install` |
| `env.sh` | sources ROS 2 Humble and the workspace (use it in every hand terminal) |
| `run_driver.sh`, `run_bridge.sh` | the two terminals you need, in that order |

The driver config `ws/src/dg5f_ros2/dg5f_driver/config/dg5f_right_pid_all_controller.yaml` carries a local
tuning (PID gain p = 2.0 on all joints instead of upstream's 1.5). The sources are otherwise upstream Tesollo
code; see the README files inside `ws/src` for the exact commits.

## 1. Install (once per PC)

Tested on Ubuntu 22.04 with ROS 2 Humble. Other combinations are untested.

```bash
# ROS 2 Humble (skip if already installed): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-humble-ros-base python3-colcon-common-extensions python3-rosdep

# packages the four workspace packages build and run against
sudo apt install \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-hardware-interface ros-humble-pluginlib ros-humble-control-msgs \
  ros-humble-joint-state-broadcaster ros-humble-pid-controller \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-joint-state-publisher \
  libboost-all-dev python3-numpy

# optional: only for `ros2 launch dg5f_description ... rviz`
sudo apt install ros-humble-rviz2 ros-humble-joint-state-publisher-gui
```

If something is still missing, let rosdep tell you:

```bash
sudo rosdep init 2>/dev/null; rosdep update
cd hand/ws && rosdep install --from-paths src --ignore-src -y
```

## 2. Build (once, and after editing sources)

```bash
./hand/build.sh
```

This creates `ws/build`, `ws/install`, `ws/log`. They are machine-specific: do not copy them to another PC, rebuild there.

## 3. Hardware and network

- Power the hand from the bench supply: 24 V, current limit 10 A.
- Connect the hand's Ethernet cable to the PC. The hand answers at **169.254.186.72**, port 502 (launch arguments
  `delto_ip` / `delto_port`).
- Give the PC a static address on that link. With NetworkManager:

```bash
sudo nmcli con add type ethernet ifname <iface> con-name tesollo ipv4.method manual ipv4.addresses 169.254.186.0/24
sudo nmcli con up tesollo
ping -c 2 169.254.186.72
```

(`<iface>` is the name from `ip link`, e.g. `enx...` for a USB adapter.)

## 4. Run

Terminal 1, driver:

```bash
./hand/run_driver.sh                     # default IP 169.254.186.72
./hand/run_driver.sh delto_ip:=169.254.186.73   # if the hand has another address
```

Terminal 2, bridge (wait until the driver prints that `rj_dg_pospid` and `joint_state_broadcaster` are active):

```bash
./hand/run_bridge.sh
```

The bridge streams `/dg5f_right/joint_states` to ZMQ port 5563 and forwards targets from port 5562 to
`/dg5f_right/rj_dg_pospid/reference` at 100 Hz. If no target arrives for 0.25 s it holds the measured position.
Useful options: `--state-port`, `--command-port`, `--max-command-step-rad` (0.12), `--controller-mode trajectory`
(old spline controller fallback). `./hand/run_bridge.sh --help` lists them all.

Doing it by hand instead of the scripts:

```bash
source hand/env.sh
ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py
# second terminal
source hand/env.sh
python3 hand/dg5f_policy_ros_bridge.py
```

Do not run the MANUS teleop or anything else that commands the hand or binds ports 5562 / 5563 at the same time.

## Checks

```bash
source hand/env.sh
ros2 control list_controllers          # rj_dg_pospid and joint_state_broadcaster must be "active"
ros2 topic hz /dg5f_right/joint_states # ~100 Hz
```
