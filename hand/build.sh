#!/usr/bin/env bash
# Builds the DG5F ROS 2 workspace (4 packages) into hand/ws/install. Run once, and again after editing sources.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
# ROS 2 Humble's libraries need the libstdc++ of GCC 12. If g++-12 is installed, use it (the stock Ubuntu 22.04
# g++ 11.4 also works; a PPA g++ 11.5 with its own older libstdc++ does not).
if command -v g++-12 >/dev/null; then export CC=gcc-12 CXX=g++-12; fi
cd "$HERE/ws"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Done. Source it with: source $HERE/ws/install/setup.bash"
