#!/usr/bin/env bash
# Rebuilds impedance_controller from arm/src and copies the binary next to this script.
# Needs: cmake, g++, libzmq3-dev, libeigen3-dev, libboost-all-dev, python3-dev, ur_rtde (see README).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/src/build"
cd "$HERE/src/build"
# Prefer the apt (librtde-dev) cmake config over a stale /usr/local one left by an old source install.
EXTRA=()
APT_RTDE=/usr/lib/$(gcc -dumpmachine)/cmake/ur_rtde
[ -d "$APT_RTDE" ] && EXTRA+=(-Dur_rtde_DIR="$APT_RTDE")
cmake .. -DCMAKE_BUILD_TYPE=Release "${EXTRA[@]}"
make -j"$(nproc)" impedance_controller
cp impedance_controller "$HERE/impedance_controller"
echo "Built $HERE/impedance_controller"
