# Source this in every terminal that talks to the hand:  source hand/env.sh
HAND_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HAND_DIR/ws/install/setup.bash"
