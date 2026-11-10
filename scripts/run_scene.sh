#!/usr/bin/env bash
# Full enriched scene: station, 12 PSD, passengers, scenario, camera and RViz.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros_env.sh"

DISTRO="$(resolve_ros_distro)"
# shellcheck disable=SC1090
source "/opt/ros/${DISTRO}/setup.bash"
WS_DIR="$(cd "${SCRIPT_DIR}/../ws_metro" && pwd)"
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"

exec ros2 launch metro_bringup metro_scene.launch.py "$@"
