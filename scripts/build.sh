#!/usr/bin/env bash
# Build the metro simulation workspace.  On the target image ROS 2 Humble is
# found automatically; a newer local distribution is used only for validation.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros_env.sh"

DISTRO="$(resolve_ros_distro)"
# shellcheck disable=SC1090
source "/opt/ros/${DISTRO}/setup.bash"
WS_DIR="$(cd "${SCRIPT_DIR}/../ws_metro" && pwd)"
cd "${WS_DIR}"

# colcon/CMake caches contain absolute source paths.  If the workspace was
# copied or renamed, a stale cache points at the old directory and CMake aborts.
shopt -s nullglob
for cache in "${WS_DIR}"/build/*/CMakeCache.txt; do
  recorded="$(sed -n 's/^# For build in directory: //p' "${cache}" | head -n 1)"
  if [[ -n "${recorded}" && "${recorded}" != "${WS_DIR}/"* ]]; then
    echo "stale build cache detected (${recorded}); cleaning build/install/log"
    rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
    break
  fi
done
shopt -u nullglob

CMAKE_ARGS=()
# Local validation containers may expose an old Windows/Anaconda protobuf
# include directory before /usr/include.  It is unrelated to the ROS/Gazebo
# target; pin the system protobuf headers when that directory is present.
if [[ -f /mnt/d/Anaconda/Library/include/google/protobuf/port_def.inc ]]; then
  mkdir -p /tmp/metro_sysproto/google
  ln -sfn /usr/include/google/protobuf /tmp/metro_sysproto/google/protobuf
  CMAKE_ARGS+=("-DCMAKE_CXX_FLAGS=-I/tmp/metro_sysproto")
  echo "note: local Anaconda protobuf detected, using system headers for validation"
fi

if [[ ${#CMAKE_ARGS[@]} -gt 0 ]]; then
  colcon build --symlink-install \
    --packages-select metro_description metro_gazebo metro_application metro_bringup \
    --cmake-args "${CMAKE_ARGS[@]}" \
    --event-handlers console_direct+
else
  colcon build --symlink-install \
    --packages-select metro_description metro_gazebo metro_application metro_bringup \
    --event-handlers console_direct+
fi

echo
echo "Build finished with ROS 2 ${DISTRO}. Source ${WS_DIR}/install/setup.bash before launching."
