#!/usr/bin/env bash
# Shared ROS environment resolution.  The target stack is Humble; a newer
# development container (e.g. Lyrical) is used only for local validation.
set -eo pipefail

resolve_ros_distro() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "${ROS_DISTRO}"
    return 0
  fi
  for candidate in humble lyrical jazzy; do
    if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  echo "No ROS 2 distribution found under /opt/ros" >&2
  return 1
}

ws_dir() {
  cd "$(dirname "${BASH_SOURCE[0]}")/../ws_metro" >/dev/null 2>&1
  pwd
}
