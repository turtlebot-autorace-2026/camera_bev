#!/usr/bin/env bash
set -eo pipefail
BEV_PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BEV_WS="$(cd -- "$BEV_PACKAGE_DIR/../.." && pwd)"
BEV_DISTRO="${BEV_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [ -z "$BEV_DISTRO" ]; then
  if [ -f /opt/ros/galactic/setup.bash ]; then BEV_DISTRO=galactic; else BEV_DISTRO=humble; fi
fi
source "/opt/ros/$BEV_DISTRO/setup.bash"
source "$BEV_WS/install_bev/local_setup.bash"
exec python3 "$BEV_PACKAGE_DIR/tools/preview.py" "$@"
