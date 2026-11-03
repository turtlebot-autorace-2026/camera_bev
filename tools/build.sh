#!/usr/bin/env bash
set -eo pipefail
BEV_PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BEV_WS="$(cd -- "$BEV_PACKAGE_DIR/../.." && pwd)"
BEV_DISTRO="${BEV_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [ -z "$BEV_DISTRO" ]; then
  if [ -f /opt/ros/galactic/setup.bash ]; then BEV_DISTRO=galactic; else BEV_DISTRO=humble; fi
fi
source "/opt/ros/$BEV_DISTRO/setup.bash"
cd "$BEV_WS"
colcon --log-base log_bev build --base-paths "$BEV_PACKAGE_DIR" --packages-select camera_bev \
  --build-base build_bev --install-base install_bev --symlink-install
