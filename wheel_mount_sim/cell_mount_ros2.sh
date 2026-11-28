#!/usr/bin/env bash
# Phase-2 launcher: Isaac Sim with its bundled ROS 2 Jazzy libraries (Python 3.11). Do not source
# /opt/ros/jazzy in this shell (its Python 3.12 rclpy would shadow the bundled one).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
LIB="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
case ":$LD_LIBRARY_PATH:" in *":$LIB:"*) ;; *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$LIB";; esac
unset PYTHONPATH AMENT_PREFIX_PATH
cd "$HERE/.."
exec "$HOME/isaacsim/python.sh" "$HERE/cell_mount_ros2.py" "$@"
