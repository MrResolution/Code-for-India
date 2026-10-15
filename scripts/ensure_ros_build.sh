#!/usr/bin/env bash
# Sourced by launch_dashboard.sh and launch_web_dashboard.sh once ROS 2 is
# sourced:   ensure_ros_build <repo root>
#
# A plain `colcon build` copies servo_joint_publisher into ros2_ws/install, and
# `ros2 run` / `ros2 launch` start that copy. After a `git pull` the launchers
# would keep running the old code: a desktop app without the newest tabs, an
# old servo_serial_commander. This rebuilds the package when the installed
# copy differs from the checkout. A `--symlink-install` build already points
# at the checkout, so it is never rebuilt here.
ensure_ros_build() {
  local ws="$1/ros2_ws"
  local src="$ws/src/servo_joint_publisher/servo_joint_publisher"
  local installed=""
  if [ -f "$ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ws/install/setup.bash"
    # From / so a checkout in the current directory can't stand in for the install.
    installed="$(cd / && python3 -c 'import os, servo_joint_publisher as p; print(os.path.dirname(os.path.realpath(p.__file__)))' 2>/dev/null || true)"
  fi
  if [ -n "$installed" ]; then
    if [ "$installed" = "$(realpath "$src")" ] || diff -rq -x __pycache__ "$src" "$installed" >/dev/null 2>&1; then
      return 0
    fi
  fi

  if ! command -v colcon >/dev/null 2>&1; then
    echo "⚠️  ros2_ws/install is missing or older than the checkout, and colcon is not installed:"
    echo "    the dashboard may run old code. See docs/QUICK_INSTALL_GUIDE.md."
    return 0
  fi
  # Keep the workspace's build mode: colcon can't switch an existing build to symlinks.
  local mode=()
  [ -d "$ws/build/servo_joint_publisher" ] || mode=(--symlink-install)
  echo "🔧 ros2_ws/install is missing or older than the checkout: rebuilding servo_joint_publisher ..."
  if ! (cd "$ws" && colcon build --packages-select servo_joint_publisher "${mode[@]}"); then
    echo "❌ colcon build failed (see above). Fix it, or rebuild by hand: cd ros2_ws && colcon build" >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  source "$ws/install/setup.bash"
}
