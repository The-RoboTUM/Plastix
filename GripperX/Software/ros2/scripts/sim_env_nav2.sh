#!/bin/bash
# Layers the locally-extracted Nav2 debs (.rosdeps_local, gitignored, 194MB)
# on top of the standard sim_env.sh, so the Nav2 core packages missing from the
# system install become available WITHOUT sudo/system changes. Domain 7 (SR-8).
# Usage: source this file (from ~/gripperx_ws/Software/ros2).
WS="$HOME/gripperx_ws/Software/ros2"
source "$WS/scripts/sim_env.sh"

R="$WS/.rosdeps_local/opt/ros/jazzy"
U="$WS/.rosdeps_local/usr"
if [ -d "$R" ]; then
  # AMENT_PREFIX_PATH: prepend so `ros2 launch` can resolve the Nav2 executables.
  export AMENT_PREFIX_PATH="$R:$AMENT_PREFIX_PATH"
  export PATH="$PATH:$R/bin"
  export PYTHONPATH="$PYTHONPATH:$R/lib/python3.12/site-packages"
  # LD_LIBRARY_PATH is deliberately NOT modified here: the .rosdeps_local Nav2
  # debs were built against a NEWER diagnostic_updater ABI than this laptop's
  # system ROS. Prepending it globally kills gz sim / controller_manager
  # (system, older ABI); appending it kills the Nav2 lifecycle_manager (needs
  # the newer ABI). So only the Nav2 C++ nodes get the .rosdeps_local libs
  # prepended, per-process, via additional_env in the launch files, keyed off
  # this variable. gz sim, controllers and slam_toolbox keep the system libs.
  export GRIPPERX_ROSDEPS_LIB="$R/lib:$U/lib/x86_64-linux-gnu"

  # DEFENSIVE SANITIZE (fixes the "gz GUI opens then closes" bringup failure):
  # ros_gz_sim/gz_sim.launch.py copies the launch process's LD_LIBRARY_PATH into
  # GZ_SIM_SYSTEM_PLUGIN_PATH, so if ANY .rosdeps_local entry is present in
  # LD_LIBRARY_PATH (e.g. left over in the shell from an earlier env attempt),
  # the gz sim process loads the newer-ABI libdiagnostic_updater.so from
  # .rosdeps_local and the system libcontroller_manager.so fails its symbol
  # lookup -> gz sim crashes -> on_exit_shutdown tears down the whole bringup.
  # Strip every .rosdeps_local entry from LD_LIBRARY_PATH and
  # GZ_SIM_SYSTEM_PLUGIN_PATH so gz always sees the system libs, regardless of
  # prior shell state. The Nav2 C++ nodes still get .rosdeps_local via the
  # per-process additional_env (GRIPPERX_ROSDEPS_LIB) in the launch files.
  _gx_strip_rosdeps() {  # $1 = variable name
    local val="${!1}" out="" p IFS=:
    for p in $val; do
      case "$p" in *".rosdeps_local"*|"") ;; *) out="${out:+$out:}$p" ;; esac
    done
    printf -v "$1" '%s' "$out"; export "$1"
  }
  _gx_strip_rosdeps LD_LIBRARY_PATH
  [ -n "$GZ_SIM_SYSTEM_PLUGIN_PATH" ] && _gx_strip_rosdeps GZ_SIM_SYSTEM_PLUGIN_PATH
  unset -f _gx_strip_rosdeps

  echo "[sim_env_nav2] GRIPPERX_ROSDEPS_LIB set (per-process Nav2 lib prepend)"
  echo "[sim_env_nav2] .rosdeps_local stripped from LD_LIBRARY_PATH (gz sim safe)"
  echo "[sim_env_nav2] layered .rosdeps_local Nav2 (AMENT prepended)"
else
  echo "[sim_env_nav2] WARNING: .rosdeps_local not found"
fi
