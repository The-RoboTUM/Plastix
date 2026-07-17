# gripperx_localization/maps

Occupancy grid maps (`.pgm` + `.yaml`) generated via `slam_toolbox`/`map_saver_cli`
against the Gazebo simulation (`gripperx_gazebo`).
Referenced by `launch/localization.launch.py` (`map_yaml_file`, default
`testworld_v1_map.yaml`) for `enable_saved_map_localization:=true` (AMCL).

This folder is deliberately **package-local** (distinct from the top-level
`Software/ros2/maps/`, which contains the real robot's maps used by
`gripperx_bringup`). `setup.py` installs everything located here under
`*.yaml`/`*.pgm` to `share/gripperx_localization/maps/`.
