# bot_bringup

**Solo robot real.**

```bash
source install/setup.bash
ros2 launch bot_bringup bringup.launch.py
```

URDF + `bot_control` (`use_sim_time:=false`). Sin Gazebo, sin EKF, sin Nav2.

Simulación → `bot_gazebo`. Localización → `bot_localization`. Navegación → `bot_planning`.
