# bot_gazebo

Gazebo Sim: mundo, spawn, `ros2_control`, bridge de sensores, `bot_control`.

**No** lanza localización, AMCL, EKF ni Nav2 — eso va en `bot_localization` y `bot_planning`.

## Prerequisites (Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/install_jazzy_sim_deps.sh
```

## Terminal 1 — solo simulación

```bash
source install/setup.bash
ros2 launch bot_gazebo simulation.launch.py
```

Opcional: `use_lidar:=false`, `use_camera:=false`, `use_rviz:=true`.

## Siguiente (otras terminales)

| Terminal | Paquete | Launch |
|----------|---------|--------|
| 2 | `bot_localization` | `localization.launch.py` (+ RViz por defecto) |
| 3 | `bot_planning` | `navigation.launch.py` |

Ver README del repo y `bot_localization/README.md`.
