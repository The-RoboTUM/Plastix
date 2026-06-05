# bot_localization

EKF, odometría de ruedas, laser odom, SLAM, AMCL y **RViz** (un solo launch).

Nav2 → **`bot_planning`**.

## Simulación (terminal 2)

Con Gazebo ya en marcha:

```bash
source install/setup.bash
ros2 launch bot_localization localization.launch.py
```

Por defecto: mapa (`arena_map.yaml`), AMCL, laser odom, EKF y RViz con `/map`.

La pose inicial AMCL (`x=2`, `y=0`) debe coincidir con el spawn de sim (`simulation.launch.py`). Si el robot se ve **fuera del mapa**, alinea ambos o usa en RViz **2D Pose Estimate**.

RViz va en el mismo launch para ver mapa, láser, pose y mandar **2D Goal** a Nav2 (T3). Sin ventana: `use_rviz:=false`.

## Robot real

Igual con `use_sim_time:=false`.

## Modos

| Modo | Flags |
|------|--------|
| Mapa + AMCL | `enable_saved_map_localization:=true` |
| SLAM | `enable_slam:=true`, `enable_saved_map_localization:=false` |
| Solo EKF | ambos `false` |
