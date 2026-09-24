# gripperx_sensors

Two executables: `sensor_mocks` (bench stand-ins) and **`scan_range_filter`**
(the real scan chain).

## The scan chain — read this first

```
LiDAR driver (or ros_gz_bridge)  ->  /scan_raw  ->  scan_range_filter  ->  /scan
```

**The LiDAR driver does not publish `/scan`.** `scan_range_filter` republishes it
with the robot's own returns removed (`min_range` 0.10 m). The same filter runs on
both platforms, and it is launched from `gripperx_bringup/real_robot.launch.py`
and `gripperx_gazebo/simulate_robot.launch.py` — **not** from this package's
`sensors.launch.py`, which starts only the mocks.

If the filter is bypassed or misconfigured, the robot maps itself: self-returns
appear as an obstacle ring at the chassis radius and SLAM/AMCL degrade.

## Topics

Consumed by `gripperx_localization` and Nav2:

| Topic | Type | Used by |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | laser odom, SLAM, AMCL, Nav2 |
| `/imu/data` | `sensor_msgs/Imu` | EKF (via `localization_input_node`) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | optional GPS fusion (`enable_gps`) |

## Launch

```bash
# Mock sensors (bench)
ros2 launch gripperx_sensors sensors.launch.py

# Real hardware — launch your drivers, then:
ros2 launch gripperx_sensors sensors.launch.py use_mock_sensors:=false
```

Replace mocks with your driver nodes (RPLidar, ublox, microstrain, etc.) publishing the same topic names.
