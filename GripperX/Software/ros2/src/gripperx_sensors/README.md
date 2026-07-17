# gripperx_sensors

Publishes sensor topics expected by `gripperx_localization`:

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
