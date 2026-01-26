# msg/DevicePosition.msg
# Position of a device in local coordinate system

string device_id           # e.g., "drone_1", "robot_2"
string device_type         # "drone", "robot", or "laptop"
float64 x                  # X position in meters (East-West)
float64 y                  # Y position in meters (North-South)
float64 z                  # Altitude in meters
float64 latitude           # GPS latitude
float64 longitude          # GPS longitude
float64 distance_from_base # Distance from laptop in meters
float64 bearing            # Bearing from laptop in degrees
builtin_interfaces/Time timestamp

---

# msg/DeviceStatus.msg
# Status information for a device

string device_id
string device_type
float32 battery_percent
string state              # "active", "charging", "idle"
float64 distance_from_base
bool is_connected
builtin_interfaces/Time timestamp

---

# msg/FleetStatus.msg
# Overall fleet status

uint32 total_devices
uint32 active_robots
uint32 active_drones
uint32 charging_devices
uint32 idle_devices
float64 average_battery
builtin_interfaces/Time server_start_time
builtin_interfaces/Time timestamp