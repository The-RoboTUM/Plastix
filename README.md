# Robot Bringup Package

This repository contains ROS2 packages for robot simulation, control, and navigation.

## Prerequisites

- ROS2 (Humble or later)
- Gazebo Simulator
- Required ROS2 packages:
  - `teleop_twist_keyboard`
  - `controller_manager`
  - `ros_gz_sim`
  - `ros_gz_bridge`
  - `rviz2`

## Building the Workspace

```bash
cd /home/ce/pi/src
colcon build
source install/setup.bash
```

## Running the Code

### Launch the Robot Bringup

To start the robot simulation with Gazebo, controllers, and all necessary nodes:

```bash
ros2 launch robot_bringup bringUp.launch.py
```

This launch file will:
- Start Gazebo simulator with the robot model
- Launch the robot controller
- Start the joint state broadcaster
- Initialize the skid steering velocity controller
- Set up the ROS-Gazebo bridge for sensor data

### Teleop Control

In a separate terminal, run the keyboard teleop to control the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q` - Increase linear speed
- `z` - Decrease linear speed
- `w` - Increase angular speed
- `x` - Decrease angular speed

### Navigation with Nav2

To start the complete navigation stack with mapping, localization, and path planning:

```bash
ros2 launch robot_mapping nav2.launch.py
```

This launch file will:
- Start Nav2 navigation stack
- Launch AMCL localization
- Initialize map server with the stored map
- Start controller and planner servers
- Launch RViz with navigation configuration
- Enable global and local costmaps

**Setting Goals in RViz:**

1. In RViz, make sure the **2D Goal Pose** tool is selected (toolbar button)
2. Click on the map to set a navigation goal
3. Drag the arrow to set the desired orientation
4. The robot will plan and execute a path to the goal
5. Monitor the path planning in the **Plan** display
6. Check **Global Costmap** and **Local Costmap** displays to see obstacle avoidance

**Navigation Features:**
- Automatic path planning with obstacle avoidance
- Local and global costmaps for dynamic obstacle detection
- AMCL localization for accurate robot pose estimation
- Recovery behaviors when stuck or blocked

## ROS2 Topics

### Command Topics

- **`/cmd_vel`** (`geometry_msgs/msg/Twist`)
  - Velocity commands for the robot
  - Published by: `teleop_twist_keyboard`
  - Subscribed by: `robot_controller`

### Sensor Topics

- **`/scan`** (`sensor_msgs/msg/LaserScan`)
  - Laser scan data from the robot's lidar sensor
  - Published by: Gazebo bridge
  - Subscribed by: Mapping nodes

- **`/imu/out`** (`sensor_msgs/msg/Imu`)
  - IMU sensor data
  - Published by: Gazebo bridge

- **`/joint_states`** (`sensor_msgs/msg/JointState`)
  - Joint positions and velocities
  - Published by: `joint_state_broadcaster`
  - Subscribed by: `robot_controller`

### Odometry Topics

- **`/odom`** (`nav_msgs/msg/Odometry`)
  - Robot odometry information (position, orientation, velocities)
  - Published by: `robot_controller`
  - Frame: `odom` → `base_link`

### Control Topics

- **`/kid_steering_velocity_controller/commands`** (`std_msgs/msg/Float64MultiArray`)
  - Wheel velocity commands for the skid steering controller
  - Published by: `robot_controller`
  - Subscribed by: Gazebo controller

### Mapping Topics

- **`/map`** (`nav_msgs/msg/OccupancyGrid`)
  - Occupancy grid map
  - Published by: `map_server`

- **`/map_updates`** (`map_msgs/msg/OccupancyGridUpdate`)
  - Map update messages
  - Published by: `map_server`

- **`/odometry_motion_model/samples`** (`geometry_msgs/msg/PoseArray`)
  - Motion model samples for localization
  - Published by: `odometry_motion_model` node

### Navigation Topics

- **`/goal_pose`** (`geometry_msgs/msg/PoseStamped`)
  - Navigation goal pose
  - Published by: RViz (when using 2D Goal Pose tool)
  - Subscribed by: `bt_navigator`

- **`/amcl_pose`** (`geometry_msgs/msg/PoseWithCovarianceStamped`)
  - Localized robot pose with covariance
  - Published by: `amcl`
  - Frame: `map`

- **`/initialpose`** (`geometry_msgs/msg/PoseWithCovarianceStamped`)
  - Initial pose estimate for localization
  - Published by: RViz (when using 2D Pose Estimate tool)
  - Subscribed by: `amcl`

- **`/plan`** (`nav_msgs/msg/Path`)
  - Planned path to goal
  - Published by: `planner_server`

- **`/local_costmap/costmap`** (`nav_msgs/msg/OccupancyGrid`)
  - Local costmap for obstacle avoidance
  - Published by: `controller_server`
  - Frame: `odom`

- **`/global_costmap/costmap`** (`nav_msgs/msg/OccupancyGrid`)
  - Global costmap for path planning
  - Published by: `planner_server`
  - Frame: `map`

### System Topics

- **`/clock`** (`rosgraph_msgs/msg/Clock`)
  - Simulation clock (when using simulation time)
  - Published by: Gazebo

- **`/tf`** (`tf2_msgs/msg/TFMessage`)
  - Transform tree messages
  - Published by: Various nodes (robot_state_publisher, controllers)

## Launch Files

### Main Launch Files

- **`robot_bringup/bringUp.launch.py`**
  - Main launch file that starts the complete system
  - Includes: Gazebo simulation, robot controller, and visualization

- **`robot_description/gazebo.launch.py`**
  - Launches Gazebo simulator with robot model
  - Spawns robot in the simulation world
  - Sets up ROS-Gazebo bridge

- **`robot_controller/controller.launch.py`**
  - Launches robot controllers
  - Starts joint state broadcaster
  - Initializes skid steering velocity controller

### Additional Launch Files

- **`robot_description/display.launch.py`** - Robot visualization in RViz
- **`robot_mapping/slam.launch.py`** - SLAM mapping
- **`robot_mapping/nav2.launch.py`** - Navigation stack
- **`robot_localization/global_localization.launch.py`** - Localization

## Package Structure

- **`robot_bringup`** - Main launch files and bringup configuration
- **`robot_description`** - URDF models, meshes, and world files
- **`robot_controller`** - Robot control nodes and controllers
- **`robot_mapping`** - SLAM and navigation packages
- **`robot_localization`** - Localization and motion models

## Usage Examples

### Basic Robot Control

1. **Terminal 1 - Start the simulation:**
   ```bash
   ros2 launch robot_bringup bringUp.launch.py
   ```

2. **Terminal 2 - Control the robot:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Terminal 3 - Monitor topics (optional):**
   ```bash
   ros2 topic list
   ros2 topic echo /odom
   ros2 topic echo /scan
   ```

### Autonomous Navigation

1. **Terminal 1 - Start the simulation:**
   ```bash
   ros2 launch robot_bringup bringUp.launch.py
   ```

2. **Terminal 2 - Start navigation:**
   ```bash
   ros2 launch robot_mapping nav2.launch.py
   ```

3. **In RViz:**
   - First, set the initial pose using **2D Pose Estimate** tool
   - Then, set navigation goals using **2D Goal Pose** tool
   - The robot will automatically plan and execute paths

### Verifying Navigation Setup

Check if all navigation nodes are active:
```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /amcl
```

Check costmaps are publishing:
```bash
ros2 topic info /local_costmap/costmap
ros2 topic info /global_costmap/costmap
```

## Configuration

- Robot parameters can be configured in:
  - `robot_controller/config/controller.yaml`
  - `robot_mapping/config/slam.yaml`
  - `robot_mapping/config/nav_params.yaml`
  - `robot_localization/config/amcl.yaml`

## Media

<!--### Images-->

<!-- Add robot images here -->
<!-- Example: ![Robot in simulation](images/robot_simulation.png) -->
<!-- Example: ![Navigation in RViz](images/nav2_rviz.png) -->
<!-- Example: ![Map and costmaps](images/costmaps.png) -->



### Videos

<!-- Add robot videos here -->
<!-- Example: [![Robot Navigation Demo](images/video_thumbnail.png)](videos/navigation_demo.mp4) -->
<!-- Example: [Navigation Demo](videos/navigation_demo.mp4) - Robot autonomously navigating to goals -->

*Teleoperation robot.*

![Robot Autonomous Navigation Demo](imagesandvideos/teleoperationrobot.webm)

*Nav2 for set goal position.*

![Robot Autonomous Navigation Demo](imagesandvideos/nav2video.webm)

## Troubleshooting

### General Issues

- **Robot doesn't move:**
  - Check that `/cmd_vel` topic is being published: `ros2 topic info /cmd_vel`
  - Verify controllers are running: `ros2 control list_controllers`
  - Check controller state: `ros2 lifecycle get /controller_server`

- **Simulation time issues:**
  - Ensure simulation time is set correctly if using Gazebo
  - Check `/clock` topic: `ros2 topic echo /clock`

### Navigation Issues

- **Map not displaying in RViz:**
  - Verify map_server is active: `ros2 lifecycle get /map_server`
  - Check map topic: `ros2 topic info /map`
  - Verify map file path in `nav_params.yaml`

- **Costmaps not showing:**
  - Ensure controller_server and planner_server are active
  - Check costmap topics: `ros2 topic list | grep costmap`
  - Verify AMCL is localizing: `ros2 topic echo /amcl_pose`

- **Robot not reaching goals:**
  - Set initial pose in RViz using **2D Pose Estimate** tool
  - Check that AMCL has converged (particle cloud should be focused)
  - Verify `/plan` topic shows planned paths: `ros2 topic echo /plan`
  - Check controller_server is active: `ros2 lifecycle get /controller_server`

- **Path planning fails:**
  - Ensure map_server is active: `ros2 lifecycle get /map_server`
  - Check planner_server state: `ros2 lifecycle get /planner_server`
  - Verify global_costmap is publishing: `ros2 topic info /global_costmap/costmap`

### Topic Connection Issues

- Check topic connections: `ros2 topic info <topic_name>`
- List all topics: `ros2 topic list`
- View active nodes: `ros2 node list`



