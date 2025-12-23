# Exam Proctoring Robot - ROS 2 Jazzy

ROS 2 Jazzy workspace for Exam Proctoring Bot:
- URDF/xacro robot description
- Gazebo (Gz) simulation with ros_gz_sim
- SLAM (slam_toolbox)
- Navigation (Nav2) - optional

## Prerequisites

- ROS 2 Jazzy installed
- Gazebo (Gz) and ros_gz_sim packages
- Standard ROS 2 packages: rviz2, robot_state_publisher, etc.

## Build Instructions

```bash
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Quick Start Commands

### 1. Description Only (RViz Display)

Test the robot URDF model in RViz:

```bash
ros2 launch robot_description display.launch.py
```

**Expected Result:**
- RViz opens showing robot model
- TF tree visible (base_link -> lidar_link, wheels, casters)
- Robot model loads with meshes

**Acceptance Test A:** Robot model displays correctly, TF valid, meshes load.

### 2. Simulation (Gazebo + RViz)

Launch Gazebo simulation with robot spawned:

```bash
ros2 launch robot_bringup sim_classroom.launch.py
```

**Expected Result:**
- Gazebo opens with classroom world
- Robot spawns at (2.0, -3.0, 0.1)
- RViz shows robot, LaserScan, Odometry
- TF tree: odom -> base_link -> lidar_link

**Teleop (in separate terminal):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

**Acceptance Test B:** Gazebo loads world, robot spawns, teleop moves robot, /odom updates, TF sane.

### 3. SLAM Demo (Preferred)

Launch simulation with SLAM mapping:

```bash
ros2 launch robot_bringup slam_classroom.launch.py
```

**Expected Result:**
- Same as simulation above
- Plus: slam_toolbox running
- RViz shows /map building as robot moves
- Map updates in real-time

**Teleop (in separate terminal):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

**Acceptance Test C:** Same as (B) plus slam_toolbox produces /map and RViz shows it.

## Validation Commands

Check topics:
```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /cmd_vel
```

Check TF tree:
```bash
ros2 run tf2_tools view_frames
# Output saved to frames.pdf
```

Check node graph:
```bash
rqt_graph
```

## Robot Parameters

- **Wheel Separation:** 0.5897 m (left wheel at y=+0.29485, right at y=-0.29485)
- **Wheel Radius:** 0.1 m (estimated, can be adjusted)
- **Base Frame:** base_link
- **Odometry Frame:** odom
- **Map Frame:** map (when SLAM running)
- **LiDAR Frame:** lidar_link

## Troubleshooting

### 1. TF Errors / Missing Frames

**Symptoms:** `[ERROR] [tf2_ros]: Lookup would require extrapolation into the past`

**Fix:**
- Ensure `use_sim_time:=true` is set in all launch files when running simulation
- Check that robot_state_publisher is running
- Verify Gazebo is publishing odometry

**Check:**
```bash
ros2 topic echo /tf
ros2 run tf2_ros tf2_echo odom base_link
```

### 2. No LaserScan Data

**Symptoms:** `/scan` topic exists but no data, or RViz shows no scan

**Fix:**
- Verify LiDAR plugin is loaded in URDF
- Check bridge is running: `ros2 node list | grep bridge`
- Check Gazebo sensor is publishing: `gz topic -l` (in Gazebo console)

**Check:**
```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

### 3. Bridge Not Connecting Topics

**Symptoms:** Topics exist but no data flows between ROS 2 and Gazebo

**Fix:**
- Verify bridge nodes are running: `ros2 node list`
- Check bridge syntax in launch file (topic names must match)
- Ensure Gazebo topics exist: `gz topic -l`

**Bridge format:** `/ros_topic@ros_msg_type@gz.msgs.GzType`

**Check:**
```bash
ros2 topic list
gz topic -l  # In Gazebo console or separate terminal
```

### 4. Robot Not Spawning

**Symptoms:** Gazebo opens but robot doesn't appear

**Fix:**
- Check spawn node output for errors
- Verify URDF/xacro processes correctly: `xacro src/robot_description/urdf/main.urdf.xacro`
- Check world name matches: `classroom_world`
- Increase spawn delay if needed (TimerAction period)

**Check:**
```bash
ros2 topic echo /robot_description --once
# Should show URDF XML
```

### 5. Missing Joint States / TF Errors for Wheels/Casters

**Symptoms:** RViz RobotModel shows "No transform from wheel links to base_link" or joint_states topic is empty

**Fix:**
- Verify joint state publisher plugin is loaded in URDF
- Check joint_states bridge is running: `ros2 node list | grep joint_states_bridge`
- Verify joint states topic exists: `ros2 topic echo /joint_states --once`
- Check Gazebo joint state topic: `gz topic -l | grep joint`

**Check:**
```bash
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo base_link left_wheel_link
```

**Note:** Joint states come from Gazebo's joint state publisher plugin, not from ros2_control.

### 6. Wheel Direction / Robot Moves Backwards

**Symptoms:** Robot moves opposite to cmd_vel command

**Fix:**
- Swap left/right wheel joints in diff-drive plugin
- Check wheel joint axes direction in URDF
- Verify wheel separation sign

**URDF Check:**
- Left wheel: y = +0.29485 (positive)
- Right wheel: y = -0.29485 (negative)
- Wheel separation = |left_y - right_y| = 0.5897 m

### 7. Build Errors

**Symptoms:** `colcon build` fails

**Common fixes:**
- Run `rosdep install --from-paths src --ignore-src -r -y` again
- Check package.xml dependencies are correct
- Verify CMakeLists.txt installs launch/config directories
- Clean build: `rm -rf build install log` then rebuild

### 8. Package Not Found Errors

**Symptoms:** `[ERROR] package 'robot_description' not found`

**Fix:**
- Source workspace: `source install/setup.bash`
- Rebuild: `colcon build --symlink-install`
- Check package is built: `colcon list`

### 9. RViz Config Not Found

**Symptoms:** `[ERROR] Could not load config file`

**Fix:**
- Verify config files are installed: `ls install/robot_bringup/share/robot_bringup/config/rviz/`
- Rebuild with install: `colcon build --symlink-install`
- Check CMakeLists.txt installs config directory

## File Structure

```
exam_bot_ws/
├── src/
│   ├── robot_description/     # URDF, meshes, RViz configs
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   ├── urdf/
│   │   │   ├── main.urdf.xacro
│   │   │   └── AEP_Robot_macro.urdf.xacro
│   │   └── config/rviz/
│   │       └── description.rviz
│   ├── robot_sim/            # Gazebo worlds and spawn
│   │   ├── launch/
│   │   │   └── classroom_world.launch.py
│   │   └── worlds/
│   │       └── classroom.sdf
│   ├── robot_bringup/        # Top-level launch files
│   │   ├── launch/
│   │   │   ├── sim_classroom.launch.py
│   │   │   └── slam_classroom.launch.py
│   │   └── config/rviz/
│   │       ├── sim.rviz
│   │       └── slam.rviz
│   ├── robot_slam/           # SLAM configs
│   │   └── config/
│   │       └── slam_toolbox_online_async.yaml
│   └── robot_nav/            # Nav2 configs (optional)
└── install/                  # Built packages
```

## Implementation Notes

### Simulation Approach

**Approach A Only:** Gazebo DiffDrive Plugin + ros_gz_bridge

This workspace uses **ONLY** Gazebo's built-in DiffDrive system plugin (`gz-sim-diff-drive-system`) for robot locomotion in simulation. **ros2_control and gz_ros2_control are NOT used** in the simulation setup.

**Key Components:**
- **DiffDrive Plugin:** `gz-sim-diff-drive-system` plugin on `base_link` handles wheel motion and odometry
- **Joint State Publisher:** `gz-sim-joint-state-publisher-system` plugin publishes joint states for TF tree
- **Topic Bridges:** `ros_gz_bridge` bridges topics between Gazebo and ROS 2:
  - `/cmd_vel` (ROS → Gazebo): Robot velocity commands
  - `/odom` (Gazebo → ROS): Odometry from DiffDrive plugin
  - `/scan` (Gazebo → ROS): LiDAR scan data
  - `/clock` (Gazebo → ROS): Simulation time for `use_sim_time`
  - `/joint_states` (Gazebo → ROS): Joint states for robot_state_publisher
- **Wheel Parameters:** separation=0.5897m, radius=0.1m

**Why Approach A:**
- Simpler architecture without ros2_control dependencies
- Direct use of Gazebo's physics and control systems
- Easier to debug and maintain
- Sufficient for simulation and SLAM demos

### Gazebo Plugins

- **Diff-drive:** `gz-sim-diff-drive-system` plugin on `base_link`
  - Consumes `/cmd_vel` for motion control
  - Publishes `/odom` for odometry
  - Configures TF frames: `odom` → `base_link`
- **Joint State Publisher:** `gz-sim-joint-state-publisher-system` plugin
  - Publishes joint states for all joints (wheels + casters)
  - Required for `robot_state_publisher` to publish TF transforms
- **LiDAR:** `gz-sensors-lidar` plugin on `lidar_link`
  - Publishes `/scan` LaserScan messages

### Casters

- Casters are kept as continuous joints (not fixed)
- They remain passive physics-driven joints (not controlled)
- May cause minor instability; acceptable for demo purposes
- Can be fixed in URDF if needed for stability

### Controllers Configuration

The `controllers.yaml` file exists but is **NOT required** for simulation. It is kept for reference but is not loaded by any simulation launch files. The simulation relies entirely on Gazebo plugins and bridges.

## Screenshots / Evidence

Create screenshots showing:
1. RViz RobotModel + TF tree
2. Gazebo world with robot spawned
3. RViz LaserScan visible
4. RViz Odometry / trajectory visible
5. (If SLAM) RViz map building
6. rqt_graph OR `ros2 topic list` output

Save in `/docs` or `/media` directory.

## Rebuild Script

For quick rebuilds, use `rebuild.sh`:

```bash
# Edit rebuild.sh to set correct workspace path
WS=/home/kiran_gunathilaka/development/ros/exam_bot_ws

# Make executable and run
chmod +x rebuild.sh
./rebuild.sh
```

## License

Apache-2.0

## Maintainer

KIMP_Droids (kimpdroids@gmail.com)
