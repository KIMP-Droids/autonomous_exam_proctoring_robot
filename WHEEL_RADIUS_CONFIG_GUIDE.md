# Wheel Radius Configuration Guide

This guide shows where and how to change the wheel radius parameter for the robot.

## Wheel Radius Parameter Location

The wheel radius is currently set to **0.1 m (10 cm)** and is defined in the URDF file.

### Primary Location: URDF File

**File:** `src/robot_description/urdf/AEP_Robot_macro.urdf.xacro`

**Line:** ~36 (in the diff-drive plugin section)

```xml
<gazebo>
  <plugin name="gz::plugin::DiffDrive" filename="gz-sim-diff-drive-system">
    <left_joint>${prefix}left_wheel_joint</left_joint>
    <right_joint>${prefix}right_wheel_joint</right_joint>
    <wheel_separation>0.5897</wheel_separation>
    <wheel_radius>0.1</wheel_radius>  <!-- CHANGE THIS VALUE -->
    <odom_publish_rate>50</odom_publish_rate>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <odom_frame>odom</odom_frame>
    <robot_base_frame>${prefix}base_link</robot_base_frame>
  </plugin>
</gazebo>
```

**To change:** Edit the `<wheel_radius>0.1</wheel_radius>` value.

### Secondary Location: Wheel Collision Geometry

**File:** `src/robot_description/urdf/AEP_Robot_macro.urdf.xacro`

**Lines:** ~225 and ~255 (left and right wheel collision definitions)

```xml
<collision>
  <origin xyz="0 0 0" rpy="1.5708 0 0" />
  <geometry>
    <cylinder radius="0.1" length="0.05" />  <!-- CHANGE radius HERE -->
  </geometry>
</collision>
```

**To change:** Edit the `radius="0.1"` value in both left and right wheel collision geometries.

**Note:** The collision radius should match the wheel_radius in the plugin for accurate physics simulation.

## Measuring Actual Wheel Radius

Use the provided script to measure the actual wheel radius from the STL mesh:

```bash
# Install numpy-stl if needed
pip install numpy-stl

# Run measurement script
python3 tools/measure_wheel_radius.py src/robot_description/meshes/left_wheel_link.STL
```

This will output the estimated radius based on the STL bounding box.

## Package Configuration Files Overview

### 1. robot_description Package

**Purpose:** Robot URDF/xacro, meshes, RViz configs

**Key Files:**
- `urdf/AEP_Robot_macro.urdf.xacro` - Main robot definition
  - Wheel radius: Line ~36 (`<wheel_radius>`)
  - Wheel collision radius: Lines ~225, ~255 (`<cylinder radius>`)
  - Wheel separation: Line ~35 (`<wheel_separation>`)
  - IMU position: Line ~135 (imu_joint origin)
  - LiDAR position: Line ~103 (lidar_joint origin)
  
- `urdf/main.urdf.xacro` - Includes macro and instantiates robot
- `config/rviz/description.rviz` - RViz config for description-only view
- `launch/display.launch.py` - Launch file for description-only

**Configuration Parameters:**
- Robot geometry (links, joints, collisions)
- Sensor positions (LiDAR, IMU)
- Gazebo plugins (diff-drive, LiDAR, IMU)
- Visual materials and colors

### 2. robot_sim Package

**Purpose:** Gazebo simulation worlds and robot spawning

**Key Files:**
- `worlds/classroom.sdf` - Gazebo world definition
  - Physics parameters (gravity, timestep)
  - Floor/wall models
  - Lighting
  
- `launch/classroom_world.launch.py` - Launches Gazebo with world
  - World file path
  - Robot spawn position (x, y, z, yaw)
  - use_sim_time parameter

**Configuration Parameters:**
- World physics settings
- Robot spawn pose
- Simulation time settings

### 3. robot_bringup Package

**Purpose:** Top-level launch files that orchestrate everything

**Key Files:**
- `launch/sim_classroom.launch.py` - Full simulation launch
  - Includes robot_sim world launch
  - Starts bridges (cmd_vel, odom, scan, imu)
  - Launches RViz
  
- `launch/slam_classroom.launch.py` - SLAM demo launch
  - Includes sim_classroom launch
  - Starts slam_toolbox
  
- `config/rviz/sim.rviz` - RViz config for simulation
  - Displays: RobotModel, TF, LaserScan, Odometry, IMU
  - Fixed frame: odom
  
- `config/rviz/slam.rviz` - RViz config for SLAM
  - Displays: RobotModel, TF, LaserScan, Odometry, IMU, Map
  - Fixed frame: map

**Configuration Parameters:**
- Bridge topic mappings
- RViz display settings
- Launch file arguments (use_sim_time, rviz toggle)

### 4. robot_slam Package

**Purpose:** SLAM Toolbox configuration

**Key Files:**
- `config/slam_toolbox_online_async.yaml` - SLAM parameters
  - Frame names (odom_frame, map_frame, base_frame)
  - Scan topic: `/scan`
  - Map resolution: 0.05 m
  - Scan matching parameters
  - Loop closure settings

**Configuration Parameters:**
- SLAM algorithm parameters
- Frame names
- Topic remappings
- Map resolution

### 5. robot_nav Package

**Purpose:** Nav2 navigation configuration (optional)

**Key Files:**
- (Currently empty - Nav2 configs would go here)
  - `config/nav2_params.yaml` - Nav2 parameters (if implemented)
  - `launch/nav2.launch.py` - Nav2 launch (if implemented)

**Configuration Parameters:**
- Costmap parameters
- Planner settings
- Controller parameters
- Behavior tree

## Quick Reference: Where to Change What

| Parameter | File | Location |
|-----------|------|----------|
| **Wheel Radius** | `robot_description/urdf/AEP_Robot_macro.urdf.xacro` | Line ~36: `<wheel_radius>` |
| **Wheel Collision Radius** | `robot_description/urdf/AEP_Robot_macro.urdf.xacro` | Lines ~225, ~255: `<cylinder radius>` |
| **Wheel Separation** | `robot_description/urdf/AEP_Robot_macro.urdf.xacro` | Line ~35: `<wheel_separation>` |
| **Robot Spawn Position** | `robot_sim/launch/classroom_world.launch.py` | Lines ~76-79: spawn arguments |
| **LiDAR Position** | `robot_description/urdf/AEP_Robot_macro.urdf.xacro` | Line ~103: lidar_joint origin |
| **IMU Position** | `robot_description/urdf/AEP_Robot_macro.urdf.xacro` | Line ~135: imu_joint origin |
| **SLAM Parameters** | `robot_slam/config/slam_toolbox_online_async.yaml` | Various lines |
| **Bridge Topics** | `robot_bringup/launch/sim_classroom.launch.py` | Bridge node arguments |
| **RViz Displays** | `robot_bringup/config/rviz/*.rviz` | Displays section |

## Step-by-Step: Changing Wheel Radius

1. **Measure actual wheel radius:**
   ```bash
   python3 tools/measure_wheel_radius.py src/robot_description/meshes/left_wheel_link.STL
   ```

2. **Update URDF plugin parameter:**
   - Edit `src/robot_description/urdf/AEP_Robot_macro.urdf.xacro`
   - Change `<wheel_radius>0.1</wheel_radius>` to your measured value

3. **Update collision geometry (optional but recommended):**
   - In the same file, update both wheel collision cylinders
   - Change `radius="0.1"` to match your measured value

4. **Rebuild:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

5. **Test:**
   ```bash
   ros2 launch robot_bringup sim_classroom.launch.py
   # Drive robot and verify odometry matches expected distance
   ```

## Notes

- **Wheel radius** affects:
  - Odometry accuracy
  - Robot speed (for same cmd_vel)
  - Distance traveled calculations
  
- **Wheel separation** affects:
  - Turning radius
  - Odometry angular calculations
  
- Always rebuild after changing URDF parameters
- Test odometry by driving a known distance and comparing to `/odom` output

