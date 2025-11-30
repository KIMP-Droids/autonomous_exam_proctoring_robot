# Exam Proctoring Robot

ROS2 (C++) workspace for Exam Proctoring Bot:
- URDF 
- Gazebo simulation
- Navigation (Nav2)
- SLAM (slam_toolbox)
- Robot localization

## Quickstart

```bash
git clone https://github.com/KiranGunathilaka/autonomous_exam_proctoring_robot
cd exam_bot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch exam_bot_bringup exam_bot_sim_bringup.launch.py
```

### For rebuilding and sourcing back

Change the below line in the *rebuild.sh* file to suit your cloned directory location

WS=~/development/ros/exam_bot_ws

then,

```bash
sudo chmod 777 rebuild.sh
./rebuild.sh
```