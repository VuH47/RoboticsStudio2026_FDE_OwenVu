# MinhandOwen Robotics Studio Workspace

Curated workspace containing only active and verified assets for **Minh (Vu)** and **Owen**.

## Run command 
```bash
# Use for demonstration
source /opt/ros/humble/setup.bash
source ~/41068_ws/install/setup.bash
ros2 launch 41068_ignition_bringup ice_pin_view.launch.py block_count:=4 enable_viewer:=false enble_console:=true
```

```bash
#Unit test 
cd ~/41068_ws/src/MinhandOwen && python3 -m pytest test/
```


## Debug
Follow these if the command above does not run smoothly

+ Verify package/dependency 
```bash
sudo apt-get update
sudo apt-get install -y python3-opencv python3-pyqt5 

#!/usr/bin/env python3 to line 1 to any file that build failed or not displayed as image, vid to contain executable permissions (chmod +x).
colcon build --symlink-install --packages-select 41068_ignition_bringup

# 1. THen 
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_LOCALHOST_ONLY=1
ros2 launch 41068_ignition_bringup ice_unified_perception.launch.py enable_viewer:=false enable_console:=true

# The Console window will launch and display alongside the Gazebo simulation and RViz.

```
## Quick Launch
```bash
# 1. Master simulation in snow_forest
ros2 launch 41068_ignition_bringup 41068_ignition_dual_ice.launch.py world:=snow_forest rviz:=true rabbits:=false

# 2. Deploy ice on cable
ros2 launch 41068_ignition_bringup ice_scenario.launch.py world:=snow_forest seed:=42 block_count:=1 span_ids:=span_01

# 3. Perception node
ros2 launch 41068_ignition_bringup ice_detector.launch.py

# 4. Operator UI (Owen)
ros2 launch 41068_ignition_bringup 41068_operator_ui.launch.py
```
