# ROS2 Mapping with Lidar(/scan)
A ROS2 package that implements 2D mapping using SLAM.

## Overview

This package provides launch files and configurations to create 2D occupancy grid maps using a Lidar sensor on a robot platform. It utilizes SLAM toolbox to generate accurate maps for navigation.

## Prerequisites
- ROS 2 (Jazzy)
- Rosbot Workspace


## Installation

1. Create a directory for the project and clone this repository into your ROS2 workspace's `src` directory:

```bash
cd ~/rosbot_ws/src/rosbot_ros
git clone https://github.com/InterplanetarCodebase/rosbot_mapping-by-scan-.git rosbot_mapping
```

2. Install dependencies:

```bash
cd ~/rosbot_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
cd ~/rosbot_ws
colcon build --packages-select rosbot_mapping
source install/setup.bash
```

## Usage

Launch the mapping node:

```bash
ros2 launch rosbot_mapping slam.launch.py
```
Save the map :

```bash
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{
  map_url: './maps/my_map',
  image_format: 'pgm',
  map_mode: 'trinary',
  free_thresh: 0.196,
  occupied_thresh: 0.65
}"
```

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Author

Abul Hasnat Abdullah

## Acknowledgments

* ROS2 community
