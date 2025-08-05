# ROS2_LIDAR_SDK
ROS2 SDK to enable integration with LiDAR Smart Sensors for robotics and autonomous systems 

## Main features and characteristics:
- ROS2 LiDAR Driver Package (in C++ or Python)
- Point Cloud Processing & Filtering Nodes
- Calibration & Diagnostic Tools
- Multi-LiDAR Synchronization Module

## Before start
Clone the repository into your ROS 2 workspace
```bash
cd ~/your_ros2_ws/src
git clone https://github.com/Wang06Leo/contract.git
```

## Installations
ROS2 HUMBLE:
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

sick_scan_xd:
```bash
cd ~/your_ros2_ws/src/lidar_ws/src
git clone https://github.com/SICKAG/sick_scan_xd.git
git checkout feature/ros2_kilted_fix
```

dual_laser_merger:
```bash
cd ~/your_ros2_ws/src/lidar_ws/src
git clone -b humble https://github.com/pradyum/dual_laser_merger.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Wireshark (For finding ip address for lidars):
```bash
# To install WireShark 
sudo apt update
sudo apt install wireshark
```


Open wire shark to check the ip of the lidar
You should likely get 192.168.x.x
