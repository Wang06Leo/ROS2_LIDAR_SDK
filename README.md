# ROS2_LIDAR_SDK
ROS2 SDK to enable integration with LiDAR Smart Sensors (SICK Picoscan150) for robotics and autonomous systems 

## Main features and characteristics:
- ROS2 LiDAR Driver Package (in C++ or Python)
- Point Cloud Processing & Filtering Nodes
- Calibration & Diagnostic Tools
- Multi-LiDAR Synchronization Module

## Before start
Clone the repository into your ROS 2 workspace
```bash
cd ~/your_ros2_ws/src
git clone https://github.com/Wang06Leo/ROS2_LIDAR_SDK.git lidar_ws
```

## Installations
ROS2 HUMBLE:
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

sick_scan_xd:
```bash
cd ~/your_ros2_ws/src/lidar_ws/src
git clone https://github.com/SICKAG/sick_scan_xd.git
cd sick_scan_xd
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

## Find Your LiDAR’s IP Address

### Step 1: Connect LiDAR via Ethernet

- Connect your LiDAR directly to your PC using an Ethernet cable.
- Make sure your PC is set to a **static IP address** in the same subnet range (e.g., `192.168.1.100`).

### Step 2: Use Wireshark to Discover the LiDAR IP

1. Launch Wireshark
2. Select your Ethernet interface (e.g., `eth0`, `enp1s0`) and start capturing.
3. Power on or reset your LiDAR device.
4. Look for incoming packets from a `192.168.x.x` address — this is likely your LiDAR.

### Example of Lidar ip searching:  

For example my Lidars' ip addresses are 192.168.1.133 and 192.168.1.134.


## Change your lidars' IP addresses

For example you get a lidar ip that is 192.168.0.100

Copy your ip addresses and paste on web search bar

You should see a SICK service webpage

Login to service level
- Password: servicelevel

Go to connection option and change your ip address:
- first lidar to 192.168.1.134 
- second lidar to 192.168.1.133

## Lidar driver package

Launch your first lidar:
```bash
ros2 launch lidar_driver first_lidar.py
```

Launch your second lidar:
```bash
ros2 launch lidar_driver second_lidar.py
```

## Point cloud

In the Rviz2 of the lidar_driver package you launched:

- Change the fixed frame to "world" 
- Unselect the LaserScan and select PointCloud2

## Multi-LiDAR Synchronization Module

If your 2 Lidars' ip address followed:
- first lidar to 192.168.1.134 
- second lidar to 192.168.1.133

You can use this module directly

```bash
ros2 launch dual_merge dual_merge.py
```

## How to modify specific parameters or settings

```bash
cd ~/your_ros2_ws/src/lidar_ws/src/dual_merge/launch
nano dual_merge.py
```

At line 20, 21, follow the instruction to change first lidar's x, y, z, r, p, y 

At line 57, 58, follow the instruction to change second lidar's x, y, z, r, p, y 

