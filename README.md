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
git clone https://github.com/Wang06Leo/ROS2-SDK-for-Smart-Sensors-Lidar-.git lidar_ws
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
<img width="961" height="274" alt="image" src="https://github.com/user-attachments/assets/b4e74a22-dec0-4e34-b253-b1df17ee5263" />

For example my Lidars' ip addresses are 192.168.1.133 and 192.168.1.134.


## Change your lidars' IP addresses

For example you get a lidar ip that is 192.168.0.100

Copy your ip addresses and paste on web search bar

<img width="1366" height="728" alt="image" src="https://github.com/user-attachments/assets/4aaddd87-d0dc-450a-9b41-7c2f74da1553" />

Then you will ses a page like this:

<img width="1844" height="975" alt="image" src="https://github.com/user-attachments/assets/600085ee-4680-45a0-85d4-049726550256" />

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

<img width="914" height="684" alt="image" src="https://github.com/user-attachments/assets/2c31a1f2-c66f-43bf-bd42-9e8f142918da" />

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

