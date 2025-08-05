import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sick_scan_launch_file = os.path.join(
        get_package_share_directory('sick_scan_xd'),
        'launch',
        'sick_picoscan.launch'
    )

    static_tf_lidar_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_rear',
        arguments=[
            '0.0', '0.0', '0.0',   # x, y, z
            '0.0', '0.0', '0.0',          # roll, pitch, yaw (in radians)
            'base_link', 'laser_front_1' 
            #transform from one frame to another frame, you can change the "base_link" to any frame that you want.
        ]
    )

    sick_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_lidar_front',
        output='log',
        arguments=[
            sick_scan_launch_file,
            'hostname:=192.168.1.134', # change it to your lidar ip.
            'udp_receiver_ip:=192.168.1.100',
            'udp_port:=2115',
            'imu_udp_port:=7503',
            'publish_frame_id:=laser_front',
            '--ros-args',
            '--log-level', 'error',
            '-r', '/scan_fullframe:=/scan_front',
            '-r', '/cloud_unstructured_fullframe:=/cloud_front',
            '-r', '/sick_lidar_front/imu:=/imu_front',
            '-r', '__node:=sick_lidar_front'
        ],
        parameters=[{
            'verbose_level': 1,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('lidar_driver'),
                'config',
                'rviz.rviz'
            )
        ]
    )

    return LaunchDescription([
        static_tf_lidar_front,
        sick_lidar_node,
        rviz_node
    ])
