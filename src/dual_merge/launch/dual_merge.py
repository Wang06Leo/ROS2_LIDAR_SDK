import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    sick_scan_launch_file = os.path.join(
        get_package_share_directory('sick_scan_xd'),
        'launch',
        'sick_picoscan.launch'
    )

    # --- Front LiDAR ---
    static_tf_lidar_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_front',
        arguments=[
            '0.28', '0.0', '0.04',
            '0', '0', '-3.1415',
            'base_link', 'laser_front_1'
        ]
    )

    sick_lidar_front_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_lidar_front',
        output='log',
        arguments=[
            sick_scan_launch_file,
            'hostname:=192.168.1.134', #change it with your own lidar ip
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
            'verbose_level': 1
        }]
    )

    # --- Rear LiDAR ---
    static_tf_lidar_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_rear',
        arguments=[
            '-0.28', '0.0', '0.04',
            '0', '3.1415', '0',
            'base_link', 'laser_rear_1'
        ]
    )

    sick_lidar_rear_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_lidar_rear',
        output='log',
        arguments=[
            sick_scan_launch_file,
            'hostname:=192.168.1.133', #change it with your own lidar ip
            'udp_receiver_ip:=192.168.1.100',
            'udp_port:=2112',
            'imu_udp_port:=7500',
            'publish_frame_id:=laser_rear',
            '--ros-args',
            '--log-level', 'error',
            '-r', '/scan_fullframe:=/scan_rear',
            '-r', '/cloud_unstructured_fullframe:=/cloud_rear',
            '-r', '/sick_lidar_rear/imu:=/imu_rear',
            '-r', '__node:=sick_lidar_rear'
        ],
        parameters=[{
            'verbose_level': 1
        }]
    )

    # --- Merger Node ---
    dual_laser_merger_node = ComposableNodeContainer(
        name='laser_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'enable_calibration': False},
                    {'laser_1_topic': '/scan_front'},
                    {'laser_2_topic': '/scan_rear'},
                    {'merged_topic': '/merged'},
                    {'target_frame': 'base_link'}, #you can change to other frame if u want
                    {'laser_1_x_offset': 0.0},
                    {'laser_1_y_offset': 0.0},
                    {'laser_1_yaw_offset': 0.0},
                    {'laser_2_x_offset': 0.0},
                    {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': 0.0},
                    {'tolerance': 0.01},
                    {'queue_size': 5},
                    {'angle_increment': 0.001},
                    {'scan_time': 0.067},
                    {'range_min': 0.01},
                    {'range_max': 25.0},
                    {'min_height': -1.0},
                    {'max_height': 1.0},
                    {'angle_min': -3.141592654},
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': True},
                    {'allowed_radius': 0.45},
                    {'enable_shadow_filter': True},
                    {'enable_average_filter': True},
                ]
            )
        ],
        output='screen',
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('dual_merge'),
                'config',
                'rviz.rviz'
            )
        ]
    )

    return LaunchDescription([
        static_tf_lidar_front,
        static_tf_lidar_rear,
        sick_lidar_front_node,
        sick_lidar_rear_node,
        dual_laser_merger_node,
        rviz_node
    ])
