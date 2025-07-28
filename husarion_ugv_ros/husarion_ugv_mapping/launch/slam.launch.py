import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",  # Changed from true to false unless using Gazebo
        description="Use simulation (Gazebo) clock if true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("husarion_ugv_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to the SLAM configuration file"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz if true"
    )
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'panther/base_footprint', 'panther/base_link']
    )

    # Node for SLAM Toolbox - Changed to async and added lifecycle
    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",  # Changed from sync to async
        name="slam_toolbox",
        namespace="",
        output="screen",
        arguments=['--ros-args', '--log-level', 'debug'],  # Add debug logging
        parameters=[
            LaunchConfiguration("slam_config"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),

            }
        ]
    )

    # Map Saver Node
    map_saver_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        namespace="",
        output='screen',
        parameters=[
            {'save_map_timeout': 5.0},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'free_thresh_default': 0.196},
            {'occupied_thresh_default': 0.65},
            {'save_map_on_exit': True}  # Add this parameter
        ]
    )

    # Lifecycle Manager Node - Fixed node names
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        namespace="",
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['slam_toolbox', 'map_saver_server']},  # Fixed node list
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    rviz_config_path = os.path.join(
        get_package_share_directory("husarion_ugv_mapping"),
        "rviz",
        "mapping.rviz"
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox,
        rviz_arg,
        map_saver_server,
        static_transform_publisher,
        lifecycle_manager,
        rviz_node,
    ])