
"""This is all-in-one launch script intended for use by nav2 developers."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory,get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from orient_common.launch import ReplacePath
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.substitutions import TextSubstitution

from orient_common.launch import ReplacePath,JoinPath

def generate_launch_description():
    # Get the launch directory    
    map_name = LaunchConfiguration('map_name', default="office")
    orient_description_share_dir = get_package_share_directory('orient_description')
    ld = LaunchDescription()
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    ld.add_action(declare_namespace_cmd)
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    ld.add_action(declare_use_namespace_cmd)

    declare_world_cmd = DeclareLaunchArgument(
        'map_name',
        default_value="office",
        description='Full path to world model file to load')

    ld.add_action(declare_world_cmd)
    
    world_dir = PathJoinSubstitution([
        get_package_share_directory('demo_maps'),
        'maps',
        map_name,
    ])
    
    world_path = PathJoinSubstitution([
        world_dir,
        TextSubstitution(text=''),
        [LaunchConfiguration('map_name'), TextSubstitution(text='.world')]
    ])
    
    orient_description_share_parent_dir = Path(orient_description_share_dir).parent
    ign_resource_path = [
        TextSubstitution(text=orient_description_share_dir), # this gets the workspace src directory,
        TextSubstitution(text=str(orient_description_share_parent_dir)), # this gets the workspace src directory,
        PathJoinSubstitution([world_dir,'models']),
        PathJoinSubstitution([get_package_share_directory('rmf_demos_assets'), 'models']),
        PathJoinSubstitution([get_package_share_directory('gazebo_ros'), 'models']),
        PathJoinSubstitution([EnvironmentVariable('HOME', default_value=''), '.gazebo/models']),
    ]
    ign_resource_path_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value= JoinPath(ign_resource_path)
    )
    ld.add_action(ign_resource_path_env)
    
    ign_plugin_path = [
        PathJoinSubstitution([get_package_prefix('rmf_robot_sim_gz_plugins'), 'lib/rmf_robot_sim_gz_plugins']),
        PathJoinSubstitution([get_package_prefix('rmf_building_sim_gz_plugins'), 'lib/rmf_building_sim_gz_plugins']),
    ]
    ign_system_plugin_path_env = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH', JoinPath(ign_plugin_path)
    )
    ld.add_action(ign_system_plugin_path_env)

    ign_gui_plugin_path_env = AppendEnvironmentVariable(
        'IGN_GUI_PLUGIN_PATH', JoinPath(ign_plugin_path)
    )
    ld.add_action(ign_gui_plugin_path_env)
    
    open_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments=[
                ('gz_args', [' -r ', world_path]),
            ],
    )
    ld.add_action(open_ign)
    
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
  
        arguments=[ 
            'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            'scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            'scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            'imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            'camera@sensor_msgs/msg/Image@gz.msgs.Image',
            'camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            'rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            'rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            'rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            'rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
        remappings=[ 
            ('/world/world_model/model/joint_state', '/joint_states'),
        ],
        output='screen'
    )
    ld.add_action(bridge)

    return ld
