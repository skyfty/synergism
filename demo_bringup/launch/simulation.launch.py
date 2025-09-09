
"""This is all-in-one launch script intended for use by nav2 developers."""

import os
from pathlib import Path
import math

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
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction

from orient_common.launch import ReplacePath,JoinPath
from building_map.generator import Building, indent_etree, ElementToString

def parse_editor_yaml(input_filename):
    if not os.path.isfile(input_filename):
        raise FileNotFoundError(f'input file {input_filename} not found')

    with open(input_filename, 'r') as f:
        y = yaml.load(f, Loader=yaml.CLoader)
        return Building(y)
    
def spawn_robot_at_vertex_idx(level, vertex_idx):
    vertex = level.transformed_vertices[vertex_idx]
    robot_name = vertex.params['spawn_robot_name'].value
    robot_type = vertex.params['spawn_robot_type'].value

    yaw = 0
    # find the first vertex connected by a lane to this vertex
    for lane in level.lanes:
        if vertex_idx == lane.start_idx or vertex_idx == lane.end_idx:
            yaw = level.edge_heading(lane)
            if lane.orientation() == 'backward':
                yaw += math.pi
            break
    robot_topic = robot_name + '/robot_description'
    robot_x = vertex.x - level.transform.x
    robot_y = vertex.y - level.transform.y
    gz_spawn_entity = Node( package='ros_gz_sim', executable='create', output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['-topic', robot_topic, '-x', str(robot_x), '-y', str(robot_y), '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', str(yaw), '-name', robot_name],
    )
    return gz_spawn_entity;

def create_ros_gz_bridge(robot_name):
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[ 
                'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                'scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                'scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                'imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            parameters=[
                {
                    'use_sim_time': True,
                }
            ],
            namespace=robot_name,
            remappings=[ 
                ('/world/world_model/model/joint_state', '/joint_states'),
            ],
            output='screen'
        )
    return bridge
                
def spawn_model(context, *args, **kwargs):
    building_map_path = LaunchConfiguration('building_map_path').perform(context)
    print(f'Using building map file: {building_map_path}')
    if not os.path.isfile(building_map_path):
        raise FileNotFoundError(f'building map file {building_map_path} not found')
    
    actions = []
    building = parse_editor_yaml(building_map_path)
    for level_name, level in building.levels.items():
        for vertex_idx, vertex in enumerate(level.vertices):
            if 'spawn_robot_type' in vertex.params:
                robot_name = vertex.params['spawn_robot_name'].value
                gz_spawn_entity = spawn_robot_at_vertex_idx(level, vertex_idx)
                actions.append(gz_spawn_entity)
                actions.append(create_ros_gz_bridge(robot_name))
    return actions


def generate_launch_description():
    # Get the launch directory    
    map_name = LaunchConfiguration('map_name', default="office")
    orient_description_share_dir = get_package_share_directory('orient_description')
    demo_maps_share_dir = get_package_share_directory('demo_maps')

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
        demo_maps_share_dir,
        'maps',
        map_name,
    ])
    
    world_path = PathJoinSubstitution([
        world_dir,
        TextSubstitution(text=''),
        [map_name, TextSubstitution(text='.world')]
    ])

    building_map_path = PathJoinSubstitution([
        PathJoinSubstitution([
            demo_maps_share_dir,
            map_name,
        ]),
        TextSubstitution(text=''),
        [map_name, TextSubstitution(text='.building.yaml')]
    ])
    
    declare_building_map_path_cmd = DeclareLaunchArgument(
        'building_map_path',
        default_value=building_map_path,
        description='Full path to building model file to load')
    ld.add_action(declare_building_map_path_cmd)


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


    ld.add_action(declare_world_cmd)
    ld.add_action(OpaqueFunction(function=spawn_model))


  
    return ld
