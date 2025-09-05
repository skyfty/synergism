# 导入库
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from orient_common.launch import ReplacePath
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.substitutions import TextSubstitution

from launch.launch_description_sources import XMLLaunchDescriptionSource

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    orient_fleet_dir = get_package_share_directory('orient_fleet')
    description_dir = get_package_share_directory('orient_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    use_namespace = LaunchConfiguration('use_namespace')

    namespace = LaunchConfiguration('namespace', default='')
    log_level = LaunchConfiguration('log_level')
    nav_graph_map = LaunchConfiguration('nav_graph_map', default="office")
    
    nav_graph_file = PathJoinSubstitution([
        PathJoinSubstitution([
            get_package_share_directory('demos_maps'),
            'maps',
            nav_graph_map,
        ]),
        TextSubstitution(text='nav_graphs'),
        TextSubstitution(text='0.yaml')
    ])

    ld = LaunchDescription()

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    ld.add_action(declare_use_namespace_cmd)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    ld.add_action(declare_namespace_cmd)

    declare_nav_graph_map_cmd = DeclareLaunchArgument(
        'nav_graph_map', default_value='',
        description='Full path to nav_graph yaml file')
    ld.add_action(declare_nav_graph_map_cmd)
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    ld.add_action(declare_log_level_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    ld.add_action(declare_use_namespace_cmd)

    
    orient_fleet_node = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_blockade',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='door_supervisor',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='lift_supervisor',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
    ])
    ld.add_action(orient_fleet_node)
    
    rmf_visualization = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('rmf_visualization'),'launch','visualization.launch.xml')),
        launch_arguments={
            'namespace': namespace,
            'log_level': log_level,
            'use_sim_time': use_sim_time,
            'map_name': nav_graph_map,
            'rviz_config_file': os.path.join(get_package_share_directory('demo_bringup'),'rviz','demo.rviz'),
        }.items(),
    )
    ld.add_action(rmf_visualization)

    agent_params = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('orient_bringup'),'params','agent.yaml'),
            root_key=namespace,
            convert_types=True),
        allow_substs=True)

    fleet_mqtt_bridge = Node(
            package='orient_fleet',
            executable='fleet_mqtt_bridge',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                agent_params,
                {'use_sim_time': use_sim_time}
            ],
            namespace=namespace,
            output='both',
        )   
    ld.add_action(fleet_mqtt_bridge)
    

    kf2404_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orient_fleet_dir,'/launch','/fleet.launch.py']),
        launch_arguments={
            'namespace': namespace,
            'log_level': log_level,
            'use_sim_time': use_sim_time,
            'config_file': ReplacePath(name="kf2404",path=description_dir,source_file=os.path.join('params','fleet.yaml')),
            'nav_graph_file': nav_graph_file,
        }.items(),
    )
    ld.add_action(kf2404_fleet)
    
    
    
    
    return ld

