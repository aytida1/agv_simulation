import os # python module for path handling

from ament_index_python.packages import get_package_share_directory  # as name suggests to get the "share" directory to import config files etc.
from launch import LaunchDescription  # main class in which sequence of actions is defined
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction  # ExecuteProcess to run cli commands
from launch.actions import SetEnvironmentVariable # used for gazebo models environments
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription  # to include other launch files
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit  # handle events when a process exits
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import time


def generate_launch_description():
    
    ld = LaunchDescription()

    my_package_path = get_package_share_directory("agv_simulation")
    xacro_file = os.path.join(my_package_path, 'urdf', 'dose_car.urdf.xacro')

    # array for FIVE agv's config : namespace, parking location
    agv_configs = [
        {
            'name':'agv_1',
            'namespace':'agv1',
            'x':0.8,
            'y':0,
            'z':0.2
        },

        {
            'name':'agv_2',
            'namespace':'agv2',
            'x':0.8,
            'y':1.0,
            'z':0.2
        },

        {
            'name':'agv_3',
            'namespace':'agv3',
            'x':0.8,
            'y':2.0,
            'z':0.2
        },

        {
            'name':'agv_4',
            'namespace':'agv4',
            'x':0.8,
            'y':3.0,
            'z':0.2
        },

        {
            'name':'agv_5',
            'namespace':'agv5',
            'x':0.8,
            'y':4.0,
            'z':0.2
        }
    ]

    
    
    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ros gz bridge config file
    ros_gz_config_file = os.path.join(
        my_package_path,
        'config',
        'ros_gz_bridge.yaml'
    )

    def create_agv_nodes(agv_config):
        
        bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_bridge'),
                    'launch',
                    'ros_gz_bridge.launch.py'
                )
            ]),
            launch_arguments={
                'bridge_name':'ros_gz_bridge',
                'config_file':ros_gz_config_file,
                'namespace':agv_config['namespace']
            }
        )

        mapper_node = Node(
            package='agv_simulation',
            executable='mapper.cpp',
            name='odom_mapper',
            output='screen'
        )

        scan_merger_node = Node(
            package='agv_simulation',
            executable='scan_merger_v2',
            name='scan_merger_v2',
            output='screen'
        )

        delayed_mapper_node = TimerAction(
            period=2.0,
            actions=[mapper_node]
        )

        delayed_scan_merger_node = TimerAction(
            period=4.0,
            actions=[scan_merger_node]
        )

        return [
            bridge,
            delayed_mapper_node,
            delayed_scan_merger_node
        ]
    ld.add_action(create_agv_nodes(agv_configs[0]))


    return ld