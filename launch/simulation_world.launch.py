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
    world_file_path = os.path.join(my_package_path, "worlds", "test_world_new.sdf")
    xacro_file = os.path.join(my_package_path, 'urdf', 'dose_car.urdf.xacro')

    # setting environment variable for gazebo
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(my_package_path, "models"),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', world_file_path],
        output='screen'
    )

    ld.add_action(gz_resource_path)
    ld.add_action(gazebo_launch)

    x = 0.8
    y = 0.0
    z = 0.2
    last_action = None

    # Five AGVs spawning (just spawning no contact with ros2!!)
    for i in range(5):
        namespace_string = f"agv{i+1}"
        name_string = f"agv_{i+1}"

        # Robot description for this AGV with unique scene name
        robot_description_content = ParameterValue(
            Command([
                'xacro ', xacro_file, 
                ' robot_name:=', name_string,
                ' namespace:=', namespace_string
            ]), 
            value_type=str
        )

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace_string,
            output='screen',
            parameters=[{
                'robot_description':robot_description_content,
                'use_sim_time':True,
                'publish_frequency': 50.0,  # Ensure regular publishing
            }],
            respawn=True,
            respawn_delay=2.0
        )

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name_string,
                '-topic', f"/{namespace_string}/robot_description",
                '-x', f"{x}",
                '-y', f"{y}",
                '-z', f"{z}",
                '--timeout', '30.0'
            ],
            output='screen',
            respawn=False
        )

        if last_action is None:
            ld.add_action(rsp)
            ld.add_action(spawn_entity)
        else:
            spawn_agv_envent = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[rsp, spawn_entity]
                )
            )
            ld.add_action(spawn_agv_envent)
        last_action = spawn_entity
        y = y + 0.8
        

    return ld