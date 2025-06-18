import os # python module for path handling

from ament_index_python.packages import get_package_share_directory  # as name suggests to get the "share" directory to import config files etc.
from launch import LaunchDescription  # main class in which sequence of actions is defined
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess  # ExecuteProcess to run cli commands
from launch.actions import SetEnvironmentVariable # used for gazebo models environments
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription  # to include other launch files
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit  # handle events when a process exits
from launch.conditions import IfCondition



def generate_launch_description():
    
    ld = LaunchDescription()

    my_package_path = get_package_share_directory("agv_simulation")
    world_file_path = os.path.join(my_package_path, "worlds", "test_world_new.sdf")

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


    return ld