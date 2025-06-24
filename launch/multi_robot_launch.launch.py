import os # python module for path handling
import tempfile
import yaml

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
import launch.logging


def generate_launch_description():
    
    ld = LaunchDescription()

    my_package_path = get_package_share_directory("agv_simulation")
    xacro_file = os.path.join(my_package_path, 'urdf', 'dose_car.urdf.xacro')
    nav_launch_dir = os.path.join(my_package_path, 'launch', 'nav2_bringup')
    map_file = os.path.join(my_package_path, 'map', 'sacramento.yaml')
    bt_xml_file = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
        )

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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='use simulator time'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(my_package_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file_cmd)

    
    
    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    # remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    remappings = []

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)

    

    def generate_bridge_config(agv_config):

        namespace = agv_config['namespace']
        model_name = agv_config['name']
         # first create logic for making bridge config with namespace append on start of gazebo and ros topic
        bridge_config_data = []

        bridge_config_data.append({
            'ros_topic_name': '/clock',        # Global topic
            'gz_topic_name': '/clock',         # Global in Gazebo too
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS'
        })

        # Command velocity (ROS → Gazebo)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/cmd_vel",
            'gz_topic_name': f"/{namespace}/cmd_vel",
            'ros_type_name': "geometry_msgs/msg/Twist",
            'gz_type_name': "gz.msgs.Twist",
            'direction': "ROS_TO_GZ"
        })
            
        # Odometry (Gazebo → ROS)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/odom",
            'gz_topic_name': f"/{namespace}/odom",
            'ros_type_name': "nav_msgs/msg/Odometry",
            'gz_type_name': "gz.msgs.Odometry",
            'direction': "GZ_TO_ROS"
        })
            
        # Joint states (Gazebo → ROS)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/joint_states",
            'gz_topic_name': f"/{namespace}/joint_states",
            'ros_type_name': "sensor_msgs/msg/JointState",
            'gz_type_name': "gz.msgs.Model",
            'direction': "GZ_TO_ROS"
        })
            
        # Lidar 1 scan (Gazebo → ROS)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/scan1",
            'gz_topic_name': f"/{namespace}/lidar1",
            'ros_type_name': "sensor_msgs/msg/LaserScan",
            'gz_type_name': "gz.msgs.LaserScan",
            'direction': "GZ_TO_ROS"
        })
            
        # Lidar 2 scan (Gazebo → ROS)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/scan2",
            'gz_topic_name': f"/{namespace}/lidar2",
            'ros_type_name': "sensor_msgs/msg/LaserScan",
            'gz_type_name': "gz.msgs.LaserScan",
            'direction': "GZ_TO_ROS"
        })
            
        # Camera image (Gazebo → ROS)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/image_rect",
            'gz_topic_name': f"/world/empty/model/{model_name}/link/{namespace}/base_link/sensor/camera/image",
            'ros_type_name': "sensor_msgs/msg/Image",
            'gz_type_name': "gz.msgs.Image",
            'direction': "GZ_TO_ROS"
        })
            
        # Camera info (Gazebo → ROS)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/camera_info",
            'gz_topic_name': f"/world/empty/model/{model_name}/link/{namespace}/base_link/sensor/camera/camera_info",
            'ros_type_name': "sensor_msgs/msg/CameraInfo",
            'gz_type_name': "gz.msgs.CameraInfo",
            'direction': "GZ_TO_ROS"
        })
            
        # Lift controller (ROS → Gazebo)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/lift_cmd",
            'gz_topic_name': f"/{namespace}/lift_cmd",
            'ros_type_name': "std_msgs/msg/Float64",
            'gz_type_name': "gz.msgs.Double",
            'direction': "ROS_TO_GZ"
        })
            
        # Lift servo controller (ROS → Gazebo)
        bridge_config_data.append({
            'ros_topic_name': f"/{namespace}/lift_servo_cmd",
            'gz_topic_name': f"/{namespace}/lift_servo_cmd",
            'ros_type_name': "std_msgs/msg/Float64",
            'gz_type_name': "gz.msgs.Double",
            'direction': "ROS_TO_GZ"
        })

        # Write dynamic bridge config to temporary file
        temp_bridge_config = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(bridge_config_data, temp_bridge_config, default_flow_style=False)
        temp_bridge_config.close()


        return temp_bridge_config.name

    def create_agv_nodes(agv_config):

        config_file_path = generate_bridge_config(agv_config)

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
                'config_file':config_file_path,
                'use_sim_time': 'true'
            }.items()
        )

        mapper_node = Node(
            package='agv_simulation',
            executable='frame_remapper',
            name='odom_mapper',
            namespace=agv_config['namespace'],
            output='screen'
        )

        scan_merger_node = Node(
            package='agv_simulation',
            executable='scan_merger_v2',
            name='scan_merger_v2',
            namespace=agv_config['namespace'],
            output='screen'
        )

        bringup_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'namespace': agv_config['namespace'],
                'use_namespace': 'True',
                'map': '',
                'map_server': 'False',
                'params_file': params_file,
                'default_bt_xml_filename': bt_xml_file,
                'autostart': 'true',
                'use_sim_time': use_sim_time,
                'log_level': 'info'
            }.items()
        )

        # # Static transform: map -> odom (initial pose)
        # static_transform_publisher = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_map_odom',
        #     namespace=agv_config['namespace'],
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', f'{agv_config['namespace']}/odom'],
        #     parameters=[{'use_sim_time': True}]
        # )

        delayed_mapper_node = TimerAction(
            period=2.0,
            actions=[mapper_node]
        )

        delayed_scan_merger_node = TimerAction(
            period=4.0,
            actions=[scan_merger_node]
        )

        delayed_bringup_node = TimerAction(
            period=8.0,
            actions=[bringup_node]
        )

        return [
            bridge,
            delayed_mapper_node,   
            delayed_scan_merger_node,
            delayed_bringup_node
        ]
    

    agv_nodes = create_agv_nodes(agv_configs[0])
    for action in agv_nodes:
        ld.add_action(action)


    return ld