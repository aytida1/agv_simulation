import os
import tempfile
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# Helper function to process the configured_params and print them
def print_configured_params(context):
    # Extract info from the launch context
    configured_params = context.launch_configurations['configured_params']
    
    print("\n\n========= RewrittenYaml Object =========")
    print(f"Type: {type(configured_params)}")
    print("=======================================\n")
    
    # Create a temporary ROS node to inspect the parameters
    temp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.py')
    temp_file.write('''#!/usr/bin/env python3
import rclpy
import yaml
import sys
from rclpy.node import Node

class DumpParams(Node):
    def __init__(self):
        super().__init__('dump_params_node')
        # Get all parameters
        params = self.get_parameters([])
        # Print as YAML
        result = {param.name: param.value for param in params}
        print("\\n\\n========= CONFIGURED PARAMS =========")
        print(yaml.dump(result))
        print("=======================================\\n")
        # Shutdown
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DumpParams()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()
''')
    temp_file.close()
    os.chmod(temp_file.name, 0o755)  # Make executable
    
    # Return actions to add to the launch description
    return [
        Node(
            executable=temp_file.name,
            output='screen',
            parameters=[configured_params]
        ),
        # Clean up the temporary file
        ExecuteProcess(
            cmd=['rm', temp_file.name],
            output='screen'
        )
    ]

def generate_launch_description():
    # Get parameters file path
    params_file = os.path.join(get_package_share_directory('agv_simulation'), 'config', 'nav2_params.yaml')
    namespace = 'agv1'

    param_substitutions = {
        'use_sim_time': 'true',
        'autostart': 'false'
    }
    
    # Create the RewrittenYaml object
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # Store the configured_params in the launch configuration
    return LaunchDescription([
        # Store configured_params in launch context
        OpaqueFunction(function=lambda context: 
            context.launch_configurations.update({'configured_params': configured_params})),
        # Call function to print params
        OpaqueFunction(function=print_configured_params)
    ])