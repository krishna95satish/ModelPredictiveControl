from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('aorta_demo'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='config_file',
            default_value=config
        ),
        Node(
            package='aorta_demo',
            executable='aorta_demo',
            name='aorta_demo_node',
            output='screen',
            parameters = [LaunchConfiguration('config_file')]
        )
    ])
