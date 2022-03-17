from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('car_emergency'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='car_emergency',
            executable='car_emergency',
            name='car_emergency_node',
            output='screen',
            parameters = [config]
        )
    ])