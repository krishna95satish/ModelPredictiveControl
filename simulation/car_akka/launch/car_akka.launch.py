from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('car_akka'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='car_akka',
            executable='car_akka',
            name='car_akka_node',
            output='screen',
            parameters = [config]
        )
    ])