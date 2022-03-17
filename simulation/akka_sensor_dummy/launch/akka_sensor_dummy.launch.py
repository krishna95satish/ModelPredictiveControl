from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
   
    
def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('akka_sensor_dummy'),
    'config',
    'default.yaml'
    )
    return LaunchDescription([
        Node(
            package='akka_sensor_dummy',
            executable='akka_sensor_dummy',
            name='akka_sensor_dummy_node',
            output='screen',
            parameters = [config]
        )
    ])