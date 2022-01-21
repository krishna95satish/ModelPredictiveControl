from launch import LaunchDescription
from launch_ros.actions import Node

rviz_path="/home/krishna/Desktop/Git_MPC_Python/trajectory_gen_mpc/Rviz/MPC_Trajecotry_viz.rviz" # Absolute Path to Rviz config file

def generate_launch_description():
    launch_description = LaunchDescription()

    trajectory_gen_node = Node(
        package="trajectory_gen_mpc",
        executable="TrajectorNode"
    )

    dummy_carla_node = Node(
        package="trajectory_gen_mpc",
        executable="CarlaNode"
    )

    rviz_visualizer = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', str(rviz_path)]
    )

    launch_description.add_action(trajectory_gen_node)
    launch_description.add_action(dummy_carla_node)
    launch_description.add_action(rviz_visualizer)
    return launch_description
