import os

import launch
from ament_index_python.packages import get_package_share_directory


def launch_carla_spawn_object(context, *args, **kwargs):
    carla_spawn_objects_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': get_package_share_directory('carla_aorta_test') + '/config/objects_demo_3.json'
        }.items()
    )

    return [carla_spawn_objects_launch]


def generate_launch_description():
    temp = [
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='100'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sigterm_timeout',
            default_value='15'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=str(["altran", "vehicle_01", "vehicle_02", "vehicle_03", "vehicle_04"])
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'passive': launch.substitutions.LaunchConfiguration('passive'),
                'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds'),
            }.items()
        )]
    temp.append(launch.actions.OpaqueFunction(function=launch_carla_spawn_object))
    ld = launch.LaunchDescription(temp)
    return ld


if __name__ == '__main__':
    generate_launch_description()
