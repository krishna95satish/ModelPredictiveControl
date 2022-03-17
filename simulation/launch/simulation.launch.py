from launch import LaunchDescription
from launch import LaunchIntrospector  # noqa: E402
from launch import LaunchService  # noqa: E402
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    altran_filter_node_pkg_prefix =     get_package_share_directory('altran_filter')      + "/launch/"
    car_altran_node_pkg_prefix =        get_package_share_directory('car_altran')         + "/launch/"

    akka_filter_node_pkg_prefix =       get_package_share_directory('akka_filter')        + "/launch/"
    akka_sensor_dummy_node_pkg_prefix = get_package_share_directory('akka_sensor_dummy')  + "/launch/"
    car_akka_node_pkg_prefix =          get_package_share_directory('car_akka')           + "/launch/"

    default_filter_node_pkg_prefix =    get_package_share_directory('default_filter')     + "/launch/"
    car_default_node_pkg_prefix =       get_package_share_directory('car_default')        + "/launch/"

    emergency_filter_node_pkg_prefix =  get_package_share_directory('emergency_filter')   + "/launch/"
    car_emergency_node_pkg_prefix =     get_package_share_directory('car_emergency')      + "/launch/"

    tuk_filter_node_pkg_prefix =        get_package_share_directory('tuk_filter')         + "/launch/"
    car_tuk_node_pkg_prefix =           get_package_share_directory('car_tuk')            + "/launch/"

    aorta_server_node_pkg_prefix =      get_package_share_directory('aorta_server')       + "/launch/"

    altran_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(altran_filter_node_pkg_prefix, 'altran_filter.launch.py')))

    car_altran_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_altran_node_pkg_prefix, 'car_altran_simulation.launch.py')))

    akka_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(akka_filter_node_pkg_prefix, 'akka_filter.launch.py')))

    akka_sensor_dummy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(akka_sensor_dummy_node_pkg_prefix, 'akka_sensor_dummy.launch.py')))

    car_akka_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_akka_node_pkg_prefix, 'car_akka.launch.py')))

    default_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(default_filter_node_pkg_prefix, 'default_filter.launch.py')))

    car_default_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_default_node_pkg_prefix, 'car_default.launch.py')))

    emergency_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(emergency_filter_node_pkg_prefix, 'emergency_filter.launch.py')))

    car_emergency_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_emergency_node_pkg_prefix, 'car_emergency.launch.py')))

    tuk_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tuk_filter_node_pkg_prefix, 'tuk_filter.launch.py')))

    car_tuk_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_tuk_node_pkg_prefix, 'car_tuk.launch.py')))

    aorta_server_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aorta_server_node_pkg_prefix, 'aorta_server.launch.py')))

    return LaunchDescription([
        altran_filter_node,
        car_altran_node,
        akka_filter_node,
        akka_sensor_dummy_node,
        car_akka_node,
        default_filter_node,
        car_default_node,
        emergency_filter_node,
        car_emergency_node,
        tuk_filter_node,
        car_tuk_node,
        aorta_server_node,
    ])
