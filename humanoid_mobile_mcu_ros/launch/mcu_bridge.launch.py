import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("humanoid_mobile_mcu_ros")
    default_params_file = os.path.join(pkg_dir, "config", "mcu_params.yaml")

    # Declare arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file to use"
    )
    declare_port_cmd = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port path (e.g. /dev/ttyUSB0)"
    )
    declare_baud_cmd = DeclareLaunchArgument(
        "baud_rate",
        default_value="57600",
        description="Serial communication baud rate"
    )

    # Node
    mcu_bridge_node = Node(
        package="humanoid_mobile_mcu_ros",
        executable="mcu_bridge_node",
        name="mcu_bridge_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "port": LaunchConfiguration("port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_baud_cmd)
    ld.add_action(mcu_bridge_node)

    return ld
