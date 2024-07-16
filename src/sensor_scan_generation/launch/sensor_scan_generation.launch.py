# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_prefix
# from launch.substitutions import LaunchConfiguration
# from launch.actions import ExecuteProcess
def generate_launch_description():

    sensorScanGeneration_node = Node(
        package='sensor_scan_generation',
        executable='sensorScanGeneration',
        output='screen',
    )
    ld = LaunchDescription()
    ld.add_action(sensorScanGeneration_node)

    return ld
