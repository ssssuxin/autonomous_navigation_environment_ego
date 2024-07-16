import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
def generate_launch_description():
    show_public_param=False

    vehicle_simulator_para_dir = os.path.join(get_package_share_directory('vehicle_simulator'), 'config', 'vehicleSimulator.yaml')

    vehicle_simulator_node = Node(
        package='vehicle_simulator',
        executable='vehicleSimulator',
        output='screen',
        parameters=[vehicle_simulator_para_dir,#9
                    {'vehicleHeight': LaunchConfiguration('vehicleHeight')},
                    {'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ')},
                    {'vehicleX': LaunchConfiguration('vehicleX')},
                    {'vehicleY': LaunchConfiguration('vehicleY')},
                    {'terrainZ': LaunchConfiguration('terrainZ')},
                    {'vehicleYaw': LaunchConfiguration('vehicleYaw')},
                    {'show_public_param': show_public_param},]
    )

    ld = LaunchDescription()
    ld.add_action(vehicle_simulator_node)
    # ld.add_action(path_follower_node)

    return ld
