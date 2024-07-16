from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    
    
    world_name = LaunchConfiguration('world_name', default='garage')

    visualization_tools_launch = os.path.join(get_package_share_directory('visualization_tools'), "launch",'visualization_tools.launch.py')


    included_launch_visualization_tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visualization_tools_launch),
        launch_arguments={'world_name':world_name}.items()
    )


    ld = LaunchDescription()
    ld.add_action(included_launch_visualization_tools)

    return ld

if __name__ == '__main__':
    generate_launch_description()
