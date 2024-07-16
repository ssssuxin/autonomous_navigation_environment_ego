from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
#在ros1工程中被重复使用的变量cameraOffsetZ；vehicleX；vehicleY
def generate_launch_description():
    # 获取要启动的launch.py文件的路径
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ', default='0.0')
    vehicleX = LaunchConfiguration('vehicleX', default='0.0')
    vehicleY = LaunchConfiguration('vehicleY', default='0.0')
    local_planner_launch = os.path.join(get_package_share_directory('local_planner'), "launch",'local_planner.launch.py')
    terrain_analysis_ext_launch = os.path.join(get_package_share_directory('terrain_analysis_ext'), "launch",'terrain_analysis_ext.launch.py')
    # print("1111:   ",terrain_analysis_ext_launch)
    # print(other_launch_file,"!!!!!!!!!!!!")
    # 创建一个包含要启动的launch.py文件的LaunchDescription
    included_launch_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(local_planner_launch),
        launch_arguments={'test_str': 'default_valuefsef3333e111','cameraOffsetZ': cameraOffsetZ,'goalX': vehicleX,'goalY':vehicleY}.items()
    )
    included_launch_terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(terrain_analysis_ext_launch),
        # launch_arguments={'test_str': 'default_valuefsef3333e111','cameraOffsetZ': cameraOffsetZ,'goalX': vehicleX,'goalY':vehicleY}.items()
    )

    # 创建主LaunchDescription
    ld = LaunchDescription()

    # 将包含的launch添加到主LaunchDescription中
    ld.add_action(included_launch_local_planner)
    ld.add_action(included_launch_terrain_analysis_ext)

    return ld

if __name__ == '__main__':
    generate_launch_description()
