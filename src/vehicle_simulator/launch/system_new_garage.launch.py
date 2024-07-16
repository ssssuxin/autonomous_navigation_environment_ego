from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction
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
    checkTerrainConn = LaunchConfiguration('checkTerrainConn', default='true')
    world_name = LaunchConfiguration('world_name', default='garage')
    vehicleHeight = LaunchConfiguration('vehicleHeight', default='0.75')
    terrainZ = LaunchConfiguration('terrainZ', default='0.0')
    vehicleYaw = LaunchConfiguration('vehicleYaw', default='0.0')
    gui = LaunchConfiguration('gui', default='false')

    gazebo_model_world_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), "launch",'turtlebot3_house.launch.py')
    local_planner_launch = os.path.join(get_package_share_directory('local_planner'), "launch",'local_planner.launch.py')
    terrain_analysis_launch = os.path.join(get_package_share_directory('terrain_analysis'), "launch",'terrain_analysis.launch.py')
    terrain_analysis_ext_launch = os.path.join(get_package_share_directory('terrain_analysis_ext'), "launch",'terrain_analysis_ext.launch.py')
    vehicle_simulator_launch = os.path.join(get_package_share_directory('vehicle_simulator'), "launch",'vehicle_simulator.launch.py')
    visualization_tools_launch = os.path.join(get_package_share_directory('visualization_tools'), "launch",'visualization_tools.launch.py')
    sensor_scan_generation_launch = os.path.join(get_package_share_directory('sensor_scan_generation'), "launch",'sensor_scan_generation.launch.py')
    # print("1111:   ",terrain_analysis_ext_launch)
    # print(other_launch_file,"!!!!!!!!!!!!")
    # 创建一个包含要启动的launch.py文件的LaunchDescription
    gazebo_model_world_launch_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_model_world_launch),
        # launch_arguments={'test_str': 'default_valuefsef3333e111','cameraOffsetZ': cameraOffsetZ,'goalX': vehicleX,'goalY':vehicleY}.items()
    )
    included_launch_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(local_planner_launch),
        launch_arguments={'test_str': 'default_valuefsef3333e111','cameraOffsetZ': cameraOffsetZ,'goalX': vehicleX,'goalY':vehicleY}.items()
    )
    included_launch_terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(terrain_analysis_launch),
        # launch_arguments={'checkTerrainConn':checkTerrainConn}.items()
        launch_arguments={
                          'registered_scan': LaunchConfiguration('registered_scan', default='registered_scan'),
                          }.items()
    )
    included_launch_terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(terrain_analysis_ext_launch),
        launch_arguments={'checkTerrainConn':checkTerrainConn,
                          'registered_scan': LaunchConfiguration('registered_scan', default='registered_scan'),
                          }.items()
    )
    included_launch_vehicle_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vehicle_simulator_launch),
        launch_arguments={'world_name':world_name,
                          'vehicleHeight':vehicleHeight,
                          'cameraOffsetZ':cameraOffsetZ,
                          'vehicleX':vehicleX,
                          'vehicleY':vehicleY,
                          'terrainZ':terrainZ,
                          'vehicleYaw':vehicleYaw,
                          'gui':gui,
                          }.items()
    )
    included_launch_sensor_scan_generation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_scan_generation_launch)
        # launch_arguments={'world_name':world_name}.items()
    )
    included_launch_visualization_tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visualization_tools_launch),
        launch_arguments={'world_name':world_name}.items()
    )
    rviz_para_dir = os.path.join(get_package_share_directory('vehicle_simulator'), 'config', 'tare_gazebo_env.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_para_dir]
    )

    # 创建主LaunchDescription
    ld = LaunchDescription()
    ld.add_action(gazebo_model_world_launch_local_planner)
    # 将包含的launch添加到主LaunchDescription中
    ld.add_action(included_launch_local_planner)
    # ld.add_action(node_action_sensorvehicle)
    # ld.add_action(node_action_sensorcamera)
    ld.add_action(included_launch_terrain_analysis)
    ld.add_action(included_launch_terrain_analysis_ext)
    ld.add_action(included_launch_vehicle_simulator)
    ld.add_action(included_launch_sensor_scan_generation)
    ld.add_action(included_launch_visualization_tools)
    # ld.add_action(rviz_node)
    timer_duration = 10.0  # Set the delay duration in seconds##延时启动节点
    ld.add_action(TimerAction(actions=[rviz_node], period=timer_duration))
    return ld

if __name__ == '__main__':
    generate_launch_description()
