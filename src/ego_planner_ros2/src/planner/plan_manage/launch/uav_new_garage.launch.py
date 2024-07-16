from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    
    
    world_name = LaunchConfiguration('world_name', default='garage')


    gazebo_model_world_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), "launch",'uav_and_env_new_garage.launch.py')
    # local_planner_launch = os.path.join(get_package_share_directory('local_planner'), "launch",'local_planner.launch.py')
    
    # vehicle_simulator_launch = os.path.join(get_package_share_directory('vehicle_simulator'), "launch",'vehicle_simulator.launch.py')
    # visualization_tools_launch = os.path.join(get_package_share_directory('visualization_tools'), "launch",'visualization_tools.launch.py')
    # sensor_scan_generation_launch = os.path.join(get_package_share_directory('sensor_scan_generation'), "launch",'sensor_scan_generation.launch.py')
    single_run_in_sim_launch = os.path.join(get_package_share_directory('ego_planner'), "launch",'single_run_in_sim.launch.py')
    visualization_tools_launch = os.path.join(get_package_share_directory('visualization_tools'), "launch",'visualization_tools.launch.py')

    gazebo_model_world_launch_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_model_world_launch),
    )
    # included_launch_local_planner = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(local_planner_launch),
    #     launch_arguments={'test_str': 'default_valuefsef3333e111','cameraOffsetZ': cameraOffsetZ,'goalX': vehicleX,'goalY':vehicleY}.items()
    # )
    

    included_launch_single_run_in_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_run_in_sim_launch),
        launch_arguments={
                          }.items()
    )
    # included_launch_visualization_tools = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(visualization_tools_launch),
    #     launch_arguments={'world_name':world_name}.items()
    # )
    # included_launch_vehicle_simulator = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(vehicle_simulator_launch),
    #     launch_arguments={'world_name':world_name,
    #                       'vehicleHeight':vehicleHeight,
    #                       'cameraOffsetZ':cameraOffsetZ,
    #                       'vehicleX':vehicleX,
    #                       'vehicleY':vehicleY,
    #                       'terrainZ':terrainZ,
    #                       'vehicleYaw':vehicleYaw,
    #                       'gui':gui,
    #                       }.items()
    # )
    # included_launch_sensor_scan_generation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(sensor_scan_generation_launch)

    # )
    # included_launch_visualization_tools = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(visualization_tools_launch),
    #     launch_arguments={'world_name':world_name}.items()
    # )
    rviz_para_dir = os.path.join(get_package_share_directory('ego_planner'), 'config', 'tare_gazebo_env.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_para_dir]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_model_world_launch_local_planner)
    # ld.add_action(included_launch_local_planner)
    
    # ld.add_action(included_launch_vehicle_simulator)
    # ld.add_action(included_launch_sensor_scan_generation)
    # ld.add_action(included_launch_visualization_tools)
    timer_duration = 5.0
    ld.add_action(TimerAction(actions=[included_launch_single_run_in_sim], period=timer_duration))
    # ld.add_action()
      # Set the delay duration in seconds##延时启动节点
    # ld.add_action(included_launch_visualization_tools)
    ld.add_action(TimerAction(actions=[rviz_node], period=timer_duration))
    return ld

if __name__ == '__main__':
    generate_launch_description()
