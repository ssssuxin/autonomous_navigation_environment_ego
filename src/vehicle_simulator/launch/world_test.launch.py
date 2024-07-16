# start_gazebo.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
def generate_launch_description():
    # 获取world文件路径
    vehicle_simulator_pkg = get_package_share_directory('vehicle_simulator')
    world_file = os.path.join(vehicle_simulator_pkg, 'world', 'garage.world')
    robot_xacro_file = os.path.join(vehicle_simulator_pkg, 'urdf', 'robot.urdf.xacro')



    return LaunchDescription([
        # Node(
        #     package="gazebo_ros",
        #     executable="gzserver",
        #     name="gazebo_server",
        #     output="screen"
        # )
        # ,
        # # # 启动Gazebo客户端
        # Node(
        #     package="gazebo_ros",
        #     executable="gzclient",
        #     name="gazebo_client",
        #     output="screen"
        # ),
        # 声明world文件路径的launch参数
        DeclareLaunchArgument(          ##M?这个东西用来干嘛的
            'world_file',
            default_value=world_file,
            description='Path to the Gazebo world file'
        ),
        # 启动Gazebo节点
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', world_file],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', robot_xacro_file],
            output='screen'
        ),
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_xacro_file
            
        }]
    )
    ])
##笔记
##这里世界能够加载出来了，不知道DeclareLaunchArgument怎么用
##模型暂时还没加载出来
