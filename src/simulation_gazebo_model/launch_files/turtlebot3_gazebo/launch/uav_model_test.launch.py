import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    model_file_name = 'models/urdf_m100/' + 'm100' + '.gazebo'
    model = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                          model_file_name)
    print(model)
    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='firefly'),
        DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('mav_name')),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('tf_prefix', default_value=LaunchConfiguration('namespace')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('enable_logging', default_value='false'),
        DeclareLaunchArgument('enable_ground_truth', default_value='true'),
        DeclareLaunchArgument('log_file', default_value=LaunchConfiguration('mav_name')),
        DeclareLaunchArgument('wait_to_record_bag', default_value='false'),
        DeclareLaunchArgument('enable_mavlink_interface', default_value='false'),
        
        # Load robot description to parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('model')},
                        {'enable_logging': LaunchConfiguration('enable_logging')},
                        {'enable_ground_truth': LaunchConfiguration('enable_ground_truth')},
                        {'log_file': LaunchConfiguration('log_file')},
                        {'wait_to_record_bag': LaunchConfiguration('wait_to_record_bag')},
                        {'mav_name': LaunchConfiguration('mav_name')},
                        {'namespace': LaunchConfiguration('namespace')}]
        ),
        
        # Spawn model node
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                '-param', 'robot_description',
                '-urdf',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-model', LaunchConfiguration('namespace')
            ]
        )
    ])

