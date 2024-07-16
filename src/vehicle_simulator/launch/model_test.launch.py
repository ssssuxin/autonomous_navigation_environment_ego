from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',            # 这里的'my_robot'是你要加载到Gazebo中的实体名字
                '-file', '/home/suxin/English_Path/NEW_WORK_PATH/MY_FINAL_PJ/ROS2_Tare_gazebo_Environment/src/vehicle_simulator/urdf/new_robot.urdf', # 这里的'path/to/my_robot.urdf'需要替换成你的URDF文件的路径
            ],
            output='screen'
        )
    ])
