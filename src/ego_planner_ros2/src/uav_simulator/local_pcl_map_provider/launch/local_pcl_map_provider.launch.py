import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
def generate_launch_description():

    local_pcl_map_provider_node = Node(
        package='local_pcl_map_provider',
        executable='local_pcl_map_provider',
        output='screen',
        parameters=[

                    {'localpclmap_x_and_y_size':LaunchConfiguration('localpclmap_x_and_y_size')},
                    {'localpclmap_z_size': LaunchConfiguration('localpclmap_z_size')},

                    {'ego_local_update_Range_x':LaunchConfiguration('local_update_range_x_y_')},
                    {'ego_local_update_Range_y':LaunchConfiguration('local_update_range_x_y_')},
                    {'ego_local_update_Range_z': LaunchConfiguration('local_update_range_z_')},
                    # {'triger_thred_for_xy':20.0},
                    # {'triger_thred_for_z': 20.0},
                    ],
        remappings=[
            ('odometry', LaunchConfiguration('odometry')),
        ],
        
    )
    #这里默认    输入的是雷达数据pcl_scan_at_map    输出 local_pcl_map_for_ego。 如果更改话题信息需要重映射


    ld = LaunchDescription()
    ld.add_action(local_pcl_map_provider_node)


    return ld
