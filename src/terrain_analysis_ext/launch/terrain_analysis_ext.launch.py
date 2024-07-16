import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
def generate_launch_description():
    show_public_param=False
    # sensorOffsetX = 0.0
    # sensorOffsetY = 0.0
    # twoWayDrive = True
    # maxSpeed = 2.0
    # autonomyMode = True
    # autonomySpeed = 2.0
    # joyToSpeedDelay = 2.0
    # 获取软件包路径
    # local_planner_dir = get_package_share_directory('local_planner')
    terrain_analysis_ext_para_dir = os.path.join(get_package_share_directory('terrain_analysis_ext'), 'config', 'terrain_analysis_ext.yaml')
    # local_planner_para_dir_1 = os.path.join(get_package_share_directory('local_planner'), 'config', 'pathFollower.yaml')
    pkg_install_dir = get_package_prefix('terrain_analysis_ext')
    project_root = os.path.dirname(os.path.dirname(pkg_install_dir))  #这里获取了工程根目录路径
    # pathFolder = os.path.join(project_root,'src','local_planner','paths')
    terrainAnalysisExt_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        output='screen',
        parameters=[terrain_analysis_ext_para_dir,#9
                    # {'pathFolder':pathFolder},
                    {'checkTerrainConn': LaunchConfiguration('checkTerrainConn')},
                    # {'goalY': LaunchConfiguration('goalY')},
                    # {'sensorOffsetX': sensorOffsetX},
                    # {'sensorOffsetY': sensorOffsetY},
                    # {'twoWayDrive': twoWayDrive},
                    # {'maxSpeed': maxSpeed},
                    # {'autonomyMode': autonomyMode},
                    # {'autonomySpeed': autonomySpeed},
                    # {'joyToSpeedDelay': joyToSpeedDelay},
                    
                    {'show_public_param': show_public_param},
                    ],
                    
        remappings=[
        # 重映射 position_cmd 话题
            ('registered_scan', LaunchConfiguration('registered_scan')),
            ('state_estimation', LaunchConfiguration('state_estimation')),
            ],
    )
    # path_follower_node = Node(
    #     package='local_planner',
    #     executable='pathFollower',
    #     output='screen',
    #     parameters=[local_planner_para_dir_1,#7
    #                 # {'sensorOffsetX': sensorOffsetX},
    #                 # {'sensorOffsetY': sensorOffsetY},
    #                 # {'twoWayDrive': twoWayDrive},
    #                 # {'maxSpeed': maxSpeed},
    #                 # {'autonomyMode': autonomyMode},
    #                 # {'autonomySpeed': autonomySpeed},
    #                 # {'joyToSpeedDelay': joyToSpeedDelay},
    #                 # {'show_public_param': show_public_param}
    #                 ]
    # )
    # 创建LaunchDescription对象并添加节点
    ld = LaunchDescription()
    ld.add_action(terrainAnalysisExt_node)
    # ld.add_action(path_follower_node)

    return ld
