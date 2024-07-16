import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
def generate_launch_description():
    show_public_param=False
    # 获取软件包路径
    # local_planner_dir = get_package_share_directory('local_planner')
    visualization_tools_para_dir = os.path.join(get_package_share_directory('visualization_tools'), 'config', 'visualizationTools.yaml')

    pkg_install_dir = get_package_prefix('visualization_tools')
    project_root = os.path.dirname(os.path.dirname(pkg_install_dir))  #这里获取了工程根目录路径
    vehicle_simulator_dir = os.path.join(project_root,'src','vehicle_simulator')
    # print(visualization_tools_para_dir)
    visualization_tools_node = Node(
        package='visualization_tools',
        executable='visualizationTools',
        output='screen',
        parameters=[visualization_tools_para_dir,#9
                    # {'pathFolder':pathFolder},
                    {'vehicle_simulator_dir': vehicle_simulator_dir},
                    # {'world_name': LaunchConfiguration('world_name')},
                    # {'sensorOffsetX': sensorOffsetX},
                    # {'sensorOffsetY': sensorOffsetY},
                    # {'twoWayDrive': twoWayDrive},
                    # {'maxSpeed': maxSpeed},
                    # {'autonomyMode': autonomyMode},
                    # {'autonomySpeed': autonomySpeed},
                    # {'joyToSpeedDelay': joyToSpeedDelay},
                    {'show_public_param': show_public_param},],
        remappings=[
        # 重映射 position_cmd 话题
            # ('/registered_scan', "/pcl_scan_at_map"),
            # ('/state_estimation', "/drone_0_visual_slam/odom"),
            ('/trajectory', "/trajectory_color"),
            # 重映射 planning/bspline 话题
            # ('planning/bspline', "/drone_"+drone_id_+"_planning/bspline"),
            ],
    )
    realTimePlot_node = Node(
        package='visualization_tools_py',
        executable='realTimePlot',
        output='screen',
    )

    # 创建LaunchDescription对象并添加节点
    ld = LaunchDescription()
    ld.add_action(visualization_tools_node)
    ld.add_action(realTimePlot_node)

    return ld
