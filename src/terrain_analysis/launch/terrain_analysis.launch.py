import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
def generate_launch_description():
    show_public_param=False
    terrain_analysis_para_dir = os.path.join(get_package_share_directory('terrain_analysis'), 'config', 'terrain_analysis.yaml')
    # local_planner_para_dir_1 = os.path.join(get_package_share_directory('local_planner'), 'config', 'pathFollower.yaml')
    # pkg_install_dir = get_package_prefix('terrain_analysis')
    # project_root = os.path.dirname(os.path.dirname(pkg_install_dir))  #这里获取了工程根目录路径
    # pathFolder = os.path.join(project_root,'src','local_planner','paths')
    print(terrain_analysis_para_dir)
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        output='screen',
        parameters=[terrain_analysis_para_dir,
                    {'show_public_param':show_public_param}],
        remappings=[
        # 重映射 position_cmd 话题
            ('registered_scan', LaunchConfiguration('registered_scan')),
            ('state_estimation', LaunchConfiguration('state_estimation')),
            ],
    )
    # 创建LaunchDescription对象并添加节点
    ld = LaunchDescription()
    ld.add_action(terrain_analysis_node)
    # ld.add_action(path_follower_node)

    return ld
