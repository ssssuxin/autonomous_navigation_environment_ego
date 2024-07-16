# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
# from ament_index_python.packages import get_package_prefix
# from launch.substitutions import LaunchConfiguration
# from launch.actions import ExecuteProcess

def generate_launch_description():
    drone_id_ = "0"
    odometry_topic = "visual_slam/odom"
    real_time_input_pcl = '/local_pcl_map_for_egoxxx'#这里就是输入环境的信息
    real_time_input_pcl_lidar = '/pcl_scan_at_map'#这里给无人机加点实时的环境信息，让它反应快一点，因为上面那个更新较慢
    # real_time_input_for_terrain = real_time_input_pcl_lidar#旋转平移后的雷达数据
    map_size_x_ = LaunchConfiguration('map_size_x_', default='100.0')
    map_size_y_ = map_size_x_
    map_size_z_ = LaunchConfiguration('map_size_z_', default='30.0')
    local_update_range_x_y_ = 10.5#ego局部buff更新的尺寸
    local_update_range_z_ = 3.0
    rviz_para_dir = os.path.join(get_package_share_directory('ego_planner'), 'config', 'ros2_ego_rviz.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # output="log",
        arguments=['-d', rviz_para_dir]
    )
    
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        output='screen',
        name="drone_"+drone_id_+"_traj_server",
        # parameters=[local_planner_para_dir,]
        remappings=[
        # 重映射 position_cmd 话题
            ('position_cmd', "/drone_"+drone_id_+"_planning/pos_cmd"),

            # 重映射 planning/bspline 话题
            ('planning/bspline', "/drone_"+drone_id_+"_planning/bspline"),
            ],

    )

    advanced_param_launch = os.path.join(get_package_share_directory('ego_planner'), "launch",'advanced_param.launch.py')
    included_launch_advanced_param = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(advanced_param_launch),
        launch_arguments={
                            'drone_id':drone_id_,
                          'map_size_x_': map_size_x_,
                          'map_size_y_': map_size_y_,
                          'map_size_z_':map_size_z_,
                          'obj_num_set': LaunchConfiguration('obj_num_set', default='10'),
                          'camera_pose_topic': LaunchConfiguration('camera_pose_topic', default='pcl_render_node/camera_pose'),
                          'depth_topic': LaunchConfiguration('depth_topic', default='pcl_render_node/depth'),
                          'cloud_topic': LaunchConfiguration('cloud_topic', default='pcl_render_node/cloud'),
                          'cx': LaunchConfiguration('cx', default='321.04638671875'),
                          'cy': LaunchConfiguration('cy', default='243.44969177246094'),
                          'fx': LaunchConfiguration('fx', default='387.229248046875'),
                          'fy': LaunchConfiguration('fy', default='387.229248046875'),
                          'max_vel': LaunchConfiguration('max_vel', default='5.0'),
                          'max_acc': LaunchConfiguration('max_acc', default='2.5 '),
                        #   'planning_horizon': LaunchConfiguration('planning_horizon', default='3.5'),
                          'planning_horizon': LaunchConfiguration('planning_horizon', default='20.0'),
                          'use_distinctive_trajs': LaunchConfiguration('use_distinctive_trajs', default='True'),
                          'flight_type': LaunchConfiguration('flight_type', default='1'),
                          'point_num': LaunchConfiguration('point_num', default='4'),
                          'point0_x': LaunchConfiguration('point0_x', default='15.0'),
                          'point0_y': LaunchConfiguration('point0_y', default='0.0'),
                          'point0_z': LaunchConfiguration('point0_z', default='1.0'),
                          'point1_x': LaunchConfiguration('point1_x', default='-15.0 '),
                          'point1_y': LaunchConfiguration('point1_y', default='0.0 '),
                          'point1_z': LaunchConfiguration('point1_z', default='1.0 '),
                          'point2_x': LaunchConfiguration('point2_x', default='15.0 '),
                          'point2_y': LaunchConfiguration('point2_y', default='0.0 '),
                          'point2_z': LaunchConfiguration('point2_z', default='1.0 '),
                          'point3_x': LaunchConfiguration('point3_x', default='-15.0 '),
                          'point3_y': LaunchConfiguration('point3_y', default='0.0 '),
                          'point3_z': LaunchConfiguration('point3_z', default='1.0 '),
                          'point4_x': LaunchConfiguration('point4_x', default='15.0 '),
                          'point4_y': LaunchConfiguration('point4_y', default='0.0 '),
                          'point4_z': LaunchConfiguration('point4_z', default='1.0 '),
                          "name_of_ego_planner_node": LaunchConfiguration('name_of_ego_planner_node', default="drone_"+drone_id_+"_ego_planner_node"),
                          'odom_world': LaunchConfiguration('odom_world', default='/drone_'+drone_id_+'_'+odometry_topic),
                          'planning/bspline': LaunchConfiguration('planning/bspline', default='/drone_'+drone_id_+'_planning/bspline'),
                          'planning/data_display': LaunchConfiguration('planning/data_display', default='/drone_'+drone_id_+'_planning/data_display'),
                          'grid_map/odom': LaunchConfiguration('grid_map/odom', default='/drone_'+drone_id_+'_'+odometry_topic),#M?
                          # 'grid_map/odom': LaunchConfiguration('grid_map/odom', default='/state_estimation_at_scan'),
                          # 'grid_map/cloud': LaunchConfiguration('grid_map/cloud', default='/drone_'+drone_id_+'_pcl_render_node/cloud'),#M?
                          'grid_map/cloud': LaunchConfiguration('grid_map/cloud', default=real_time_input_pcl),
                          'grid_map/cloud_lidar': LaunchConfiguration('grid_map/cloud_lidar', default=real_time_input_pcl_lidar),
                          'grid_map/pose': LaunchConfiguration('grid_map/pose', default='/drone_'+drone_id_+'_pcl_render_node/camera_pose'),
                          'grid_map/depth': LaunchConfiguration('grid_map/depth', default='/drone_'+drone_id_+'_pcl_render_node/depth'),
                          'grid_map/occupancy_inflate': LaunchConfiguration('grid_map/occupancy_inflate', default='/drone_'+drone_id_+'_ego_planner_node/grid_map/occupancy_inflate'),
                          'optimal_list': LaunchConfiguration('optimal_list', default='/drone_'+drone_id_+'_ego_planner_node/optimal_list'),
                          'odometry': LaunchConfiguration('odometry', default='drone_'+drone_id_+'_'+odometry_topic),

                          'local_update_range_x_y_': LaunchConfiguration('local_update_range_x_y_', default=local_update_range_x_y_),
                          'local_update_range_z_': LaunchConfiguration('local_update_range_z_', default=local_update_range_z_),
                          
                          #TAG 这里实在没办法把字符串传过去再拼接，但是可以直接传过去直接做为那边函数的形参输入用，所以这里只能每次多输入点节点名  以及关于drone_id_的话题了
                          }.items()
    )

    simulator_launch = os.path.join(get_package_share_directory('ego_planner'), "launch",'simulator.launch.py')
    included_launch_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulator_launch),
        launch_arguments={
                            'drone_id':drone_id_,
                          'map_size_x_': map_size_x_,
                          'map_size_y_': map_size_y_,
                          'map_size_z_': map_size_z_,
                          'init_x_': LaunchConfiguration('init_x_', default='0.0'),
                          'init_y_': LaunchConfiguration('init_y_', default='0.0'),
                          'init_z_': LaunchConfiguration('init_z_', default='1.0'),
                          'odometry_topic': LaunchConfiguration('odometry_topic', default=odometry_topic),
                          'name_of_poscmd_2_odom': LaunchConfiguration('name_of_poscmd_2_odom', default='drone_'+drone_id_+'_poscmd_2_odom'),
                          'command': LaunchConfiguration('command', default='drone_'+drone_id_+'_planning/pos_cmd'),
                          'odometry': LaunchConfiguration('odometry', default='drone_'+drone_id_+'_'+odometry_topic),
                          'optimal_list': LaunchConfiguration('optimal_list', default='/drone_'+drone_id_+'_ego_planner_node/optimal_list'),

                          'name_of_odom_visualization': LaunchConfiguration('name_of_odom_visualization', default='drone_'+drone_id_+'_odom_visualization'),
                          'odom': LaunchConfiguration('odom', default='/drone_'+drone_id_+'_'+odometry_topic),#M?
                          # 'odom': LaunchConfiguration('odom', default='/state_estimation_at_scan'),
                          'robot': LaunchConfiguration('robot', default='/drone_'+drone_id_+'_odom_visualization/robot'),
                          'path': LaunchConfiguration('path', default='/drone_'+drone_id_+'_odom_visualization/path'),
                          
                          'name_of_local_sensing_node': LaunchConfiguration('name_of_local_sensing_node', default='drone_'+drone_id_+'_pcl_render_node'),
                          'pcl_render_node/cloud': LaunchConfiguration('pcl_render_node/cloud', default='drone_'+drone_id_+'_pcl_render_node/cloud'),

                          }.items()
    )
    local_pcl_map_provider_launch = os.path.join(get_package_share_directory('local_pcl_map_provider'), "launch",'local_pcl_map_provider.launch.py')
    included_launch_local_pcl_map_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(local_pcl_map_provider_launch),
        launch_arguments={
                            'odometry':LaunchConfiguration('odometry', default='drone_'+drone_id_+'_'+odometry_topic),
                            'localpclmap_x_and_y_size':LaunchConfiguration('localpclmap_x_and_y_size', default=map_size_x_),
                            'localpclmap_z_size':LaunchConfiguration('localpclmap_z_size', default=map_size_z_),
                            'local_update_range_x_y_': LaunchConfiguration('local_update_range_x_y_', default=local_update_range_x_y_),
                            'local_update_range_z_': LaunchConfiguration('local_update_range_z_', default=local_update_range_z_),
                            # 'map_size_x_': map_size_x_,
                          }.items()
    )


    ld = LaunchDescription()
    # ld.add_action(rviz_node_1)
    ld.add_action(rviz_node)
    ld.add_action(included_launch_advanced_param)
    ld.add_action(traj_server_node)
    ld.add_action(included_launch_simulator)
    ld.add_action(included_launch_local_pcl_map_provider)

    return ld
