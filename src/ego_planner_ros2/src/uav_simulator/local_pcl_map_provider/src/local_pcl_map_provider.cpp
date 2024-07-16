
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/crop_box.h>
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> Filter_0p3;
pcl::CropBox<pcl::PointXYZI> cropBoxFilter;
pcl::CropBox<pcl::PointXYZI> cropBoxFilter_local_to_egolocal;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcl_map(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr local_to_egolocal(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr global_pcl_map(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr local_get_from_global(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr local_handin_for_global(new pcl::PointCloud<pcl::PointXYZI>());
// pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pcl_map_pub_ptr = NULL;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pcl_map_pub_ptr = NULL;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_get_from_global_vis = NULL;
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_handin_for_global_vis = NULL;
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_get_from_global_vis = NULL;
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_handin_for_global_vis = NULL;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_map_signal_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_change_origin;
int exploredAreaDisplayCount;
int exploredAreaDisplayInterval;
double localpclmap_x_and_y_size,localpclmap_z_size,triger_thred_for_xy,triger_thred_for_z;
double exploredAreaVoxelSize;
double ego_local_update_Range_x,ego_local_update_Range_y,ego_local_update_Range_z;
bool do_flag=true;
int count_to_ban_flag=0;
nav_msgs::msg::Odometry odom;
nav_msgs::msg::Odometry last_odom;


void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  odom = *odom_msg;
  if(abs(odom.pose.pose.position.x-last_odom.pose.pose.position.x)>(localpclmap_x_and_y_size/2)||
     abs(odom.pose.pose.position.y-last_odom.pose.pose.position.y)>(localpclmap_x_and_y_size/2)||
     abs(odom.pose.pose.position.z-last_odom.pose.pose.position.z)>(localpclmap_z_size/2)   )
        {
          last_odom=odom;
          // 在这里写local和global的交换
          //local上交，并且global下采样
          *global_pcl_map = *global_pcl_map+*local_pcl_map;
          Filter_0p3.setInputCloud(global_pcl_map);
          Filter_0p3.filter(*global_pcl_map);
          //local保留还在方框内的点云，
          Eigen::Vector4f minPoint(odom.pose.pose.position.x-localpclmap_x_and_y_size/2, odom.pose.pose.position.y-localpclmap_x_and_y_size/2, odom.pose.pose.position.z-localpclmap_z_size/2, 1.0); // 最小点坐标
          Eigen::Vector4f maxPoint(odom.pose.pose.position.x+localpclmap_x_and_y_size/2, odom.pose.pose.position.y+localpclmap_x_and_y_size/2, odom.pose.pose.position.z+localpclmap_z_size/2, 1.0);    // 最大点坐标
          cropBoxFilter.setMin(minPoint);
          cropBoxFilter.setMax(maxPoint);
          cropBoxFilter.setInputCloud(local_pcl_map);
          cropBoxFilter.filter(*local_pcl_map);
          //global下放，并且local下采样
          cropBoxFilter.setInputCloud(global_pcl_map);
          cropBoxFilter.filter(*local_get_from_global);
          *local_pcl_map = *local_pcl_map+*local_get_from_global;
          exploredAreaDwzFilter.setInputCloud(local_pcl_map);
          exploredAreaDwzFilter.filter(*local_pcl_map);
          sensor_msgs::msg::PointCloud2 local_get_from_global_msg;
          sensor_msgs::msg::PointCloud2 global_msg;
          
          pcl::toROSMsg(*local_get_from_global, local_get_from_global_msg);
          local_get_from_global_msg.header.frame_id = "map";
          

          
          pcl::toROSMsg(*global_pcl_map, global_msg);
          global_msg.header.frame_id = "map";
          local_get_from_global_vis->publish(local_get_from_global_msg);
          global_pcl_map_pub_ptr->publish(global_msg);
          // std::cout<<"交换"<<std::endl;
        }

    

    
}
void clear_map_signal_call_cack(const std_msgs::msg::Bool::SharedPtr msg)
{
  laserCloud->clear();
  local_pcl_map->clear();
  local_to_egolocal->clear();
  global_pcl_map->clear();
  global_pcl_map->clear();
  local_get_from_global->clear();
  local_handin_for_global->clear();
}
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn)
{
  
  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);


  *local_pcl_map += *laserCloud;
  exploredAreaDisplayCount++;
  // std::cout<<"local_pcl_map:    "<<local_pcl_map->points.size()<<std::endl;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval || do_flag) {
    // exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(local_pcl_map);
    exploredAreaDwzFilter.filter(*local_pcl_map);
    // std::cout<<"exploredAreaCloud2: "<<local_pcl_map->points.size()<<std::endl;
    // local_pcl_map = exploredAreaCloud2;
    Eigen::Vector4f minPoint(odom.pose.pose.position.x-ego_local_update_Range_x, odom.pose.pose.position.y-ego_local_update_Range_y, odom.pose.pose.position.z-ego_local_update_Range_z, 1.0); // 最小点坐标
    Eigen::Vector4f maxPoint(odom.pose.pose.position.x+ego_local_update_Range_x, odom.pose.pose.position.y+ego_local_update_Range_y, odom.pose.pose.position.z+ego_local_update_Range_z, 1.0);    // 最大点坐标
    cropBoxFilter_local_to_egolocal.setMin(minPoint);
    cropBoxFilter_local_to_egolocal.setMax(maxPoint);
    cropBoxFilter_local_to_egolocal.setInputCloud(local_pcl_map);
    cropBoxFilter_local_to_egolocal.filter(*local_to_egolocal);

    
    cropBoxFilter_local_to_egolocal.setNegative(true);
    Eigen::Vector4f min_point (odom.pose.pose.position.x-1.4, odom.pose.pose.position.y-1.4, odom.pose.pose.position.z-1.4, 1.0); // Define minimum point coordinates
    Eigen::Vector4f max_point (odom.pose.pose.position.x+1.4, odom.pose.pose.position.y+1.4, odom.pose.pose.position.z+1.4, 1.0);    // Define maximum point coordinates
    cropBoxFilter_local_to_egolocal.setInputCloud (local_to_egolocal);
    cropBoxFilter_local_to_egolocal.setMin (min_point);
    cropBoxFilter_local_to_egolocal.setMax (max_point);
    cropBoxFilter_local_to_egolocal.filter(*local_to_egolocal);


    sensor_msgs::msg::PointCloud2 local_pcl_map_msg;
    pcl::toROSMsg(*local_to_egolocal, local_pcl_map_msg);
    local_pcl_map_msg.header.stamp = laserCloudIn->header.stamp;
    local_pcl_map_msg.header.frame_id = "map";
    local_pcl_map_pub_ptr->publish(local_pcl_map_msg);
    exploredAreaDisplayCount = 0;
    cropBoxFilter_local_to_egolocal.setNegative(false);


    if(do_flag)//起初50次都下采样并发布出去给无人机，毕竟刚开始没有点云，要收集附近点云
    {
      count_to_ban_flag++;
      if(count_to_ban_flag>=15)
      {
        do_flag=false;
        std::cout<<"无人机已出新手村，不再帧帧下采样发布"<<std::endl;
      }
    }
  }
}


int main(int argc, char** argv)
{
//收集点云，并且
//分两组，一组全局，一组局部。   1。局部负责频繁收集数据并下采样， （ 无人机每突破一定距离就找全局补）
//                          2.定期把局部的全部上交给全局并全局下采样（局部先下采样再上交）
//                          3.局部裁剪（必须是上交了才裁剪，裁剪是在比“局部更大的一个范围外的点云被裁剪”）
//
//
  odom.pose.pose.position.x=0;
  odom.pose.pose.position.y=0;
  odom.pose.pose.position.z=1;
  last_odom=odom;
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("visualizationTools");//这个只是说节点的名字叫这个

  node_handle->declare_parameter<double>("localpclmap_x_and_y_size",100);
  node_handle->declare_parameter<double>("localpclmap_z_size",30);
  node_handle->declare_parameter<double>("triger_thred_for_xy",50);
  node_handle->declare_parameter<double>("triger_thred_for_z",15);
  node_handle->declare_parameter<double>("exploredAreaVoxelSize",0.1);
  node_handle->declare_parameter<double>("ego_local_update_Range_x",10);
  node_handle->declare_parameter<double>("ego_local_update_Range_y",10);
  node_handle->declare_parameter<double>("ego_local_update_Range_z",10);

  node_handle->get_parameter("localpclmap_x_and_y_size",localpclmap_x_and_y_size);//局部pcl不变尺寸
  node_handle->get_parameter("localpclmap_z_size",localpclmap_z_size);//局部pcl不变尺寸
  node_handle->get_parameter("triger_thred_for_xy",triger_thred_for_xy);
  node_handle->get_parameter("triger_thred_for_z",triger_thred_for_z);
  node_handle->get_parameter("exploredAreaVoxelSize",exploredAreaVoxelSize);//栅格大小
  node_handle->get_parameter("ego_local_update_Range_x",ego_local_update_Range_x);
  node_handle->get_parameter("ego_local_update_Range_y",ego_local_update_Range_y);//传送给ego的尺度大小
  node_handle->get_parameter("ego_local_update_Range_z",ego_local_update_Range_z);//传送给ego的尺度大小
  ego_local_update_Range_x=ego_local_update_Range_x+4;//机器人移动得比发布得快，补偿发布滞后机器人移动，多加一点区域让机器人至少前方的区域>=设定区域
  ego_local_update_Range_y=ego_local_update_Range_y+4;
  local_pcl_map_pub_ptr = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/local_pcl_map_for_ego", 5);
  global_pcl_map_pub_ptr = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pcl_map", 5);
  local_get_from_global_vis = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/local_get_from_global", 5);
  odom_sub_change_origin = 
      node_handle->create_subscription<nav_msgs::msg::Odometry>("/odometry", 1, odom_callback);
  //pcl_scan_at_map这个是雷达数据
  subLaserCloud = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>("/pcl_scan_at_map",5,laserCloudHandler);
  clear_map_signal_sub = node_handle->create_subscription<std_msgs::msg::Bool>("/clear_map_signal",5,clear_map_signal_call_cack);
  exploredAreaDisplayInterval=1;
  exploredAreaDisplayCount=5;
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  Filter_0p3.setLeafSize(0.3, 0.3, 0.3);
  std::cout<<exploredAreaVoxelSize<<std::endl;
  rclcpp::spin(node_handle);
}
