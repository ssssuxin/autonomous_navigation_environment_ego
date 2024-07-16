#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
using namespace std;

const double PI = 3.1415926;

string metricFile;
string trajFile;
string mapFile;
string vehicle_simulator_dir;
string world_name;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

sensor_msgs::msg::PointCloud2 overallMap2;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExploredAreaPtr = NULL;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrajectoryPtr = NULL;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubExploredVolumePtr = NULL;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTravelingDisPtr = NULL;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTimeDurationPtr = NULL;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;

void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  systemTime = odom->header.stamp.sec;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::msg::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  travelingDis += dis;

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::msg::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::msg::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::msg::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

void runtimeHandler(const std_msgs::msg::Float32::SharedPtr runtimeIn)
{
  runtime = runtimeIn->data;
}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("visualizationTools");//这个只是说节点的名字叫这个

  node_handle->declare_parameter<std::string>("vehicle_simulator_dir","/home/suxin/English_Path/NEW_WORK_PATH/MY_FINAL_PJ/ROS1_tare_gazebo_environment/src/vehicle_simulator");
  node_handle->declare_parameter<std::string>("world_name","garage");
  // node_handle->declare_parameter<std::string>("mapFile","/home/suxin/English_Path/NEW_WORK_PATH/MY_FINAL_PJ/ROS1_tare_gazebo_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply");
  node_handle->declare_parameter<double>("overallMapVoxelSize",0.5);
  node_handle->declare_parameter<double>("exploredAreaVoxelSize",0.3);
  node_handle->declare_parameter<double>("exploredVolumeVoxelSize",0.5);
  node_handle->declare_parameter<double>("transInterval",0.2);
  node_handle->declare_parameter<double>("yawInterval",10.0);
  node_handle->declare_parameter<int>("overallMapDisplayInterval",2);
  node_handle->declare_parameter<int>("exploredAreaDisplayInterval",1);

  node_handle->get_parameter("vehicle_simulator_dir",vehicle_simulator_dir);
  node_handle->get_parameter("world_name",world_name);
  // node_handle->get_parameter("mapFile",mapFile);
  node_handle->get_parameter("overallMapVoxelSize",overallMapVoxelSize);
  node_handle->get_parameter("exploredAreaVoxelSize",exploredAreaVoxelSize);
  node_handle->get_parameter("exploredVolumeVoxelSize",exploredVolumeVoxelSize);
  node_handle->get_parameter("transInterval",transInterval);
  node_handle->get_parameter("yawInterval",yawInterval);
  node_handle->get_parameter("overallMapDisplayInterval",overallMapDisplayInterval);
  node_handle->get_parameter("exploredAreaDisplayInterval",exploredAreaDisplayInterval);
  metricFile = vehicle_simulator_dir+"/log/metrics";
  trajFile = vehicle_simulator_dir+"/log/trajectory";
  mapFile = vehicle_simulator_dir+ "/mesh/" + world_name + "/preview/pointcloud.ply";
  bool test_param=0;
  node_handle->declare_parameter<bool>("test_param",0);
  node_handle->get_parameter("test_param",test_param);
  if(!test_param)
  std::cout<<"visualizationTools获取参数失败！ "<<std::endl;
  node_handle->declare_parameter<bool>("show_public_param",false);
  bool show_public_param;
  node_handle->get_parameter("show_public_param",show_public_param);
  if(show_public_param)
  {
    std::cout<<"overallMapVoxelSize: "<<overallMapVoxelSize<<std::endl;
    std::cout<<"exploredAreaVoxelSize: "<<exploredAreaVoxelSize<<std::endl;
    std::cout<<"exploredVolumeVoxelSize: "<<exploredVolumeVoxelSize<<std::endl;
    std::cout<<"transInterval: "<<transInterval<<std::endl;
    std::cout<<"yawInterval: "<<yawInterval<<std::endl;
    std::cout<<"overallMapDisplayInterval: "<<overallMapDisplayInterval<<std::endl;
    std::cout<<"exploredAreaDisplayInterval: "<<exploredAreaDisplayInterval<<std::endl;
    // std::cout<<"goalX: "<<goalX<<std::endl;
    // std::cout<<"goalY: "<<goalY<<std::endl;
  }

  // nhPrivate.getParam("metricFile", metricFile);
  // nhPrivate.getParam("trajFile", trajFile);
  // nhPrivate.getParam("mapFile", mapFile);
  // nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  // nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  // nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  // nhPrivate.getParam("transInterval", transInterval);
  // nhPrivate.getParam("yawInterval", yawInterval);
  // nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  // nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);
  auto subOdometry = node_handle->create_subscription<nav_msgs::msg::Odometry>("/state_estimation",5,odometryHandler);

  // ros::Subscriber subOdometry = nh.subscribe<nav_msgs::msg::Odometry> ("/state_estimation", 5, odometryHandler);
  auto subLaserCloud = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan",5,laserCloudHandler);

  // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::msg::PointCloud2> ("/registered_scan", 5, laserCloudHandler);
  auto subRuntime = node_handle->create_subscription<std_msgs::msg::Float32>("/runtime",5,runtimeHandler);

  // ros::Subscriber subRuntime = nh.subscribe<std_msgs::msg::Float32> ("/runtime", 5, runtimeHandler);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOverallMap
                    = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 5);

  // ros::Publisher pubOverallMap = nh.advertise<sensor_msgs::msg::PointCloud2> ("/overall_map", 5);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExploredArea
                    = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/explored_areas", 5);

  // ros::Publisher pubExploredArea = nh.advertise<sensor_msgs::msg::PointCloud2> ("/explored_areas", 5);

  pubExploredAreaPtr = pubExploredArea;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrajectory
                    = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/trajectory", 5);

  // ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::msg::PointCloud2> ("/trajectory", 5);
  pubTrajectoryPtr = pubTrajectory;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubExploredVolume
                    = node_handle->create_publisher<std_msgs::msg::Float32>("/explored_volume", 5);
  // ros::Publisher pubExploredVolume = nh.advertise<std_msgs::msg::Float32> ("/explored_volume", 5);
  pubExploredVolumePtr = pubExploredVolume;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTravelingDis
                    = node_handle->create_publisher<std_msgs::msg::Float32>("/traveling_distance", 5);
  // ros::Publisher pubTravelingDis = nh.advertise<std_msgs::msg::Float32> ("/traveling_distance", 5);
  pubTravelingDisPtr = pubTravelingDis;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTimeDuration
                    = node_handle->create_publisher<std_msgs::msg::Float32>("/time_duration", 5);
  // ros::Publisher pubTimeDuration = nh.advertise<std_msgs::msg::Float32> ("/time_duration", 5);
  pubTimeDurationPtr = pubTimeDuration;

  //ros::Publisher pubRuntime = nh.advertise<std_msgs::msg::Float32> ("/runtime", 5);

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    printf("\nCouldn't read pointcloud.ply file.\n\n");
  }

  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  metricFile += "_" + timeString + ".txt";
  trajFile += "_" + timeString + ".txt";
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();;
  while (status) {
    rclcpp::spin_some(node_handle);

    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      //M? 不知道这里时钟会不会出问题
      rclcpp::Time timestamp_1(static_cast<int64_t>(systemTime * 1000000000));
      overallMap2.header.stamp = timestamp_1;
      overallMap2.header.frame_id = "map";
      pubOverallMap->publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    status = rclcpp::ok();;
    rate.sleep();
  }

  fclose(metricFilePtr);
  fclose(trajFilePtr);

  printf("\nExploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.\n\n");

  return 0;
}
