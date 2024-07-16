#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include <gazebo_msgs/ModelState.h>
#include "gazebo_msgs/msg/model_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joy.hpp"
// #include <visualization_msgs/Marker.h>
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gazebo_msgs/srv/set_entity_state.hpp"
using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;
double cameraOffsetZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = false;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = false;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

const int systemDelay = 5;
int systemInitCount = 0;
bool systemInited = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

std::vector<int> scanInd;

rclcpp::Time odomTime;
rclcpp::Clock _clock;
float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
float terrainRollStack[stackNum];
float terrainPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScanPointer = NULL;

void scanHandler(const sensor_msgs::msg::PointCloud2::SharedPtr scanIn)
{
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
    }
    return;
  }

  double scanTime = scanIn->header.stamp.sec;

  if (odomSendIDPointer < 0)
  {
    return;
  }
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
  {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  double odomRecTime = odomTime.seconds();
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float vehicleRecRoll = vehicleRoll;
  float vehicleRecPitch = vehiclePitch;
  float vehicleRecYaw = vehicleYaw;
  float terrainRecRoll = terrainRoll;
  float terrainRecPitch = terrainPitch;

  if (use_gazebo_time)
  {
    odomRecTime = odomTimeStack[odomRecIDPointer];
    vehicleRecX = vehicleXStack[odomRecIDPointer];
    vehicleRecY = vehicleYStack[odomRecIDPointer];
    vehicleRecZ = vehicleZStack[odomRecIDPointer];
    vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
    vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
    vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
    terrainRecRoll = terrainRollStack[odomRecIDPointer];
    terrainRecPitch = terrainPitchStack[odomRecIDPointer];
  }

  float sinTerrainRecRoll = sin(terrainRecRoll);
  float cosTerrainRecRoll = cos(terrainRecRoll);
  float sinTerrainRecPitch = sin(terrainRecPitch);
  float cosTerrainRecPitch = cos(terrainRecPitch);

  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

  int scanDataSize = scanData->points.size();
  for (int i = 0; i < scanDataSize; i++)
  {
    float pointX1 = scanData->points[i].x;
    float pointY1 = scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
    float pointZ1 = scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

    float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    float pointY2 = pointY1;
    float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    float pointX3 = pointX2 + vehicleRecX;
    float pointY3 = pointY2 + vehicleRecY;
    float pointZ3 = pointZ2 + vehicleRecZ;

    scanData->points[i].x = pointX3;
    scanData->points[i].y = pointY3;
    scanData->points[i].z = pointZ3;
  }

  // publish 5Hz registered scan messages
  sensor_msgs::msg::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);

  rclcpp::Time timestamp_(static_cast<int64_t>(odomRecTime * 1000000000));
  // path.header.stamp = timestamp_;
  scanData2.header.stamp = timestamp_;
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);
}

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr terrainCloud2)
{
  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  pcl::PointXYZI point;
  terrainCloudIncl->clear();
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0;
  int elevCount = 0;
  bool terrainValid = true;
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

    if (dis < terrainRadiusZ)
    {
      if (point.intensity < groundHeightThre)
      {
        elevMean += point.z;
        elevCount++;
      }
      else
      {
        terrainValid = false;
      }
    }

    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
    {
      terrainCloudIncl->push_back(point);
    }
  }

  if (elevCount >= minTerrainPointNumZ)
    elevMean /= elevCount;
  else
    terrainValid = false;

  if (terrainValid && adjustZ)
  {
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  terrainCloudDwz->clear();
  terrainDwzFilter.setInputCloud(terrainCloudIncl);
  terrainDwzFilter.filter(*terrainCloudDwz);
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
  {
    return;
  }

  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

  int inlierNum = 0;
  matX.at<float>(0, 0) = terrainPitch;
  matX.at<float>(1, 0) = terrainRoll;
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      matA.at<float>(i, 0) = -point.x + vehicleX;
      matA.at<float>(i, 1) = point.y - vehicleY;
      matB.at<float>(i, 0) = point.z - elevMean;

      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0)
      {
        matA.at<float>(i, 0) = 0;
        matA.at<float>(i, 1) = 0;
        matB.at<float>(i, 0) = 0;
        outlierCount++;
      }
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (inlierNum == terrainCloudDwzSize - outlierCount)
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
  {
    terrainValid = false;
  }

  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }
}

void speedHandler(const geometry_msgs::msg::TwistStamped::SharedPtr speedIn)
{
  vehicleSpeed = speedIn->twist.linear.x;
  vehicleYawRate = speedIn->twist.angular.z;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("vehicleSimulator");//这个只是说节点的名字叫这个
  auto client = node_handle->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
  node_handle->declare_parameter<bool>("use_gazebo_time",false);
  node_handle->declare_parameter<double>("cameraOffsetZ",0.0);
  node_handle->declare_parameter<double>("sensorOffsetX",0.0);
  node_handle->declare_parameter<double>("sensorOffsetY",0.0);
  node_handle->declare_parameter<double>("vehicleHeight",0.75);
  node_handle->declare_parameter<double>("vehicleX",0.0);
  node_handle->declare_parameter<double>("vehicleY",0.0);
  node_handle->declare_parameter<double>("vehicleZ",0.0);
  node_handle->declare_parameter<double>("terrainZ",0.0);
  node_handle->declare_parameter<double>("vehicleYaw",0.0);
  node_handle->declare_parameter<double>("terrainVoxelSize",0.05);
  node_handle->declare_parameter<double>("groundHeightThre",0.1);
  node_handle->declare_parameter<bool>("adjustZ",true);
  node_handle->declare_parameter<double>("terrainRadiusZ",1.0);
  node_handle->declare_parameter<int>("minTerrainPointNumZ",5);
  node_handle->declare_parameter<bool>("adjustIncl",true);
  node_handle->declare_parameter<double>("terrainRadiusIncl",2.0);
  node_handle->declare_parameter<int>("minTerrainPointNumIncl",200);
  node_handle->declare_parameter<double>("InclFittingThre",0.2);
  node_handle->declare_parameter<double>("maxIncl",30.0);

  node_handle->get_parameter("use_gazebo_time",use_gazebo_time);
  node_handle->get_parameter("cameraOffsetZ",cameraOffsetZ);
  node_handle->get_parameter("sensorOffsetX",sensorOffsetX);
  node_handle->get_parameter("sensorOffsetY",sensorOffsetY);
  node_handle->get_parameter("vehicleHeight",vehicleHeight);
  node_handle->get_parameter("vehicleX",vehicleX);
  node_handle->get_parameter("vehicleY",vehicleY);
  node_handle->get_parameter("vehicleZ",vehicleZ);
  node_handle->get_parameter("terrainZ",terrainZ);
  node_handle->get_parameter("vehicleYaw",vehicleYaw);
  node_handle->get_parameter("terrainVoxelSize",terrainVoxelSize);
  node_handle->get_parameter("groundHeightThre",groundHeightThre);
  node_handle->get_parameter("adjustZ",adjustZ);
  node_handle->get_parameter("terrainRadiusZ",terrainRadiusZ);
  node_handle->get_parameter("minTerrainPointNumZ",minTerrainPointNumZ);
  node_handle->get_parameter("adjustIncl",adjustIncl);
  node_handle->get_parameter("terrainRadiusIncl",terrainRadiusIncl);
  node_handle->get_parameter("minTerrainPointNumIncl",minTerrainPointNumIncl);
  node_handle->get_parameter("InclFittingThre",InclFittingThre);
  node_handle->get_parameter("maxIncl",maxIncl);

  bool test_param=0;
  node_handle->declare_parameter<bool>("test_param",0);
  node_handle->get_parameter("test_param",test_param);
  if(!test_param)
  std::cout<<"vehicleSimulator获取参数失败！ "<<std::endl;
  node_handle->declare_parameter<bool>("show_public_param",false);
  bool show_public_param;
  node_handle->get_parameter("show_public_param",show_public_param);
  if(show_public_param)
  {
    std::cout<<"vehicleHeight: "<<vehicleHeight<<std::endl;
    std::cout<<"cameraOffsetZ: "<<cameraOffsetZ<<std::endl;
    std::cout<<"vehicleX: "<<vehicleX<<std::endl;
    std::cout<<"vehicleY: "<<vehicleY<<std::endl;
    std::cout<<"terrainZ: "<<terrainZ<<std::endl;
    std::cout<<"vehicleYaw: "<<vehicleYaw<<std::endl;
    // std::cout<<"twoWayDrive: "<<twoWayDrive<<std::endl;
    // std::cout<<"maxSpeed: "<<maxSpeed<<std::endl;
    // std::cout<<"autonomyMode: "<<autonomyMode<<std::endl;
    // std::cout<<"autonomySpeed: "<<autonomySpeed<<std::endl;
    // std::cout<<"joyToSpeedDelay: "<<joyToSpeedDelay<<std::endl;
    // std::cout<<"goalX: "<<goalX<<std::endl;
    // std::cout<<"goalY: "<<goalY<<std::endl;
  }
  auto subScan = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points/out",2,scanHandler);

  // ros::Subscriber subScan = nh.subscribe<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2, scanHandler);
  auto subTerrainCloud = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map",2,terrainCloudHandler);
  // ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::msg::PointCloud2>("/terrain_map", 2, terrainCloudHandler);
  auto subSpeed = node_handle->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel",5,speedHandler);
  // ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, speedHandler);
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubVehicleOdom = node_handle->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);
  // ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::msg::Odometry>("/state_estimation", 5);

  nav_msgs::msg::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";

  // tf::TransformBroadcaster tfBroadcaster;
  tf2_ros::TransformBroadcaster tfBroadcaster(node_handle);
  // tf::StampedTransform odomTrans;
  geometry_msgs::msg::TransformStamped odomTrans;
  // odomTrans.frame_id_ = "map";
  odomTrans.header.frame_id = "map";
  odomTrans.child_frame_id = "sensor";
  
  rclcpp::Publisher<gazebo_msgs::msg::ModelState>::SharedPtr pubModelState=node_handle->create_publisher<gazebo_msgs::msg::ModelState>("/gazebo/set_model_state", 5);
  // ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
  // gazebo_msgs::ModelState cameraState;
  gazebo_msgs::msg::ModelState cameraState;
  cameraState.model_name = "camera";
  gazebo_msgs::msg::ModelState lidarState;
  lidarState.model_name = "lidar";
  gazebo_msgs::msg::ModelState robotState;
  robotState.model_name = "robot";

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScan=node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 2);

  // ros::Publisher pubScan = nh.advertise<sensor_msgs::msg::PointCloud2>("/registered_scan", 2);
  pubScanPointer = pubScan;

  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  printf("\nSimulation started.\n\n");

  rclcpp::Rate rate(200);
  bool status = rclcpp::ok();;
  std::this_thread::sleep_for(std::chrono::seconds(5));
  while (status)
  {
    rclcpp::spin_some(node_handle);

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    // ros::Time odomTimeRec = odomTime;
    // odomTime = ros::Time::now();
    // if (odomTime == odomTimeRec) odomTime += ros::Duration(0.005);
    
    //M? 这里的时间不知道会不会有问题 
    rclcpp::Time odomTimeRec = odomTime;
    // std::cout<<"yes"<<std::endl;
    odomTime = _clock.now();
    
    if (odomTime == odomTimeRec)
    {
        // Add 0.005 seconds to the odom_time
        odomTime += rclcpp::Duration::from_seconds(0.005);
    }
    // std::cout<<"hahaha"<<std::endl;


    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.seconds();
    vehicleXStack[odomSendIDPointer] = vehicleX;
    vehicleYStack[odomSendIDPointer] = vehicleY;
    vehicleZStack[odomSendIDPointer] = vehicleZ;
    vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    vehicleYawStack[odomSendIDPointer] = vehicleYaw;
    terrainRollStack[odomSendIDPointer] = terrainRoll;
    terrainPitchStack[odomSendIDPointer] = terrainPitch;

    // publish 200Hz odometry messages
    tf2::Quaternion quat;
    quat.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat;
    geoQuat.x = quat.x();
    geoQuat.y = quat.y();
    geoQuat.z = quat.z();
    geoQuat.w = quat.w();

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ);
    pubVehicleOdom->publish(odomData);

    // publish 200Hz tf messages
    // odomTrans.stamp_ = odomTime;
    odomTrans.header.stamp = odomTime;
    // odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.transform.rotation.x = geoQuat.x;
    odomTrans.transform.rotation.y = geoQuat.y;
    odomTrans.transform.rotation.z = geoQuat.z;
    odomTrans.transform.rotation.w = geoQuat.w;
    //M? 不知道这里设置平移量对不对
    // odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
    odomTrans.transform.translation.x = vehicleX;
    odomTrans.transform.translation.y = vehicleY;
    odomTrans.transform.translation.z = vehicleZ;

    tfBroadcaster.sendTransform(odomTrans);

    // publish 200Hz Gazebo model state messages (this is for Gazebo simulation)
    // cameraState.pose.orientation = geoQuat;
    // cameraState.pose.position.x = vehicleX;
    // cameraState.pose.position.y = vehicleY;
    // cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    // pubModelState->publish(cameraState);

    // robotState.pose.orientation = geoQuat;
    // robotState.pose.position.x = vehicleX;
    // robotState.pose.position.y = vehicleY;
    // robotState.pose.position.z = vehicleZ;
    // pubModelState->publish(robotState);
    if (client->wait_for_service(std::chrono::microseconds(100))) 
    {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        // 填充请求参数
        request->state.name = "turtlebot3_burger";
        request->state.pose.orientation = geoQuat;
        request->state.pose.position.x = vehicleX;
        request->state.pose.position.y = vehicleY;
        request->state.pose.position.z = vehicleZ-0.75;
        request->state.reference_frame = "world";
        // std::cout<<request->state.pose.position.z<<std::endl;
        // 发送服务请求（异步）
        client->async_send_request(request);

    } 
    else {
        RCLCPP_ERROR(node_handle->get_logger(), "Service not available");
    }


    // geoQuat = tf::createQuaternionMsgFromRollPitchYaw(terrainRoll, terrainPitch, 0);

    quat.setRPY(terrainRoll, terrainPitch, 0);
    // geometry_msgs::msg::Quaternion geoQuat;
    geoQuat.x = quat.x();
    geoQuat.y = quat.y();
    geoQuat.z = quat.z();
    geoQuat.w = quat.w();

    // lidarState.pose.orientation = geoQuat;
    // lidarState.pose.position.x = vehicleX;
    // lidarState.pose.position.y = vehicleY;
    // lidarState.pose.position.z = vehicleZ;
    // pubModelState->publish(lidarState);
    if (client->wait_for_service(std::chrono::microseconds(100))) 
    {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = "VLP16_lidar";
        request->state.pose.orientation = geoQuat;
        request->state.pose.position.x = vehicleX;
        request->state.pose.position.y = vehicleY;
        request->state.pose.position.z = vehicleZ;
        request->state.reference_frame = "world";
        client->async_send_request(request);
        
        // 可以在此添加其他操作，不需要等待响应

    } 
    else {
        RCLCPP_ERROR(node_handle->get_logger(), "Service not available");
    }

    status = rclcpp::ok();;
    rate.sleep();
  }

  return 0;
}
