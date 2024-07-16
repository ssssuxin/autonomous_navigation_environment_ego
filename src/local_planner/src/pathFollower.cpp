#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include <std_msgs/Int8.h>
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
rclcpp::Clock _clock;
nav_msgs::msg::Path path;

void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomIn)
{
  odomTime = odomIn->header.stamp.sec;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.sec;
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.sec;
  }
}

void pathHandler(const nav_msgs::msg::Path::SharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  joyTime =  _clock.now().seconds();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::SharedPtr speed)
{
  double speedTime =  _clock.now().seconds();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::SharedPtr stop)
{
  safetyStop = stop->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("pathFollower");//这个只是说节点的名字叫这个
  node_handle->declare_parameter<double>("sensorOffsetX",0);
  node_handle->declare_parameter<double>("sensorOffsetY",0);
  node_handle->declare_parameter<int>("pubSkipNum",1);
  node_handle->declare_parameter<bool>("twoWayDrive",true);
  node_handle->declare_parameter<double>("lookAheadDis",0.5);
  node_handle->declare_parameter<double>("yawRateGain",7.5);
  node_handle->declare_parameter<double>("stopYawRateGain",7.5);
  node_handle->declare_parameter<double>("maxYawRate",90.0);
  node_handle->declare_parameter<double>("maxSpeed",2.0);
  node_handle->declare_parameter<double>("maxAccel",2.5);
  node_handle->declare_parameter<double>("switchTimeThre",1.0);
  node_handle->declare_parameter<double>("dirDiffThre",0.1);
  node_handle->declare_parameter<double>("stopDisThre",0.2);
  node_handle->declare_parameter<double>("slowDwnDisThre",0.85);
  node_handle->declare_parameter<bool>("useInclRateToSlow",false);
  node_handle->declare_parameter<double>("inclRateThre",120.0);
  node_handle->declare_parameter<double>("slowRate1",0.25);
  node_handle->declare_parameter<double>("slowRate2",0.5);
  node_handle->declare_parameter<double>("slowTime1",2.0);
  node_handle->declare_parameter<double>("slowTime2",2.0);
  node_handle->declare_parameter<bool>("useInclToStop",false);
  node_handle->declare_parameter<double>("inclThre",45.0);
  node_handle->declare_parameter<double>("stopTime",5.0);
  node_handle->declare_parameter<bool>("noRotAtStop",false);
  node_handle->declare_parameter<bool>("noRotAtGoal",true);
  node_handle->declare_parameter<bool>("autonomyMode",true);
  node_handle->declare_parameter<double>("autonomySpeed",2.0);
  node_handle->declare_parameter<double>("joyToSpeedDelay",2.0);
  node_handle->get_parameter("sensorOffsetX",sensorOffsetX);
  node_handle->get_parameter("sensorOffsetY",sensorOffsetY);
  node_handle->get_parameter("pubSkipNum",pubSkipNum);
  node_handle->get_parameter("twoWayDrive",twoWayDrive);
  node_handle->get_parameter("lookAheadDis",lookAheadDis);
  node_handle->get_parameter("yawRateGain",yawRateGain);
  node_handle->get_parameter("stopYawRateGain",stopYawRateGain);
  node_handle->get_parameter("maxYawRate",maxYawRate);
  node_handle->get_parameter("maxSpeed",maxSpeed);
  node_handle->get_parameter("maxAccel",maxAccel);
  node_handle->get_parameter("switchTimeThre",switchTimeThre);
  node_handle->get_parameter("dirDiffThre",dirDiffThre);
  node_handle->get_parameter("stopDisThre",stopDisThre);
  node_handle->get_parameter("slowDwnDisThre",slowDwnDisThre);
  node_handle->get_parameter("useInclRateToSlow",useInclRateToSlow);
  node_handle->get_parameter("inclRateThre",inclRateThre);
  node_handle->get_parameter("slowRate1",slowRate1);
  node_handle->get_parameter("slowRate2",slowRate2);
  node_handle->get_parameter("slowTime1",slowTime1);
  node_handle->get_parameter("slowTime2",slowTime2);
  node_handle->get_parameter("useInclToStop",useInclToStop);
  node_handle->get_parameter("inclThre",inclThre);
  node_handle->get_parameter("stopTime",stopTime);
  node_handle->get_parameter("noRotAtStop",noRotAtStop);
  node_handle->get_parameter("noRotAtGoal",noRotAtGoal);
  node_handle->get_parameter("autonomyMode",autonomyMode);
  node_handle->get_parameter("autonomySpeed",autonomySpeed);
  node_handle->get_parameter("joyToSpeedDelay",joyToSpeedDelay);
  bool test_param=0;
  node_handle->declare_parameter<bool>("test_param",0);
  node_handle->get_parameter("test_param",test_param);
  if(!test_param)
  std::cout<<"pathFollower获取参数失败！ "<<std::endl;
  node_handle->declare_parameter<bool>("show_public_param",false);
  bool show_public_param;
  node_handle->get_parameter("show_public_param",show_public_param);
  if(show_public_param)
  {
    std::cout<<"sensorOffsetX: "<<sensorOffsetX<<std::endl;
    std::cout<<"sensorOffsetY: "<<sensorOffsetY<<std::endl;
    std::cout<<"twoWayDrive: "<<twoWayDrive<<std::endl;
    std::cout<<"maxSpeed: "<<maxSpeed<<std::endl;
    std::cout<<"autonomyMode: "<<autonomyMode<<std::endl;
    std::cout<<"autonomySpeed: "<<autonomySpeed<<std::endl;
    std::cout<<"joyToSpeedDelay: "<<joyToSpeedDelay<<std::endl;
  }


  auto subOdom = node_handle->create_subscription<nav_msgs::msg::Odometry>("/state_estimation",5,odomHandler);
  auto subPath = node_handle->create_subscription<nav_msgs::msg::Path>("/path",5,pathHandler);
  auto subJoystick = node_handle->create_subscription<sensor_msgs::msg::Joy>("/joy",5,joystickHandler);
  auto subSpeed = node_handle->create_subscription<std_msgs::msg::Float32>("/joy",5,speedHandler);
  auto subStop = node_handle->create_subscription<std_msgs::msg::Int8>("/stop",5,stopHandler);
  
  // ros::Subscriber subOdom = nh.subscribe<nav_msgs::msg::Odometry> ("/state_estimation", 5, odomHandler);

  // ros::Subscriber subPath = nh.subscribe<nav_msgs::msg::Path> ("/path", 5, pathHandler);

  // ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::msg::Joy> ("/joy", 5, joystickHandler);

  // ros::Subscriber subSpeed = nh.subscribe<std_msgs::msg::Float32> ("/speed", 5, speedHandler);

  // ros::Subscriber subStop = nh.subscribe<std_msgs::msg::Int8> ("/stop", 5, stopHandler);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubSpeed=node_handle->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

  // ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(node_handle);

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        double time = _clock.now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        rclcpp::Time timestamp_(static_cast<int64_t>(odomTime * 1000000000));
        cmd_vel.header.stamp = timestamp_;
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed->publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
