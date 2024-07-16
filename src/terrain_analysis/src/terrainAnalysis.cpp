#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"

#include <tf2_ros/transform_broadcaster.h>#include <tf2/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
using namespace std;

const double PI = 3.1415926;

double scanVoxelSize = 0.05;
double decayTime = 2.0;
double noDecayDis = 4.0;
double clearingDis = 8.0;
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25;
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// terrain voxel parameters
float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.2;
const int planarVoxelWidth = 51;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelEdge[planarVoxelNum] = {0};
int planarVoxelDyObs[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum];

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

// state estimation callback function
void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0) {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1) {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis)
      noDataInited = 2;
  }
}

// registered laser scan callback function
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloud2) {
  laserCloudTime = laserCloud2->header.stamp.sec;

  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::msg::Joy::SharedPtr joy) {
  if (joy->buttons[5] > 0.5) {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::msg::Float32::SharedPtr dis) {
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv) {


  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("terrainAnalysis");//这个只是说节点的名字叫这个

  node_handle->declare_parameter<double>("scanVoxelSize",0.05);
  node_handle->declare_parameter<double>("decayTime",2.0);
  node_handle->declare_parameter<double>("noDecayDis",4.0);
  node_handle->declare_parameter<double>("clearingDis",8.0);
  node_handle->declare_parameter<bool>("useSorting",false);
  node_handle->declare_parameter<double>("quantileZ",0.25);
  node_handle->declare_parameter<bool>("considerDrop",true);
  node_handle->declare_parameter<bool>("limitGroundLift",false);
  node_handle->declare_parameter<double>("maxGroundLift",0.15);
  node_handle->declare_parameter<bool>("clearDyObs",true);
  node_handle->declare_parameter<double>("minDyObsDis",0.3);
  node_handle->declare_parameter<double>("minDyObsAngle",0.0);
  node_handle->declare_parameter<double>("minDyObsRelZ",-0.5);
  node_handle->declare_parameter<double>("minDyObsVFOV",-16.0);
  node_handle->declare_parameter<double>("maxDyObsVFOV",16.0);
  node_handle->declare_parameter<int>("minDyObsPointNum",1);
  node_handle->declare_parameter<bool>("noDataObstacle",false);
  node_handle->declare_parameter<int>("noDataBlockSkipNum",0);
  node_handle->declare_parameter<int>("minBlockPointNum",10);
  node_handle->declare_parameter<double>("vehicleHeight",1.5);
  node_handle->declare_parameter<int>("voxelPointUpdateThre",100);
  node_handle->declare_parameter<double>("voxelTimeUpdateThre",2.0);
  node_handle->declare_parameter<double>("minRelZ",-2.5);
  node_handle->declare_parameter<double>("maxRelZ",1.0);
  node_handle->declare_parameter<double>("disRatioZ",0.2);

  node_handle->get_parameter("scanVoxelSize",scanVoxelSize);
  node_handle->get_parameter("decayTime",decayTime);
  node_handle->get_parameter("noDecayDis",noDecayDis);
  node_handle->get_parameter("clearingDis",clearingDis);
  node_handle->get_parameter("useSorting",useSorting);
  node_handle->get_parameter("quantileZ",quantileZ);
  node_handle->get_parameter("considerDrop",considerDrop);
  node_handle->get_parameter("limitGroundLift",limitGroundLift);
  node_handle->get_parameter("maxGroundLift",maxGroundLift);
  node_handle->get_parameter("clearDyObs",clearDyObs);
  node_handle->get_parameter("minDyObsDis",minDyObsDis);
  node_handle->get_parameter("minDyObsAngle",minDyObsAngle);
  node_handle->get_parameter("minDyObsRelZ",minDyObsRelZ);
  node_handle->get_parameter("minDyObsVFOV",minDyObsVFOV);
  node_handle->get_parameter("maxDyObsVFOV",maxDyObsVFOV);
  node_handle->get_parameter("minDyObsPointNum",minDyObsPointNum);
  node_handle->get_parameter("noDataObstacle",noDataObstacle);
  node_handle->get_parameter("noDataBlockSkipNum",noDataBlockSkipNum);
  node_handle->get_parameter("minBlockPointNum",minBlockPointNum);
  node_handle->get_parameter("vehicleHeight",vehicleHeight);
  node_handle->get_parameter("voxelPointUpdateThre",voxelPointUpdateThre);
  node_handle->get_parameter("voxelTimeUpdateThre",voxelTimeUpdateThre);
  node_handle->get_parameter("minRelZ",minRelZ);
  node_handle->get_parameter("maxRelZ",maxRelZ);
  node_handle->get_parameter("disRatioZ",disRatioZ);
  bool test_param=0;
  node_handle->declare_parameter<bool>("test_param",0);
  node_handle->get_parameter("test_param",test_param);
  if(!test_param)
  std::cout<<"terrainAnalysis获取参数失败！ "<<std::endl;
  node_handle->declare_parameter<bool>("show_public_param",false);
  bool show_public_param;
  node_handle->get_parameter("show_public_param",show_public_param);
  if(show_public_param)
  {
    std::cout<<"minDyObsRelZ: "<<minDyObsRelZ<<std::endl;
    std::cout<<"vehicleHeight: "<<vehicleHeight<<std::endl;

  }

  auto subOdometry = node_handle->create_subscription<nav_msgs::msg::Odometry>("/state_estimation",5,odometryHandler);
  auto subLaserCloud = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan",5,laserCloudHandler);
  auto subJoystick = node_handle->create_subscription<sensor_msgs::msg::Joy>("/joy",5,joystickHandler);
  auto subClearing = node_handle->create_subscription<std_msgs::msg::Float32>("/map_clearing",5,clearingHandler);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud =
      node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_map", 2);


  for (int i = 0; i < terrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();;
  while (status) {
    rclcpp::spin_some(node_handle);

    if (newlaserCloud) {
      newlaserCloud = false;

      // terrain voxel roll over
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop->points[i];

        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }

      for (int ind = 0; ind < terrainVoxelNum; ind++) {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[ind];

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++) {
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity <
                     decayTime ||
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud)) {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 5;
           indX <= terrainVoxelHalfWidth + 5; indX++) {
        for (int indY = terrainVoxelHalfWidth - 5;
             indY <= terrainVoxelHalfWidth + 5; indY++) {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // estimate ground and compute elevation for each point
      for (int i = 0; i < planarVoxelNum; i++) {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];

        int indX =
            int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
        int indY =
            int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;

        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                    .push_back(point.z);
              }
            }
          }
        }

        if (clearDyObs) {
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis) {
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
              if (angle1 > minDyObsAngle) {
                float pointX2 =
                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 =
                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                float pointX3 =
                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 =
                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                float pointX4 = pointX3;
                float pointY4 =
                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 =
                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) {
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                }
              }
            } else {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                  minDyObsPointNum;
            }
          }
        }
      }

      if (clearDyObs) {
        for (int i = 0; i < laserCloudCropSize; i++) {
          point = laserCloudCrop->points[i];

          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
            if (angle1 > minDyObsAngle) {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }

      if (useSorting) {
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = int(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            if (planarPointElev[i][quantileID] >
                    planarPointElev[i][0] + maxGroundLift &&
                limitGroundLift) {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            } else {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      } else {
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                    minDyObsPointNum ||
                !clearDyObs) {
              float disZ =
                  point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop)
                disZ = fabs(disZ);
              int planarPointElevSize =
                  planarPointElev[planarVoxelWidth * indX + indY].size();
              if (disZ >= 0 && disZ < vehicleHeight &&
                  planarPointElevSize >= minBlockPointNum) {
                terrainCloudElev->push_back(point);
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                terrainCloudElevSize++;
              }
            }
          }
        }
      }

      if (noDataObstacle && noDataInited == 2) {
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum) {
            planarVoxelEdge[i] = 1;
          }
        }

        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum;
             noDataBlockSkipCount++) {
          for (int i = 0; i < planarVoxelNum; i++) {
            if (planarVoxelEdge[i] >= 1) {
              int indX = int(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++) {
                for (int dY = -1; dY <= 1; dY++) {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                      indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                        dY] < planarVoxelEdge[i]) {
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel)
                planarVoxelEdge[i]++;
            }
          }
        }

        for (int i = 0; i < planarVoxelNum; i++) {
          if (planarVoxelEdge[i] > noDataBlockSkipNum) {
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x =
                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y =
                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.intensity = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      clearingCloud = false;

      // publish points with elevation
      sensor_msgs::msg::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      rclcpp::Time timestamp_1(static_cast<int64_t>(laserCloudTime * 1000000000));
      terrainCloud2.header.stamp = timestamp_1;
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud->publish(terrainCloud2);
    }

    status = rclcpp::ok();;
    rate.sleep();
  }

  return 0;
}
