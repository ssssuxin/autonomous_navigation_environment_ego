#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// #include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

nav_msgs::msg::Odometry odometryIn;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometryPointer = NULL;
// tf::StampedTransform transformToMap;
geometry_msgs::msg::TransformStamped transformToMap;
// tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
tf2_ros::TransformBroadcaster *tfBroadcasterPointer = NULL;
// ros::Publisher pubLaserCloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
void laserCloudAndOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& laserCloud2)
{
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);
  // std::cout<<"registered_scan ："<<laserCloudIn->width<<std::endl;
  odometryIn = *odometry;

  // transformToMap.setOrigin(
  //     tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.transform.translation.x = odometryIn.pose.pose.position.x;
  transformToMap.transform.translation.y = odometryIn.pose.pose.position.y;
  transformToMap.transform.translation.z = odometryIn.pose.pose.position.z;


  // transformToMap.setRotation(tf2::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
  //                                           odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));
  transformToMap.transform.rotation.x = odometryIn.pose.pose.orientation.x;
  transformToMap.transform.rotation.y = odometryIn.pose.pose.orientation.y;
  transformToMap.transform.rotation.z = odometryIn.pose.pose.orientation.z;
  transformToMap.transform.rotation.w = odometryIn.pose.pose.orientation.w;

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  // tf::Vector3 vec;
  Eigen::Affine3d transform;
  Eigen::Vector3d vec;
  //M? 这个地方的矩阵不知道是不是这样赋值的，出问题再说
  transform.matrix() << transformToMap.transform.rotation.w, transformToMap.transform.rotation.x,
                        transformToMap.transform.rotation.y, transformToMap.transform.rotation.z,
                        transformToMap.transform.translation.x, transformToMap.transform.translation.y,
                        transformToMap.transform.translation.z, 0, 0, 0, 0, 1;
  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec << p1.x, p1.y, p1.z;

    vec = transform.inverse() * vec;


    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    laserCLoudInSensorFrame->points.push_back(p1);
  }
  // std::cout<<"laserCloudInNum ："<<laserCloudInNum<<std::endl;
  
  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  // transformToMap.stamp_ = laserCloud2->header.stamp;
  // transformToMap.frame_id_ = "map";
  // transformToMap.child_frame_id_ = "sensor_at_scan";

  transformToMap.header.stamp = laserCloud2->header.stamp;
  transformToMap.header.frame_id = "map";
  transformToMap.child_frame_id = "sensor_at_scan";

  tfBroadcasterPointer->sendTransform(transformToMap);

  sensor_msgs::msg::PointCloud2 scan_data;
  // std::cout<<"sensor_scan ："<<laserCLoudInSensorFrame->points.size()<<std::endl;
  // std::cout<<"-----------------"<<std::endl;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";
  pubLaserCloud->publish(scan_data);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("sensor_scan");//这个只是说节点的名字叫这个


  // ROS message filters
  message_filters::Subscriber<nav_msgs::msg::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(node_handle, "/state_estimation");
  subLaserCloud.subscribe(node_handle, "/registered_scan");
  //EXPL 这里改成ros2的关键是把订阅的队列长度去掉，仅此而已（也就是话题字符后面的逗号和数字去掉）
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));
  //TAG 这里改ros2的关键 1.订阅器用subscribe操作  2.同步回调函数的形参类型格式  3.如果是用类内的函数 &类名::函数名,this,占位符
  // auto subOdometry = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(node_handle, "/state_estimation");
  // auto subLaserCloud = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node_handle, "/registered_scan");
  // using syncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  // auto sync = std::make_shared<message_filters::Synchronizer<syncPolicy>>(syncPolicy(100), subOdometry, subLaserCloud);
  // sync->registerCallback(&laserCloudAndOdometryHandler);
  // message_filters::Subscriber<nav_msgs::msg::Odometry> subOdometry;
  // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subLaserCloud;
  // subOdometry.subscribe(node_handle,"/state_estimation");
  // subLaserCloud.subscribe(node_handle, "/registered_scan");


  


  // using OdometrySubscriber = message_filters::Subscriber<nav_msgs::msg::Odometry>;
  // using LaserCloudSubscriber = message_filters::Subscriber<sensor_msgs::msg::PointCloud2>;

  // auto subOdometry = std::make_shared<OdometrySubscriber>(node_handle, "/state_estimation");
  // auto subLaserCloud = std::make_shared<LaserCloudSubscriber>(node_handle, "/registered_scan");

  // using syncPolicy = message_filters::sync_policies::ApproximateTime<
  //   nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  // auto sync = std::make_shared<message_filters::Synchronizer<syncPolicy>>(
  //   syncPolicy(100), subOdometry, subLaserCloud);

  // sync->registerCallback(&laserCloudAndOdometryHandler);




  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry=node_handle->create_publisher<nav_msgs::msg::Odometry>("/state_estimation_at_scan", 5);
  // ros::Publisher pubOdometry = nh.advertise<nav_msgs::msg::Odometry> ("/state_estimation_at_scan", 5);
  pubOdometryPointer = pubOdometry;

  // tf::TransformBroadcaster tfBroadcaster;
  // tfBroadcasterPointer = &tfBroadcaster;
  tf2_ros::TransformBroadcaster tfBroadcaster(node_handle);
  tfBroadcasterPointer = &tfBroadcaster;
  pubLaserCloud = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor_scan", 2);
  // nh.advertise<sensor_msgs::msg::PointCloud2>("/sensor_scan", 2);

  // ros::spin();
  rclcpp::spin(node_handle);

  return 0;
}
