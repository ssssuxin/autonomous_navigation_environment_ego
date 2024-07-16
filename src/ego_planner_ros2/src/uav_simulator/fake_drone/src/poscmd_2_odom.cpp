#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "quadrotor_msgs/msg/position_command.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr _cmd_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_raw_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_Draw_sub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScan;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDpcl;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  _odom_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  _odom_pub_test;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  _odom_pub_test2;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
rclcpp::Clock _clock;
quadrotor_msgs::msg::PositionCommand::SharedPtr _cmd;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud1(new pcl::PointCloud<pcl::PointXYZI>());
std::vector<int> scanInd;
double _init_x, _init_y, _init_z;

bool rcv_cmd = false;
nav_msgs::msg::Odometry odom_copy;
nav_msgs::msg::Odometry odom_for_camera;
nav_msgs::msg::Odometry odom_for_camera_service;
void rcvPosCmdCallBack(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
{	
	rcv_cmd = true;
	_cmd    = cmd;
}
void raw_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_pcl)
{
	cloud->clear();
	transformed_cloud->clear();
	Eigen::Quaterniond radar_orientation(odom_copy.pose.pose.orientation.w,
                                         odom_copy.pose.pose.orientation.x,
                                         odom_copy.pose.pose.orientation.y,
                                         odom_copy.pose.pose.orientation.z);
    Eigen::Vector3d radar_position(odom_copy.pose.pose.position.x,
                                   odom_copy.pose.pose.position.y,
                                   odom_copy.pose.pose.position.z);

	
	pcl::fromROSMsg(*raw_pcl,*cloud);
	pcl::removeNaNFromPointCloud(*cloud, *cloud, scanInd);

	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(radar_position);
    transform.rotate(radar_orientation);

    // Transform the point cloud
    
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
	sensor_msgs::msg::PointCloud2::SharedPtr msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
	pcl::toROSMsg(*transformed_cloud, *msg);
	
	msg->header.stamp = raw_pcl->header.stamp;
	msg->header.frame_id="map";
	pubScan->publish(*msg);
}
void raw_Dcamera_callback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_pcl)
{
	cloud1->clear();
	transformed_cloud1->clear();
	

	// Eigen::Quaterniond radar_orientation(odom_for_camera.pose.pose.orientation.w,
    //                                      odom_for_camera.pose.pose.orientation.x,
    //                                      odom_for_camera.pose.pose.orientation.y,
    //                                      odom_for_camera.pose.pose.orientation.z);
	Eigen::Quaterniond radar_orientation(odom_for_camera.pose.pose.orientation.w,
                                         odom_for_camera.pose.pose.orientation.x,
                                         odom_for_camera.pose.pose.orientation.y,
                                         odom_for_camera.pose.pose.orientation.z);
    Eigen::Vector3d radar_position(odom_for_camera.pose.pose.position.x,
                                   odom_for_camera.pose.pose.position.y,
                                   odom_for_camera.pose.pose.position.z-0.1);

	// std::cout<<" sssss  "<<std::endl;
	pcl::fromROSMsg(*raw_pcl,*cloud1);
	pcl::removeNaNFromPointCloud(*cloud1, *cloud1, scanInd);

	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(radar_position);
    transform.rotate(radar_orientation);

    // Transform the point cloud
    
    pcl::transformPointCloud(*cloud1, *transformed_cloud1, transform);
	sensor_msgs::msg::PointCloud2::SharedPtr msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
	pcl::toROSMsg(*transformed_cloud1, *msg);
	// std::cout<<"-----------:  "<<transformed_cloud1->size()<<std::endl;
	msg->header.stamp = raw_pcl->header.stamp;
	msg->header.frame_id="map";
	pubDpcl->publish(*msg);
}

void pubOdom()
{	
	nav_msgs::msg::Odometry odom;
	geometry_msgs::msg::TransformStamped t;
	odom.header.stamp    = _clock.now();
	odom.header.frame_id = "world";
	Eigen::Quaterniond q_copy;
	bool falg=false;
	if(rcv_cmd)
	{
	    odom.pose.pose.position.x = _cmd->position.x;
	    odom.pose.pose.position.y = _cmd->position.y;
	    odom.pose.pose.position.z = _cmd->position.z;

		Eigen::Vector3d alpha = Eigen::Vector3d(_cmd->acceleration.x, _cmd->acceleration.y, _cmd->acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
		Eigen::Vector3d xC(cos(_cmd->yaw), sin(_cmd->yaw), 0);
		Eigen::Vector3d yC(-sin(_cmd->yaw), cos(_cmd->yaw), 0);
		Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
		Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
		Eigen::Vector3d zB = xB.cross(yB);
		Eigen::Matrix3d R;
		R.col(0) = xB;
		R.col(1) = yB;
		R.col(2) = zB;
		Eigen::Quaterniond q(R);
		q_copy=q;
		falg=true;
	    // odom.pose.pose.orientation.w = q.w();
	    // odom.pose.pose.orientation.x = q.x();
		///
	    // odom.pose.pose.orientation.y = q.y();
	    // odom.pose.pose.orientation.z = q.z();

	    odom.twist.twist.linear.x = _cmd->velocity.x;
	    odom.twist.twist.linear.y = _cmd->velocity.y;
	    odom.twist.twist.linear.z = _cmd->velocity.z;

	    odom.twist.twist.angular.x = _cmd->acceleration.x;
	    odom.twist.twist.angular.y = _cmd->acceleration.y;
	    odom.twist.twist.angular.z = _cmd->acceleration.z;
	}
	else
	{
		odom.pose.pose.position.x = _init_x;
	    odom.pose.pose.position.y = _init_y;
	    odom.pose.pose.position.z = _init_z;

	    odom.pose.pose.orientation.w = 1;
	    odom.pose.pose.orientation.x = 0;
	    odom.pose.pose.orientation.y = 0;
	    odom.pose.pose.orientation.z = 0;

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;
	}
	t.header.stamp = _clock.now();
    t.header.frame_id = "map";
    t.child_frame_id = "UAV";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = odom.pose.pose.position.z;
	// std::cout<<"("<<odom.pose.pose.position.x<<","<<odom.pose.pose.position.y<<","<<odom.pose.pose.position.z<<")"<<std::endl;
    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    t.transform.rotation.x = q_copy.x();
    t.transform.rotation.y = q_copy.y();
    t.transform.rotation.z = q_copy.z();
    t.transform.rotation.w = q_copy.w();
	if(falg)
	tf_broadcaster_->sendTransform(t);



    _odom_pub->publish(odom);
	odom_copy=odom;
	odom_for_camera=odom_copy;
	odom_for_camera.pose.pose.position.z = odom_for_camera.pose.pose.position.z-0.1;
	odom_for_camera.pose.pose.orientation = t.transform.rotation;
	_odom_pub_test->publish(odom_for_camera);

////////

	tf2::Quaternion myQuaternion(odom_for_camera.pose.pose.orientation.x, odom_for_camera.pose.pose.orientation.y, odom_for_camera.pose.pose.orientation.z, odom_for_camera.pose.pose.orientation.w);
//使用 Matrix3x3 工具，其构造函数可以直接输入四元数
	tf2::Matrix3x3 myMat(myQuaternion);
	double roll, pitch, yaw;
	myMat.getRPY(roll, pitch, yaw);
	myQuaternion.setRPY(0, 0,  yaw);
	odom_for_camera.pose.pose.orientation.w = myQuaternion.w();
    odom_for_camera.pose.pose.orientation.x = myQuaternion.x();
    odom_for_camera.pose.pose.orientation.y = myQuaternion.y();
    odom_for_camera.pose.pose.orientation.z = myQuaternion.z();
//////////

	odom_for_camera_service = odom_for_camera;
	_odom_pub_test2->publish(odom_for_camera_service);



	Eigen::Quaterniond quat(odom_for_camera.pose.pose.orientation.w, odom_for_camera.pose.pose.orientation.x, odom_for_camera.pose.pose.orientation.y, odom_for_camera.pose.pose.orientation.z);

    // 将四元数转换为欧拉角（RPY）
    Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2); // Roll-Pitch-Yaw (RPY)
	rpy[0] =  rpy[0] - M_PI/2;
	// rpy[1] = 0;
	rpy[2] = rpy[2] -M_PI/2;
	quat = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());

    // 将 Eigen::Quaterniond 转换回 geometry_msgs::msg::Quaternion
    // geometry_msgs::msg::Quaternion rotated_quat;
    odom_for_camera.pose.pose.orientation.w = quat.w();
    odom_for_camera.pose.pose.orientation.x = quat.x();
    odom_for_camera.pose.pose.orientation.y = quat.y();
    odom_for_camera.pose.pose.orientation.z = quat.z();
}

int main (int argc, char** argv) 
{        
    // ros::init (argc, argv, "odom_generator");
    // ros::NodeHandle nh( "~" );
	rclcpp::init(argc, argv);
  	auto nh = std::make_shared<rclcpp::Node>("odom_generator");//这个只是说节点的名字叫这个

	nh->declare_parameter<double>("init_x",0.0);
	nh->declare_parameter<double>("init_y",0.0);
	nh->declare_parameter<double>("init_z",0.0);
	nh->get_parameter("init_x",_init_x);
	nh->get_parameter("init_y",_init_y);
	nh->get_parameter("init_z",_init_z);
	auto client = nh->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
    // nh.param("init_x", _init_x,  0.0);
    // nh.param("init_y", _init_y,  0.0);
    // nh.param("init_z", _init_z,  0.0);

    // _cmd_sub = nh.subscribe( "command", 1, rcvPosCmdCallBack );
	_cmd_sub = nh->create_subscription<quadrotor_msgs::msg::PositionCommand>("command",1,rcvPosCmdCallBack);
	pcl_raw_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points/out",2,raw_pcl_callback);
	pcl_Draw_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/depth_camera/points",2,raw_Dcamera_callback);
    // _odom_pub = nh.advertise<nav_msgs::msg::Odometry>("odometry", 1);                      
	_odom_pub = nh->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
	_odom_pub_test = nh->create_publisher<nav_msgs::msg::Odometry>("odometrytest", 1);
	_odom_pub_test2 = nh->create_publisher<nav_msgs::msg::Odometry>("odometrytest1", 1);
	pubScan = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_scan_at_map", 2);
	pubDpcl = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/dcamera_at_map", 2);
	tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(nh);
    rclcpp::Rate rate(400);
    bool status = rclcpp::ok();
	geometry_msgs::msg::Quaternion geoQuat;
    while(status) 
    {
		pubOdom();                   
        // ros::spinOnce();
		rclcpp::spin_some(nh);
        status = rclcpp::ok();
        rate.sleep();
//////////////////////////
		geoQuat.x = odom_copy.pose.pose.orientation.x;
		geoQuat.y = odom_copy.pose.pose.orientation.y;
		geoQuat.z = odom_copy.pose.pose.orientation.z;
		geoQuat.w = odom_copy.pose.pose.orientation.w;
		if (client->wait_for_service(std::chrono::microseconds(100))) 
		{
			auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
			// 填充请求参数
			request->state.name = "lidar_uav_0";
			request->state.pose.orientation = geoQuat;
			request->state.pose.position.x = odom_copy.pose.pose.position.x;
			request->state.pose.position.y = odom_copy.pose.pose.position.y;
			request->state.pose.position.z = odom_copy.pose.pose.position.z;
			request->state.reference_frame = "world";
			// std::cout<<request->state.pose.position.z<<std::endl;
			// 发送服务请求（异步）
			client->async_send_request(request);

		} 
		else {
			RCLCPP_ERROR(nh->get_logger(), "Service not available");
		}

		if (client->wait_for_service(std::chrono::microseconds(100))) 
		{
			auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
			// 填充请求参数
			request->state.name = "uav_0";
			request->state.pose.orientation = geoQuat;
			request->state.pose.position.x = odom_copy.pose.pose.position.x;
			request->state.pose.position.y = odom_copy.pose.pose.position.y;
			request->state.pose.position.z = odom_copy.pose.pose.position.z-0.2;
			request->state.reference_frame = "world";
			// std::cout<<request->state.pose.position.z<<std::endl;
			// 发送服务请求（异步）
			client->async_send_request(request);

		} 
		else {
			RCLCPP_ERROR(nh->get_logger(), "Service not available");
		}
		// if (client->wait_for_service(std::chrono::microseconds(100))) 
		// {
		// 	auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
		// 	// 填充请求参数
		// 	request->state.name = "Dcamera_uav_0";
		// 	request->state.pose = odom_for_camera_service.pose.pose;
		// 	// request->state.pose.orientation = odom_for_camera_service.pose.pose.orientation;
		// 	// request->state.pose.position.x = odom_for_camera_service.pose.pose.position.x;
		// 	// request->state.pose.position.y = odom_for_camera_service.pose.pose.position.y;
		// 	// request->state.pose.position.z = odom_for_camera_service.pose.pose.position.z;
		// 	request->state.reference_frame = "world";
		// 	// std::cout<<request->state.pose.position.z<<std::endl;
		// 	// 发送服务请求（异步）
		// 	client->async_send_request(request);

		// } 
		// else {
		// 	RCLCPP_ERROR(nh->get_logger(), "Service not available");
		// }
		////////////////////
    }

    return 0;
}