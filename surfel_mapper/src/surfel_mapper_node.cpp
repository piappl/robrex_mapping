#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <Eigen/Geometry>
#include <limits.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>  
#include "surfel_mapper.hpp"

struct SensorPose {
	public:
		Eigen::Quaternionf orientation ;
		Eigen::Vector4f origin ;
} ;

//typedef std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudMsgListT ;
typedef std::list<sensor_msgs::PointCloud2::ConstPtr> PointCloudMsgListT ;

nav_msgs::Path::ConstPtr current_path ;
PointCloudMsgListT cloudMsgQueue ;

Eigen::Matrix4d cameraRgbToCameraLinkTrans ;

SurfelMapper surfel_mapper ;

bool getSensorPosition(const ros::Time &time_stamp, SensorPose &sensor_pose)
{
	if (!current_path) {
		ROS_WARN("No odometry path message available!") ;
		return false ;
	} else if (current_path->poses.empty()) {
		ROS_WARN("Empty list of poses in odometry path message") ;
		return false ;
	} else if (current_path->poses.front().header.stamp > time_stamp || current_path->poses.back().header.stamp < time_stamp) {
		ROS_WARN("Odometry path message does not contain pose corresponding with the keyframe. Keyframe timestamp [%d.%d]. Odometry timestamps [%d.%d]-[%d.%d]", 
				time_stamp.sec, time_stamp.nsec, current_path->poses.front().header.stamp.sec, current_path->poses.front().header.stamp.nsec, 
				current_path->poses.back().header.stamp.sec, current_path->poses.back().header.stamp.nsec) ;
		return false ;
	} else {
		//Search by bi-section
		size_t i, j, k ;
		i = 0 ; j = current_path->poses.size() - 1 ;
		while (i + 1 < j) {
			k = (i + j) / 2 ;
			if (current_path->poses[k].header.stamp <= time_stamp)
				i = k ;
			else
				j = k ;
		}

		//Find closest match (nearest neighbor)	
		ros::Duration duri = time_stamp - current_path->poses[i].header.stamp ;
		ros::Duration durj = current_path->poses[j].header.stamp - time_stamp ;
		if (duri < durj)
			k = i ;
		else
			k = j ;

		geometry_msgs::PoseStamped pose_stamped = current_path->poses[k] ;
		//ROS_INFO("Stamp found for k = %ld (out of %ld), number of steps [%ld]", k, current_path->poses.size(), steps) ;
		//ROS_INFO("search time stamp [%d,%d], found time stamp [%d,%d]", time_stamp.sec, time_stamp.nsec, current_path->poses[i].header.stamp.sec, current_path->poses[i].header.stamp.nsec) ;
		//ROS_INFO("search time stamp [%d,%d], found time stamp [%d,%d]", time_stamp.sec, time_stamp.nsec, current_path->poses[j].header.stamp.sec, current_path->poses[j].header.stamp.nsec) ;
		ROS_INFO("search time stamp [%d,%d], found time stamp [%d,%d]", time_stamp.sec, time_stamp.nsec, pose_stamped.header.stamp.sec, pose_stamped.header.stamp.nsec) ;

		sensor_pose.origin = Eigen::Vector4f((float) pose_stamped.pose.position.x, (float) pose_stamped.pose.position.y, (float) pose_stamped.pose.position.z, 1.0f) ;
		sensor_pose.orientation = Eigen::Quaternionf((float) pose_stamped.pose.orientation.w, (float) pose_stamped.pose.orientation.x, 
							     (float) pose_stamped.pose.orientation.y, (float) pose_stamped.pose.orientation.z) ;
		std::cout << "Orientation: " << pose_stamped.pose.orientation.w << " " << pose_stamped.pose.orientation.x << " " << pose_stamped.pose.orientation.y << " " << pose_stamped.pose.orientation.z << std::endl ;
		std::cout << "Pose: " << pose_stamped.pose.position.x << " " << pose_stamped.pose.position.y << " " << pose_stamped.pose.position.z << " " << std::endl ;
		return true ;
	}
}


void processCloudMsgQueue()
{
	//Try to associate clouds from the queue with appropriate transforms and process them
	while(!cloudMsgQueue.empty()) {
		SensorPose sensor_pose ;
		const sensor_msgs::PointCloud2::ConstPtr& msg = cloudMsgQueue.front() ;
		bool res = getSensorPosition(msg->header.stamp, sensor_pose) ;
		if (res) {
			//Convert message to PointCloud
			pcl::PCLPointCloud2 pcl_pc2;
			pcl_conversions::toPCL(*msg, pcl_pc2);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
			pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

			//Fix sensor pose		
			cloud->sensor_origin_ = sensor_pose.origin ;
			cloud->sensor_orientation_ = sensor_pose.orientation ;

			//Add cloud to the map
			ROS_INFO("-------------->Adding point cloud [%d, %d]", msg->header.stamp.sec, msg->header.stamp.nsec) ;
			ROS_INFO("Sensor position data: [%f, %f, %f, %f] ", cloud->sensor_origin_.x(), cloud->sensor_origin_.y(), cloud->sensor_origin_.z(), cloud->sensor_origin_.w()) ;
			ROS_INFO("Sensor orientation data: [%f, %f, %f, %f] ", cloud->sensor_orientation_.x(), cloud->sensor_orientation_.y(), cloud->sensor_orientation_.z(), cloud->sensor_orientation_.w()) ;

			surfel_mapper.addPointCloudToScene(cloud) ;
			//addPointCloudToScene1(cloud) ;

			//Remove message from queue
			cloudMsgQueue.pop_front() ;	
		} else break ;
	}
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
	ROS_DEBUG("pathCallback: [%s]", msg->header.frame_id.c_str());
	current_path = msg ;
}

void keyframeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("keyframeCallback: [%s]", msg->header.frame_id.c_str());
	//Add point cloud to our local queue (the queue is needed since we must sometimes wait for a transform from a path)
	cloudMsgQueue.push_back(msg) ;	
	processCloudMsgQueue() ;
}

void sendMapMessage(ros::Publisher &map_pub) 
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneDownsampled = surfel_mapper.getCloudSceneDownsampled() ;

	pcl::PCLPointCloud2 pcl_pc2;
	pcl::toPCLPointCloud2(*cloudSceneDownsampled, pcl_pc2) ;
	sensor_msgs::PointCloud2 cloud_msg ;
	pcl_conversions::fromPCL(pcl_pc2, cloud_msg) ;
	cloud_msg.header.frame_id = "/odom" ;
	map_pub.publish(cloud_msg) ;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "surfel_mapper");
	ros::NodeHandle n;

	ros::Subscriber sub_path = n.subscribe("mapper_path", 3, pathCallback);
	ros::Subscriber sub_keyframe = n.subscribe("keyframes", 5, keyframeCallback);

	ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("surfelmap_preview", 5);

	ros::Rate r(2) ;


	//testOctreeIterator() ;


	tf::TransformListener listener ;

	while(ros::ok()) {
		ROS_INFO("Waiting for camera_link->camera_rgb_optical_frame transform") ;
		tf::StampedTransform transform;
		try {
			listener.waitForTransform("camera_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(0.5) );
			listener.lookupTransform("camera_link", "camera_rgb_optical_frame", ros::Time(0), transform);
			ROS_INFO("Transform found!") ;
			Eigen::Affine3d affTrans ;
			tf::transformTFToEigen(transform, affTrans) ;
			//cameraRgbToCameraLinkTrans = affTrans.matrix().inverse() ;
			cameraRgbToCameraLinkTrans = affTrans.matrix().inverse() ;
			std::cout << cameraRgbToCameraLinkTrans << std::endl ;
			break ;
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			r.sleep() ;
		}
	}

	while(ros::ok()) {
		ros::spinOnce();
		processCloudMsgQueue() ;
		ros::Time start = ros::Time::now() ;
		sendMapMessage(map_pub) ;
		ros::Time stop = ros::Time::now() ;
		ROS_DEBUG("Sending Map Message time (s): [%.6lf]", (stop - start).toSec()) ;
		r.sleep() ;

		//ros::Time time_stamp ;
		//time_stamp.sec = 1408952265 ;
		//time_stamp.nsec = 954192400 ;
	
		//SensorPose sensor_pose ;

		//ros::Time begin = ros::Time::now();
		//bool ret = getSensorPosition(time_stamp, sensor_pose) ;
		//ros::Time end = ros::Time::now();
		//ros::Duration duration = end - begin ;
		//ROS_INFO("getSensorPosition: %.6lf (s) ", duration.toSec()) ;
		
		//ROS_INFO("Sensor position correctly acquired: [%d] ", ret) ;
		//ROS_INFO("Sensor position data: [%f, %f, %f, %f] ", sensor_pose.origin.x(), sensor_pose.origin.y(), sensor_pose.origin.z(), sensor_pose.origin.w()) ;
		//ROS_INFO("Sensor orientation data: [%f, %f, %f, %f] ", sensor_pose.orientation.x(), sensor_pose.orientation.y(), sensor_pose.orientation.z(), sensor_pose.orientation.w()) ;
	}

	return 0;
}
