#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <Eigen/Geometry>

#include <pcl/common/common_headers.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/PCLPointCloud2.h>


struct SensorPose {
	public:
		Eigen::Quaternionf orientation ;
		Eigen::Vector4f origin ;
} ;

//typedef std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudMsgListT ;
typedef std::list<sensor_msgs::PointCloud2::ConstPtr> PointCloudMsgListT ;

nav_msgs::Path::ConstPtr current_path ;
PointCloudMsgListT cloudMsgQueue ;

bool getSensorPosition(const ros::Time &time_stamp, SensorPose &sensor_pose)
{
	if (!current_path) 
		return false ;
	else if (current_path->poses.empty()) 
		return false ;
	else if (current_path->poses.front().header.stamp > time_stamp || current_path->poses.back().header.stamp < time_stamp) 
		return false ;
	else {
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
		sensor_pose.origin = Eigen::Vector4f((float) pose_stamped.pose.position.x, (float) pose_stamped.pose.position.y, (float) pose_stamped.pose.position.z, 1.0f) ;
		sensor_pose.orientation = Eigen::Quaternionf((float) pose_stamped.pose.orientation.w, (float) pose_stamped.pose.orientation.x, 
							     (float) pose_stamped.pose.orientation.y, (float) pose_stamped.pose.orientation.z) ;
		return true ;
	}
}

void addPointCloudToScene()
{
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
			addPointCloudToScene() ;
			ROS_INFO("Adding point cloud [%d, %d]", msg->header.stamp.sec, msg->header.stamp.nsec) ;

			//Remove message from queue
			cloudMsgQueue.pop_front() ;	
		} else break ;
	}
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
	ROS_INFO("pathCallback: [%s]", msg->header.frame_id.c_str());
	current_path = msg ;
}

void keyframeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("keyframeCallback: [%s]", msg->header.frame_id.c_str());
	//Add point cloud to our local queue (the queue is needed since we must sometimes wait for a transform from a path)
	cloudMsgQueue.push_back(msg) ;	
	processCloudMsgQueue() ;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "surfel_mapper");
	ros::NodeHandle n;

	ros::Subscriber sub_path = n.subscribe("path", 3, pathCallback);
	ros::Subscriber sub_keyframe = n.subscribe("keyframes", 5, keyframeCallback);

	ros::Rate r(2) ;
	while(ros::ok()) {
		ros::spinOnce();
		processCloudMsgQueue() ;
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

