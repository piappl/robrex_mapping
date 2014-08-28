#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <Eigen/Geometry>

#include <pcl/common/common_headers.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#define DMAX 0.05f
#define CLOUD_WIDTH 640
#define CLOUD_HEIGHT 480

struct SensorPose {
	public:
		Eigen::Quaternionf orientation ;
		Eigen::Vector4f origin ;
} ;

//typedef std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudMsgListT ;
typedef std::list<sensor_msgs::PointCloud2::ConstPtr> PointCloudMsgListT ;

nav_msgs::Path::ConstPtr current_path ;
PointCloudMsgListT cloudMsgQueue ;

//Let us define our main scene cloud (will contain surfels soon...)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudScene(new pcl::PointCloud<pcl::PointXYZRGB>) ;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>) ;

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

/**
 * Gets interpolated -z- value at specified (not necesserily integer) position in organized cloud
 */
float getZAtPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float u, float v)
{
	//Use a simplest nearest neighbor approach now (todo: upgrade to bilinear interpolation)
	if (u <= -0.5 || v <= -0.5 || u >= cloud->width - 0.5 || v >= cloud->height - 0.5) //<= >= instead of < > - easier subsequent modulo search
		return std::numeric_limits<float>::quiet_NaN () ;
	else {
		//Now perform NN interpolation
		uint32_t i = static_cast<int>(v + 0.5) ;
		uint32_t j = static_cast<int>(u + 0.5) ;
		return (*cloud)(j, i).z ; 
	}
}

void markScanAsCovered(char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH], float u, float v) 
{
	//Here we assume that the covering surfel is approximately the size of single scan pixel 
	//TODO: In some cases surfel ahead of the scan and scan was invalidated, the surfel may be larger and cover multiple can pixels - it might be worthwhile to take it into account 

	//Perform nearest-neighbor search
	uint32_t i = static_cast<int>(v + 0.5) ;
	uint32_t j = static_cast<int>(u + 0.5) ;

	//Since the function is called, the range of i, j should be correct...
	scan_covered[i][j] = 1 ;
}

void downsampleSceneCloud()
{
	//Downsample cloud for publication in a topic
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloudScene);
	sor.setLeafSize (0.05f, 0.05f, 0.05f);
	sor.filter (*cloudSceneDownsampled);
}

void addPointCloudToScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	//Compute a transformation to bring a scene cloud into new cloud frame	
	Eigen::Matrix4f trans ;
	trans << cloud->sensor_orientation_.toRotationMatrix(), cloud->sensor_origin_.topRows<3>(), 0.0, 0.0, 0.0, 1.0 ;
	//std::cout << trans << std::endl ;
	Eigen::Matrix4f transinv = trans.inverse() ;
	//std::cout << transinv << std::endl ;

	//Transform scene cloud into its original camera frame	
	ros::Time start = ros::Time::now();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneTrans(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::transformPointCloud(*cloudScene, *cloudSceneTrans, transinv) ;
	ros::Time stop = ros::Time::now() ;
	ROS_INFO("Rigid Transform time (s): [%.6lf]", (stop - start).toSec()) ;

	//Define intrinsic matrix (so far - added manually from external file) (TODO: use camera_info topics)
	Eigen::Matrix3f intMat ;
	intMat << 518.930578, 0.0, 323.483756, 0.0, 517.211658, 260.384697, 0.0, 0.0, 1.0 ;
	//std::cout << intMat ;	

	float fx = intMat(0, 0) ;
	float fy = intMat(1, 1) ;
	float cx = intMat(0, 2) ;
	float cy = intMat(1, 2) ;
	
	//Clean-up the scan covered array
	static char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH] ;
	memset(scan_covered, 0, sizeof(scan_covered[0][0]) * CLOUD_HEIGHT * CLOUD_WIDTH);

	int nsurfels_updated = 0 ;
	start = ros::Time::now();
	//Perform surfel-update step	
	//std::cout << "Cloud size " << cloudSceneTrans->width << " " << cloudSceneTrans->height ;
	for (uint32_t i = 0; i < cloudSceneTrans->height ; i++) //For cloud scene it will be 1 (unorganized cloud) 
		for (uint32_t j = 0; j < cloudSceneTrans->width ; j++) { 
			pcl::PointXYZRGB &point = (*cloudSceneTrans)(j, i) ;	
			float xp = point.x / point.z ;
			float yp = point.y / point.z ;
			float u = fx * xp + cx ;
			float v = fy * yp + cy ;
			//std::cout << "(" << j << ", " << i << ")-(" << u << ", " << v << ")" << std::endl ;
			//std::cout << i << ": (" << j - u << ", " << i - v << ")" << std::endl ;
			
			float zscan = getZAtPosition(cloud, u, v) ;
			if (!std::isnan(zscan)) {
				if (fabs(zscan - point.z) <= DMAX) { 
					//We have a surfel-scan match, we may update the surfel here... (TODO)
					markScanAsCovered(scan_covered, u, v) ; 
					nsurfels_updated++ ;
				} else if (zscan - point.z < -DMAX) {
					//The observed point is behing the surfel, we may either remove the observation or the surfel (depending e.g. on the confidence)
					markScanAsCovered(scan_covered, u, v) ; 
				}
			}
			
		}
	stop = ros::Time::now() ;
	ROS_INFO("Surfel update time (s): [%.6lf]", (stop - start).toSec()) ;
	ROS_INFO("Surfels updated [%d]", nsurfels_updated) ;

	start = ros::Time::now() ;
	//Perform surfel-addition step
	//Create temporary point cloud (of surfels) to be concatenated with the scene cloud (TODO we may do without intermediary cloud, perhaps faster)	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::copyPointCloud(*cloud, *cloudTemp) ;

	//Update surfel data in the cloud to add and remove covered measurements
	for (uint32_t i = 0; i < cloudTemp->height ; i++) 
		for (uint32_t j = 0; j < cloudTemp->width ; j++) { 
			pcl::PointXYZRGB &point = (*cloudTemp)(j, i) ;	
			if (scan_covered[i][j])
				point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN () ;
			//TODO Some other (more complex) processing is required here...
		}
	//Remove NaNs (resulting from scan and from coverage analysis)
	std::vector<int> indices ;
        pcl::removeNaNFromPointCloud(*cloudTemp,*cloudTemp, indices) ;

	//Now concatenate scen cloud and scanner cloud
	*cloudScene += *cloudTemp ;	
	stop = ros::Time::now() ;
	ROS_INFO("Surfel addition time (s): [%.6lf]", (stop - start).toSec()) ;
	ROS_INFO("Surfels added [%d]", cloudTemp->width) ;
	ROS_INFO("cloud_scene size: [%d]", cloudScene->width) ;

	//Now downsample scene cloud
	start = ros::Time::now() ;
	downsampleSceneCloud() ;
	stop = ros::Time::now() ;
	ROS_INFO("Cloud downsampling time(s): [%.6lf]", (stop - start).toSec()) ;
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
			addPointCloudToScene(cloud) ;
			ROS_INFO("Adding point cloud [%d, %d]", msg->header.stamp.sec, msg->header.stamp.nsec) ;
			ROS_INFO("Sensor position data: [%f, %f, %f, %f] ", cloud->sensor_origin_.x(), cloud->sensor_origin_.y(), cloud->sensor_origin_.z(), cloud->sensor_origin_.w()) ;
			ROS_INFO("Sensor orientation data: [%f, %f, %f, %f] ", cloud->sensor_orientation_.x(), cloud->sensor_orientation_.y(), cloud->sensor_orientation_.z(), cloud->sensor_orientation_.w()) ;

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

void sendMapMessage(ros::Publisher &map_pub) 
{
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

	ros::Subscriber sub_path = n.subscribe("path", 3, pathCallback);
	ros::Subscriber sub_keyframe = n.subscribe("keyframes", 5, keyframeCallback);

	ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("surfelmap", 5);

	ros::Rate r(2) ;
	while(ros::ok()) {
		ros::spinOnce();
		processCloudMsgQueue() ;
		ros::Time start = ros::Time::now() ;
		sendMapMessage(map_pub) ;
		ros::Time stop = ros::Time::now() ;
		ROS_INFO("Sending Map Message time (s): [%.6lf]", (stop - start).toSec()) ;
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

