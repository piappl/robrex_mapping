#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <Eigen/Geometry>
#include <limits.h>

#include <pcl/common/common_headers.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/common/common.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#define DMAX 0.05f
#define CLOUD_WIDTH 640
#define CLOUD_HEIGHT 480
#define MIN_KINECT_DIST 0.8 
#define MAX_KINECT_DIST 4.0
#define OCTREE_RESOLUTION 0.2 

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

pcl::octree::OctreePointCloudPointVector<pcl::PointXYZRGB> octree(OCTREE_RESOLUTION) ;

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
 * negative z - denotes sampling out of depth image bounds, nan - denotes invalid reading at given position of the organized cloud
 */
float getZAtPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float u, float v)
{
	//Use a simplest nearest neighbor approach now (todo: upgrade to bilinear interpolation)
	if (u <= -0.5 || v <= -0.5 || u >= cloud->width - 0.5 || v >= cloud->height - 0.5) { //<= >= instead of < > - easier subsequent modulo search 
		/*float xxx = 1.0 ;
		if (u < -0.5 - xxx || u > cloud->width + xxx || v < -0.5 - xxx || v > cloud->height + xxx)
			ROS_INFO("Out of bounds (u, v) = (%f, %f)", u, v) ;*/
		return -1.0f ;	
		//return std::numeric_limits<float>::quiet_NaN () ;
	} else {
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

//Modified PCL transformPointCloud function aimed at non-rigid homogenous transformations
template <typename PointT, typename Scalar> void transformPointCloudNonRigid (const pcl::PointCloud<PointT> &cloud_in, 
		pcl::PointCloud<PointT> &cloud_out, const Eigen::Matrix<Scalar, 4, 4> &transform)
{
	if (&cloud_in != &cloud_out) {
		// Note: could be replaced by cloud_out = cloud_in
		cloud_out.header = cloud_in.header;
		cloud_out.is_dense = cloud_in.is_dense;
		cloud_out.width = cloud_in.width;
		cloud_out.height = cloud_in.height;
		cloud_out.points.reserve (cloud_out.points.size ());
		cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
		cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
		cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
	}

	if (cloud_in.is_dense) {
		// If the dataset is dense, simply transform it!
		for (size_t i = 0; i < cloud_out.points.size (); ++i) {
			//cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
			Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
			float w = static_cast<float> (transform (3, 0) * pt.coeffRef (0) + transform (3, 1) * pt.coeffRef (1) + transform (3, 2) * pt.coeffRef (2) + transform (3, 3)) ;
			cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)) / w ;
			cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)) / w ;
			cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)) / w ;
		}
	} else {
		// Dataset might contain NaNs and Infs, so check for them first,
		// otherwise we get errors during the multiplication (?)
		for (size_t i = 0; i < cloud_out.points.size (); ++i) {
			if (!pcl_isfinite (cloud_in.points[i].x) || !pcl_isfinite (cloud_in.points[i].y) || !pcl_isfinite (cloud_in.points[i].z))
				continue;
			//cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
			Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
			float w = static_cast<float> (transform (3, 0) * pt.coeffRef (0) + transform (3, 1) * pt.coeffRef (1) + transform (3, 2) * pt.coeffRef (2) + transform (3, 3)) ;
			cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)) / w ;
			cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)) / w ;
			cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)) / w ;
		}
	}
}


void testCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	double alpha = 518.930578 ; //fx
	double cx = 323.483756 ;
	double beta = 517.211658 ; //fy
	double cy = 260.384697 ;

	double width = 640 ;
	double height = 480 ;

	double f = MAX_KINECT_DIST ; 
	double n = MIN_KINECT_DIST ;

	Eigen::Matrix4d viewMatrix ;
	viewMatrix << cloud->sensor_orientation_.toRotationMatrix().cast<double>(), cloud->sensor_origin_.topRows<3>().cast<double>(), 0.0, 0.0, 0.0, 1.0 ;
	viewMatrix = viewMatrix.inverse() ;
	Eigen::Matrix4d projectionMatrix ; 
	projectionMatrix << 2 * alpha / width, 0.0, 2 * cx / width - 1.0, 0.0,
				0.0, 2 * beta / height, 2 * cy / height - 1.0, 0.0,
				0.0, 0.0, (f + n) / (f - n), -2 * f * n / (f - n),
				0.0, 0.0, 1.0, 0.0 ;
	ROS_INFO("Testing projectionview matrix") ;
	std::cout << "View matrix: " << std::endl << viewMatrix << std::endl ;
	std::cout << "Projection matrix: " << std::endl << projectionMatrix << std::endl ;
	Eigen::Matrix4d projectionViewMatrix = projectionMatrix * viewMatrix ;
	
	//Transform point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	transformPointCloudNonRigid<pcl::PointXYZRGB, double>(*cloud, *cloudTemp, projectionViewMatrix) ;

	//Measure cube bounds (all transformed points should be in a cube [-1,1]x[-1,1]x[-1,1]
	float minx, maxx, miny, maxy, minz, maxz ;
	minx = miny = minz = 100.0f ;
	maxx = maxy = maxz = -100.0f ;
	for (uint32_t i = 0; i < cloudTemp->height ; i++) //For cloud scene it will be 1 (unorganized cloud) 
		for (uint32_t j = 0; j < cloudTemp->width ; j++) {
			pcl::PointXYZRGB &point = (*cloudTemp)(j, i) ;	
			if (point.x < minx) minx = point.x ;
			if (point.y < miny) miny = point.y ;
			if (point.z < minz) minz = point.z ;
			if (point.x > maxx) maxx = point.x ;
			if (point.y > maxy) maxy = point.y ;
			if (point.z > maxz) maxz = point.z ;
		}
	ROS_INFO("Projection space cube: x = [%f %f] y = [%f %f] z = [%f %f]", minx, maxx, miny, maxy, minz, maxz) ;

	transformPointCloudNonRigid<pcl::PointXYZRGB, double>(*cloud, *cloudTemp, viewMatrix) ;
	minx = miny = minz = 100.0f ;
	maxx = maxy = maxz = -100.0f ;
	for (uint32_t i = 0; i < cloudTemp->height ; i++) //For cloud scene it will be 1 (unorganized cloud) 
		for (uint32_t j = 0; j < cloudTemp->width ; j++) {
			pcl::PointXYZRGB &point = (*cloudTemp)(j, i) ;	
			if (point.x < minx) minx = point.x ;
			if (point.y < miny) miny = point.y ;
			if (point.z < minz) minz = point.z ;
			if (point.x > maxx) maxx = point.x ;
			if (point.y > maxy) maxy = point.y ;
			if (point.z > maxz) maxz = point.z ;
		}
	ROS_INFO("Camera space cube: x = [%f %f] y = [%f %f] z = [%f %f]", minx, maxx, miny, maxy, minz, maxz) ;

	minx = miny = minz = 100.0f ;
	maxx = maxy = maxz = -100.0f ;
	for (uint32_t i = 0; i < cloud->height ; i++) //For cloud scene it will be 1 (unorganized cloud) 
		for (uint32_t j = 0; j < cloud->width ; j++) {
			pcl::PointXYZRGB &point = (*cloud)(j, i) ;	
			if (point.x < minx) minx = point.x ;
			if (point.y < miny) miny = point.y ;
			if (point.z < minz) minz = point.z ;
			if (point.x > maxx) maxx = point.x ;
			if (point.y > maxy) maxy = point.y ;
			if (point.z > maxz) maxz = point.z ;
		}
	ROS_INFO("World space cube: x = [%f %f] y = [%f %f] z = [%f %f]", minx, maxx, miny, maxy, minz, maxz) ;

	//Computing frustum
	double frustum[24] ;
	pcl::visualization::getViewFrustum(projectionViewMatrix, frustum) ;

	ROS_INFO("Testing frustum") ;
	//Testing frustum (testing a series of world space cubes)
	for (double x = -3.0; x <= 3.0 ; x += 0.5) 
		for (double y = -3.0; y <= 3.0 ; y += 0.5) 
			for (double z = -1.0; z <= 3.0 ; z += 0.5) {
				Eigen::Vector3d min_bb(x, y, z) ;
				Eigen::Vector3d max_bb(x + 0.5, y + 0.5, z + 0.5) ;
				if (pcl::visualization::cullFrustum(frustum, min_bb, max_bb) == pcl::visualization::PCL_INSIDE_FRUSTUM)
					ROS_INFO("Cube: x = [%f %f] y = [%f %f] z = [%f %f] %d", min_bb.x(), max_bb.x(), min_bb.y(), max_bb.y(), min_bb.z(), max_bb.z(), pcl::visualization::PCL_INSIDE_FRUSTUM) ;
			}
	
}

void filterCloudByDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	//TODO: this filtering requires cloud transformation and takes 10-20 ms - consider performing this at the time of image acquisition
	//Transform cloud to the original frame	
	Eigen::Matrix4f trans ;
	trans << cloud->sensor_orientation_.toRotationMatrix(), cloud->sensor_origin_.topRows<3>(), 0.0, 0.0, 0.0, 1.0 ;
	Eigen::Matrix4f transinv = trans.inverse() ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::transformPointCloud(*cloud, *cloudTemp, transinv) ;

	//int pointsUpdated = 0 ;
	//Manually NaNing points outside effective Kinect scope
	for (uint32_t i = 0; i < cloudTemp->height ; i++) //For cloud scene it will be 1 (unorganized cloud) 
		for (uint32_t j = 0; j < cloudTemp->width ; j++) {
			float zscan = (*cloudTemp)(j, i).z ;	
			if (!std::isnan(zscan) && (zscan > MAX_KINECT_DIST || zscan < MIN_KINECT_DIST)) {
				pcl::PointXYZRGB &point = (*cloud)(j, i) ;	
				point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN () ;
				//pointsUpdated++ ;
			}
		}

	//ROS_INFO("Number of points outside Kinect scope removed: [%d]", pointsUpdated) ;
		
	//Cannot make out how to specify these filters - let's do it manually...
	//Mark points that are too close or too far by indices
	/*pcl::PassThrough<pcl::PointXYZRGB> ptfilt;
	ptfilt.setInputCloud(cloudTemp) ;
	ptfilt.setFilterFieldName("z") ;
	ptfilt.setFilterLimits(MIN_KINECT_DIST, MAX_KINECT_DIST) ;
	pcl::PointIndices::Ptr indices(new pcl::PointIndices()) ;
	ptfilt.filter(indices->indices) ;
	std::cout << indices->indices.size() ;*/

	//Mark filtered points in the original cloud as a NaN
	/*pcl::ExtractIndices<pcl::PointXYZRGB> indfilt ;
	indfilt.setIndices(indices) ;
	indfilt.setNegative(true) ;
	indfilt.filterDirectly(cloud) ;*/
}

void addPointCloudToScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	//Filter points too close and too far
	ros::Time start = ros::Time::now();
	filterCloudByDistance(cloud) ;
	ros::Time stop = ros::Time::now() ;
	ROS_INFO("Filtering points outside reliable Kinect scope time (s): [%.6lf]", (stop - start).toSec()) ;

	//Testing cloud frustum
	testCloud(cloud) ;

	//Compute a transformation to bring a scene cloud into new cloud frame	
	Eigen::Matrix4f trans ;
	trans << cloud->sensor_orientation_.toRotationMatrix(), cloud->sensor_origin_.topRows<3>(), 0.0, 0.0, 0.0, 1.0 ;
	//std::cout << trans << std::endl ;
	Eigen::Matrix4f transinv = trans.inverse() ;
	//std::cout << transinv << std::endl ;

	//Transform scene cloud into its original camera frame	
	start = ros::Time::now();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneTrans(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::transformPointCloud(*cloudScene, *cloudSceneTrans, transinv) ;
	stop = ros::Time::now() ;
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
			if (!std::isnan(zscan)) { //not valid - check the newer function...
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

inline void transformPointAffine(pcl::PointXYZRGB &point_in, pcl::PointXYZRGB &point_out, Eigen::Matrix4d transform)
{
	point_out = point_in ;
	point_out.x = static_cast<float> (transform (0, 0) * point_in.x + transform (0, 1) * point_in.y + transform (0, 2) * point_in.z + transform (0, 3));
	point_out.y = static_cast<float> (transform (1, 0) * point_in.x + transform (1, 1) * point_in.y + transform (1, 2) * point_in.z + transform (1, 3));
	point_out.z = static_cast<float> (transform (2, 0) * point_in.x + transform (2, 1) * point_in.y + transform (2, 2) * point_in.z + transform (2, 3));
}

void addPointCloudToScene1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	//Filter points too close and too far
	ros::Time start = ros::Time::now();
	filterCloudByDistance(cloud) ;
	ros::Time stop = ros::Time::now() ;
	ROS_INFO("Filtering points outside reliable Kinect scope time (s): [%.6lf]", (stop - start).toSec()) ;

	//Testing cloud frustum
	//testCloud(cloud) ;

	double alpha = 518.930578 ; //fx
	double cx = 323.483756 ;
	double beta = 517.211658 ; //fy
	double cy = 260.384697 ;

	double width = 640 ;
	double height = 480 ;

	//Compute a projection view matrix
	double f = MAX_KINECT_DIST ; 
	double n = MIN_KINECT_DIST ;

	Eigen::Matrix4d viewMatrix ;
	viewMatrix << cloud->sensor_orientation_.toRotationMatrix().cast<double>(), cloud->sensor_origin_.topRows<3>().cast<double>(), 0.0, 0.0, 0.0, 1.0 ;
	viewMatrix = viewMatrix.inverse() ;
	//Eigen::Matrix4f viewMatrixf = viewMatrix.cast<float>() ;

	Eigen::Matrix4d projectionMatrix ; 
	projectionMatrix << 2 * alpha / width, 0.0, 2 * cx / width - 1.0, 0.0,
				0.0, 2 * beta / height, 2 * cy / height - 1.0, 0.0,
				0.0, 0.0, (f + n) / (f - n), -2 * f * n / (f - n),
				0.0, 0.0, 1.0, 0.0 ;
	//std::cout << "View matrix: " << std::endl << viewMatrix << std::endl ;
	//std::cout << "Projection matrix: " << std::endl << projectionMatrix << std::endl ;

	Eigen::Matrix4d projectionViewMatrix = projectionMatrix * viewMatrix ;

	//Computing frustum
	double frustum[24] ;
	pcl::visualization::getViewFrustum(projectionViewMatrix, frustum) ;

	//Clean-up the scan covered array
	static char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH] ;
	memset(scan_covered, 0, sizeof(scan_covered[0][0]) * CLOUD_HEIGHT * CLOUD_WIDTH);

	unsigned int nsurfels_updated = 0 ;
	unsigned int octree_nodes_visited = 0 ;
	unsigned int surfels_inside_octree_frustum = 0 ;
	unsigned int surfels_projected_on_sensor = 0 ;
	unsigned int nscan_too_close = 0 ;
	unsigned int nscan_too_far = 0 ;
	unsigned int nsurfels_invalid_reading = 0 ;
	start = ros::Time::now();

	//Iterate Octree in a depth-first manner
	unsigned int acceptBelowDepth = UINT_MAX ;
	pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator it = octree.depth_begin() ;
	const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator it_end = octree.depth_end();
	while(it != it_end) {
		octree_nodes_visited++ ;
		unsigned int current_depth = it.getCurrentOctreeDepth() ;

		//Cancel acceptBelowDepth if we went above a child branch that is completely in a frustum
		if (current_depth <= acceptBelowDepth)
			acceptBelowDepth = UINT_MAX ;

		//Compute frustum if necessary
		int frustum_result ;
		if (current_depth > acceptBelowDepth)  
			frustum_result = pcl::visualization::PCL_INSIDE_FRUSTUM ;
		else {
			Eigen::Vector3f min_bb, max_bb ;
			octree.getVoxelBounds(it, min_bb, max_bb) ;	
			frustum_result = pcl::visualization::cullFrustum(frustum, min_bb.cast<double>(), max_bb.cast<double>()) ; 
			if (frustum_result == pcl::visualization::PCL_INSIDE_FRUSTUM)
				acceptBelowDepth = it.getCurrentOctreeDepth() ; //We may mark that all nodes below will be automatically accepted
		}
	
		if (frustum_result == pcl::visualization::PCL_OUTSIDE_FRUSTUM) 
			it.skipChildVoxels() ;
		else if (it.isLeafNode()) {
			//Transform and update all points in a leaf
			pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
			std::vector<int> pointIndices ; 
			container.getPointIndices (pointIndices);
			pcl::PointXYZRGB pointTrans ;
			for (int i = 0; i < pointIndices.size() ; i++)  {
				transformPointAffine(cloudScene->points[pointIndices[i]], pointTrans, viewMatrix) ;
				float xp = pointTrans.x / pointTrans.z ;
				float yp = pointTrans.y / pointTrans.z ;
				float u = alpha * xp + cx ;
				float v = beta * yp + cy ;
				float zscan = getZAtPosition(cloud, u, v) ;
				surfels_inside_octree_frustum++ ;
				if (std::isnan(zscan) || zscan >= 0.0f) //in both cases we hit image plane
					surfels_projected_on_sensor++ ;
				if (!std::isnan(zscan) && zscan >= 0.0f) {
					//surfels_projected_on_sensor++ ;
					if (fabs(zscan - pointTrans.z) <= DMAX) { 
						//We have a surfel-scan match, we may update the surfel here... (TODO)
						markScanAsCovered(scan_covered, u, v) ; 
						nsurfels_updated++ ;
					} else if (zscan - pointTrans.z > DMAX) {
						//The observed point is behing the surfel, we may either remove the observation or the surfel (depending e.g. on the confidence)
						markScanAsCovered(scan_covered, u, v) ;
						nscan_too_far++ ;
					} else
						nscan_too_close++ ;
				} else nsurfels_invalid_reading++ ;
			}
		}		
		it++ ;
	}
	
	stop = ros::Time::now() ;
	ROS_INFO("Surfels inside octree frustum [%d]", surfels_inside_octree_frustum) ;
	ROS_INFO("Surfels projected on sensor plane [%d]", surfels_projected_on_sensor) ;
	ROS_INFO("Projected/inside frustum [%f%%]", double(surfels_projected_on_sensor)/surfels_inside_octree_frustum * 100) ;
	ROS_INFO("Outside frustum/total points [%f%%]", double(cloudScene->width - surfels_inside_octree_frustum) / cloudScene->width * 100) ;
	ROS_INFO("Octree nodes visited during update [%d]", octree_nodes_visited) ;
	ROS_INFO("Surfel update time (s): [%.6lf]", (stop - start).toSec()) ;
	ROS_INFO("Surfels updated [%d]", nsurfels_updated) ;
	ROS_INFO("Scan too far for surfel update [%d]", nscan_too_far) ;
	ROS_INFO("Scan too close for surfel update [%d]", nscan_too_close) ;
	ROS_INFO("Surfels without matching reading [%d]", nsurfels_invalid_reading) ;

	start = ros::Time::now() ;
	//Perform surfel-addition step
	//Create temporary point cloud (of surfels) to be concatenated with the scene cloud (TODO we may do without intermediary cloud, perhaps faster)	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::copyPointCloud(*cloud, *cloudTemp) ;

	unsigned int surfels_added = 0 ;
	//Update surfel data in the cloud to add and remove covered measurements
	for (uint32_t i = 0; i < cloud->height ; i++) 
		for (uint32_t j = 0; j < cloud->width ; j++) { 
			if (!scan_covered[i][j]) {
				//Add a new point to the scene cloud (and the associated octree)
				pcl::PointXYZRGB &point = (*cloud)(j, i) ;
				if (pcl::isFinite(point)) {
					octree.addPointToCloud(point, cloudScene) ;
					surfels_added++ ;
					//TODO Some other (more complex) processing is required here...
				}
			}
		}
	stop = ros::Time::now() ;
	ROS_INFO("Surfel addition time (s): [%.6lf]", (stop - start).toSec()) ;
	ROS_INFO("Surfels added [%d]", surfels_added) ;
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
			addPointCloudToScene1(cloud) ;
			ROS_INFO("-------------->Adding point cloud [%d, %d]", msg->header.stamp.sec, msg->header.stamp.nsec) ;
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

	octree.setInputCloud(cloudScene) ;

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

