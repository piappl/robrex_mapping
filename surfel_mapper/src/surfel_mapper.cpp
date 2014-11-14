#include "surfel_mapper.hpp"
#include <pcl/common/transforms.h>
#include <pcl/visualization/common/common.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/io.h>
#include <pcl/features/integral_image_normal.h>
#include "logger.hpp"

#define DMAX 0.005f
#define MIN_KINECT_DIST 0.8 
#define MAX_KINECT_DIST 4.0
#define OCTREE_RESOLUTION 0.2 
//#define OCTREE_RESOLUTION 500.0 
#define PREVIEW_RESOLUTION 0.2
#define PREVIEW_COLOR_SAMPLES_IN_VOXEL 3
#define CONFIDENCE_THRESHOLD1 5
#define MIN_SCAN_ZNORMAL 0.2f

extern Logger logger ;

inline void SurfelMapper::transformPointAffine(PointCustomSurfel &point_in, PointCustomSurfel &point_out, Eigen::Matrix4d transform)
{
	point_out = point_in ;
	point_out.x = static_cast<float> (transform (0, 0) * point_in.x + transform (0, 1) * point_in.y + transform (0, 2) * point_in.z + transform (0, 3));
	point_out.y = static_cast<float> (transform (1, 0) * point_in.x + transform (1, 1) * point_in.y + transform (1, 2) * point_in.z + transform (1, 3));
	point_out.z = static_cast<float> (transform (2, 0) * point_in.x + transform (2, 1) * point_in.y + transform (2, 2) * point_in.z + transform (2, 3));
}

template <typename PointT, typename Scalar> void SurfelMapper::transformPointCloudNonRigid (const pcl::PointCloud<PointT> &cloud_in, 
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

float SurfelMapper::getZAtPosition(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, float u, float v)
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

void SurfelMapper::getPointAtPosition(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_trans, float u, float v, pcl::PointXYZRGBNormal &point, pcl::PointXYZRGBNormal &point_trans)
{
	//Use a simplest nearest neighbor approach now (todo: upgrade to bilinear interpolation)
	if (u <= -0.5 || v <= -0.5 || u >= cloud->width - 0.5 || v >= cloud->height - 0.5) { //<= >= instead of < > - easier subsequent modulo search 
		point.x = point.y = point.z =  std::numeric_limits<float>::quiet_NaN () ; //Only point is checked afterwards, no need to update point_trans
	} else {
		//Now perform NN interpolation
		uint32_t i = static_cast<int>(v + 0.5) ;
		uint32_t j = static_cast<int>(u + 0.5) ;
		point = (*cloud)(j,i) ;
		point_trans = (*cloud_trans)(j,i) ;
	}
}

void SurfelMapper::markScanAsCovered(char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH], float u, float v) 
{
	//Here we assume that the covering surfel is approximately the size of single scan pixel 
	//TODO: In some cases surfel ahead of the scan and scan was invalidated, the surfel may be larger and cover multiple can pixels - it might be worthwhile to take it into account 

	//Perform nearest-neighbor search
	uint32_t i = static_cast<int>(v + 0.5) ;
	uint32_t j = static_cast<int>(u + 0.5) ;

	//Since the function is called, the range of i, j should be correct...
	scan_covered[i][j] = 1 ;
}

void SurfelMapper::skipChildVoxelsCorrect(pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator &it_end)
{
	unsigned int current_depth = it.getCurrentOctreeDepth() ;
	it++ ;	
	if (it != it_end && it.getCurrentOctreeDepth() > current_depth)
		it.skipChildVoxels() ; //Actually we skip siblings of the child here
}

void SurfelMapper::computeVoxelColor(pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator &it_end, pcl::PointXYZRGB &point)
{
	//Select a few pixels from the current voxel and compute an average	
	unsigned int current_depth = it.getCurrentOctreeDepth() ;
	unsigned long rs, gs, bs ;
	rs = gs = bs = 0 ;
	unsigned long count  = 0;
	bool first_it = true ; //We must take into account also the starting node - it might be a leaf!
	while(it != it_end && (it.getCurrentOctreeDepth() > current_depth || first_it)) {
		first_it = false ;
		if (it.isLeafNode()) {
			//Examine points in the voxel	
			pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
			std::vector<int> pointIndices ; 
			container.getPointIndices (pointIndices);
			unsigned int step = pointIndices.size() / PREVIEW_COLOR_SAMPLES_IN_VOXEL;
			if (step < 1) step = 1 ;
			//Now select every "step" - point
			for (unsigned int i = 0; i < pointIndices.size() ; i += step) {
				PointCustomSurfel p = cloudScene->points[pointIndices[i]] ;
				rs += p.r ;
				gs += p.g ;
				bs += p.b ;
				count++ ;
			}
		}
		it++ ;
	}
	if (count > 0) {
		point.r = rs / count ;
		point.g = gs / count ;
		point.b = bs / count ;
	}
}

void SurfelMapper::filterCloudByDistance(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud)
{
	//int pointsUpdated = 0 ;
	//Manually NaNing points outside effective Kinect scope and those with too large angle of view
	//TODO: There are some points (about 0.1%) that have positive normals after cloud transformation - investigate why?
	for (uint32_t i = 0; i < cloud->height ; i++) //For cloud scene it will be 1 (unorganized cloud) 
		for (uint32_t j = 0; j < cloud->width ; j++) {
			float zscan = (*cloud)(j, i).z ;	
			float znormalscan = (*cloud)(j, i).normal_z ;
			if (!std::isnan(zscan) && !std::isnan(znormalscan) && (zscan > MAX_KINECT_DIST || zscan < MIN_KINECT_DIST || znormalscan > -MIN_SCAN_ZNORMAL)) {
				pcl::PointXYZRGBNormal &point = (*cloud)(j, i) ;	
				point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN () ;
			}
		}
}

void SurfelMapper::downsampleSceneCloud()
{
	//Establish maximum tree depth for display
	unsigned int tree_depth = octree.getTreeDepth() ;
	unsigned int display_depth = tree_depth ;
	for (unsigned int depth = 1; depth <= tree_depth ; depth++) {
		double voxel_side = sqrt(octree.getVoxelSquaredSideLen(depth)) ;
		if (voxel_side <= PREVIEW_RESOLUTION) {
			display_depth = depth ;
			break ;
		}
	}

	//Clear point cloud
	cloudSceneDownsampled->clear() ;

	//Convert voxels at fixed depth to points in a downsampled cloud
	pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator it = octree.depth_begin() ;
	const pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator it_end = octree.depth_end();
	while(it != it_end) {
		unsigned int current_depth = it.getCurrentOctreeDepth() ;
		if (current_depth == display_depth) {
			//Convert a voxel to a single point 
			Eigen::Vector3f min_bb, max_bb ;
			octree.getVoxelBounds(it, min_bb, max_bb) ;	
			
			pcl::PointXYZRGB point ;
			point.x = (min_bb[0] + max_bb[0]) / 2 ;
			point.y = (min_bb[1] + max_bb[1]) / 2 ;
			point.z = (min_bb[2] + max_bb[2]) / 2 ;
			point.r = point.g = point.b = 255 ;
			point.a = 255 ;

			computeVoxelColor(it, it_end, point) ; //Computes average color from some selected voxel points and performs skip child voxels procedure at the same time

			//Add to point cloud
			cloudSceneDownsampled->push_back(point) ;

			//Ignore children
			//it.skipChildVoxels() ;	
			//skipChildVoxelsCorrect(it, it_end) ;

			//double voxel_side = sqrt(octree.getVoxelSquaredSideLen(current_depth)) ;
			//std::cout << "side = " << max_bb[0] - min_bb[0] << " " ;
		} else it++ ;
	}

	/*
	//DEBUG!!!!Copy original cloud to downsampled cloud
	cloudSceneDownsampled->clear() ;
	for (uint32_t i = 0; i < cloudScene->width ; i++) {
		pcl::PointXYZRGB point ;

		point.x = cloudScene->points[i].x ;
		point.y = cloudScene->points[i].y ;
		point.z = cloudScene->points[i].z ;

		point.r = cloudScene->points[i].r ;
		point.g = cloudScene->points[i].g ;
		point.b = cloudScene->points[i].b ;
		point.a = 255 ;

		//Add to point cloud
		cloudSceneDownsampled->push_back(point) ;
	}
	*/

}

SurfelMapper::SurfelMapper(): cloudScene(new pcl::PointCloud<PointCustomSurfel>), cloudSceneDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>), octree(OCTREE_RESOLUTION)
{
	//octree.defineBoundingBox(-100,-100,-100, 400, 400, 400) ;	
	octree.setInputCloud(cloudScene) ;
}

SurfelMapper::~SurfelMapper()
{}

bool IsNegative (int i) { return i < 0 ; }

void SurfelMapper::addPointCloudToScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	pcl::StopWatch timer ;
	
	//Testing cloud frustum
	//testCloud(cloud) ;

	double alpha = 518.930578 ; //fx
	double cx = 323.483756 ;
	double beta = 517.211658 ; //fy
	double cy = 260.384697 ;

	//double zTor = 0.25 * (1.0 / alpha + 1.0 / beta) ;
	double zTor = 1.0/(sqrt(2.0) * (alpha + beta) / 2.0) ;

	double width = 640 ;
	double height = 480 ;


	//Compute a view matrix
	Eigen::Matrix4d viewMatrix ;
	viewMatrix << cloud->sensor_orientation_.toRotationMatrix().cast<double>(), cloud->sensor_origin_.topRows<3>().cast<double>(), 0.0, 0.0, 0.0, 1.0 ;

	//std::cout << "Transform matrix used:" << std::endl ;
	//std::cout <<  viewMatrix ;
	//std::cout << std::endl ;
	//std::cout.flush() ;
	//viewMatrix = viewMatrix.inverse() * cameraRgbToCameraLinkTrans ;
	viewMatrix = viewMatrix.inverse() ;


	//Compute normals for the input cloud
	timer.reset() ;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>) ;
	pcl::copyPointCloud(*cloud, *cloudNormals) ;	
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloudNormals);
	ne.useSensorOriginAsViewPoint() ;
        ne.compute(*cloudNormals);
	std::cout << "Normal computation for the frame [" << timer.getTimeSeconds() << "]" << std::endl ;
	logger.log("normal_computation_time", timer.getTimeSeconds()) ;

	//Filter-out incorrect normals
	//TODO:could be possibly merged with a filterCloudByDistance function
	unsigned int ncorrect_scans = 0 ;
	unsigned int ncorrect_scans_and_normals = 0 ;
	timer.reset() ;
	for (uint32_t i = 0; i < cloudNormals->height ; i++) 
		for (uint32_t j = 0; j < cloudNormals->width ; j++) {
			if (!std::isnan((*cloudNormals)(j, i).z)) {
				ncorrect_scans++ ;
				if (!std::isnan((*cloudNormals)(j, i).normal_x)) 
					ncorrect_scans_and_normals++ ;
				else {
					pcl::PointXYZRGBNormal &point = (*cloudNormals)(j, i) ;	
					point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN () ;
				}
			}
		}
	std::cout << "Filtering out incorrect normals [" << timer.getTimeSeconds() << "]" << std::endl ;
	logger.log("normal_filtering_time", timer.getTimeSeconds()) ;

	//Transform input cloud into camera coordinate system (each keyframe is referenced to the global coord. system by ccny_rgbd) 
	timer.reset() ;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudNormalsTrans(new pcl::PointCloud<pcl::PointXYZRGBNormal>) ;
	pcl::transformPointCloudWithNormals(*cloudNormals, *cloudNormalsTrans, viewMatrix) ;
	std::cout << "Keyframe transformation into original camera frame time (s): [" << timer.getTimeSeconds() << "]" << std::endl ;
	logger.log("keyframe_transformation_time", timer.getTimeSeconds()) ;

	//Debug - display the cloud transformed back 
	/*for (uint32_t i = 0; i < cloud->height ; i++)  {
		for (uint32_t j = 0; j < cloud->width ; j++) { 
			pcl::PointXYZRGBNormal pointTrans = (*cloudNormalsTrans)(j, i) ;
			if (i % 50 == 0 && j % 50 ==0 && pcl::isFinite(pointTrans)) {
				float xp = pointTrans.x / pointTrans.z ;
				float yp = pointTrans.y / pointTrans.z ;
				float u = alpha * xp + cx ;
				float v = beta * yp + cy ;
				std::cout << "(" << j << "," << i << "):" << u << " " << v << " " << pointTrans.z << " \n" ;
			}
		}
	}*/
	

	//Filter points too close and too far
	
	timer.reset() ;	
	filterCloudByDistance(cloudNormalsTrans) ;
	std::cout << "Filtering points outside reliable Kinect scope time (s): [" << timer.getTimeSeconds() << "]" << std::endl ;
	logger.log("scope_filtering_time", timer.getTimeSeconds()) ;
	
	//Compute a projection matrix	
	double f = MAX_KINECT_DIST + DMAX ; //When filtering surfels we want to have slightly larger aperture than for the scan cloud 
	double n = MIN_KINECT_DIST - DMAX ;
	Eigen::Matrix4d projectionMatrix ; 
	projectionMatrix << 2 * alpha / width, 0.0, 2 * cx / width - 1.0, 0.0,
				0.0, 2 * beta / height, 2 * cy / height - 1.0, 0.0,
				0.0, 0.0, (f + n) / (f - n), -2 * f * n / (f - n),
				0.0, 0.0, 1.0, 0.0 ;
	//std::cout << "View matrix: " << std::endl << viewMatrix << std::endl ;
	//std::cout << "Projection matrix: " << std::endl << projectionMatrix << std::endl ;

	//Compute a projection-view matrix
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
	unsigned int nsurfels_removed = 0 ;
	unsigned int ntotal_scans = 0 ;

	timer.reset() ;
	/*float umin = 1e6 ;
	float umax = -1e6 ;
	float vmin = 1e6 ;
	float vmax = -1e6 ;*/
	
	//Iterate Octree in a depth-first manner
	unsigned int acceptBelowDepth = UINT_MAX ;
	pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator it = octree.depth_begin() ;
	const pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator it_end = octree.depth_end();
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
			skipChildVoxelsCorrect(it, it_end) ;
		else { 
			if (it.isLeafNode()) {
				//Transform and update all points in a leaf
				pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
				std::vector<int> &pointIndices  = container.getPointIndicesVector() ;
				//std::vector<int> pointIndices ; 
				//container.getPointIndices(pointIndices) ;

				PointCustomSurfel pointTrans ;
				for (int i = 0; i < pointIndices.size() ; i++)  {
					surfels_inside_octree_frustum++ ;
					transformPointAffine(cloudScene->points[pointIndices[i]], pointTrans, viewMatrix) ; //TODO: might perform unnecessary copying (we need only xyz, not the metadata...)
					if (pointTrans.z <= MAX_KINECT_DIST + DMAX && pointTrans.z >= MIN_KINECT_DIST - DMAX) { //In frustum cullling we remove surfels too close or too far, should we be consistent in that? 
						float xp = pointTrans.x / pointTrans.z ;
						float yp = pointTrans.y / pointTrans.z ;
						float u = alpha * xp + cx ;
						float v = beta * yp + cy ;

						/*if (u <= umin) umin = u ;
						  if (u >= umax) umax = u ;
						  if (v <= vmin) vmin = v ;
						  if (v >= vmax) vmax = v ;*/

						float zscan = getZAtPosition(cloudNormalsTrans, u, v) ;
						if (std::isnan(zscan) || zscan >= 0.0f) //in both cases we hit image plane
							surfels_projected_on_sensor++ ;
						if (!std::isnan(zscan) && zscan >= 0.0f) {
							//surfels_projected_on_sensor++ ;
							if (fabs(zscan - pointTrans.z) <= DMAX) { 
								//We have a surfel-scan match, we may update the surfel here... 

								pcl::PointXYZRGBNormal pointInterpolated, pointInterpolatedTrans ; 
								getPointAtPosition(cloudNormals, cloudNormalsTrans, u, v, pointInterpolated, pointInterpolatedTrans) ;
								//Computing running average
								PointCustomSurfel &pointSurfel = cloudScene->points[pointIndices[i]] ;

								pointSurfel.x = (pointSurfel.x * pointSurfel.count + pointInterpolated.x) / (pointSurfel.count + 1) ;
								pointSurfel.y = (pointSurfel.y * pointSurfel.count + pointInterpolated.y) / (pointSurfel.count + 1) ;
								pointSurfel.z = (pointSurfel.z * pointSurfel.count + pointInterpolated.z) / (pointSurfel.count + 1) ;

								pointSurfel.normal_x = (pointSurfel.normal_x * pointSurfel.count + pointInterpolated.normal_x) / (pointSurfel.count + 1) ;
								pointSurfel.normal_y = (pointSurfel.normal_y * pointSurfel.count + pointInterpolated.normal_y) / (pointSurfel.count + 1) ;
								pointSurfel.normal_z = (pointSurfel.normal_z * pointSurfel.count + pointInterpolated.normal_z) / (pointSurfel.count + 1) ;

								pointSurfel.r = (uint8_t) ((((uint32_t) pointSurfel.r) * pointSurfel.count + pointInterpolated.r) / (pointSurfel.count + 1)) ;
								pointSurfel.g = (uint8_t) ((((uint32_t) pointSurfel.g) * pointSurfel.count + pointInterpolated.g) / (pointSurfel.count + 1)) ;
								pointSurfel.b = (uint8_t) ((((uint32_t) pointSurfel.b) * pointSurfel.count + pointInterpolated.b) / (pointSurfel.count + 1)) ;

								pointSurfel.count++ ;
								pointSurfel.confidence++ ;

								float scanR = -pointInterpolatedTrans.z / pointInterpolatedTrans.normal_z * zTor  ;
								/*if (fabs(scanR) > 0.2) {
								  std::cout << "pointinterpolated.z " << pointInterpolated.z << std::endl ;
								  std::cout << "pointinterpolated.normal_z " << pointInterpolated.normal_z ;
								  }*/
								pointSurfel.radius = std::min<float>(pointSurfel.radius, scanR) ; //Update radius only when the new one is smaller
								/*if (pointSurfel.radius < 0) {
								  std::cout << pointSurfel.radius ;
								  }*/

								//We do not update colors now (in original solution (Weise) - they take color from the most perpendicular view)
								//TODO: possibly handle color update...

								markScanAsCovered(scan_covered, u, v) ; 
								nsurfels_updated++ ;
							} else if (zscan - pointTrans.z > DMAX) {
								//The observed point is behing the surfel, we may either remove the observation or the surfel (depending e.g. on the confidence)
								//markScanAsCovered(scan_covered, u, v) ; 
								PointCustomSurfel &pointSurfel = cloudScene->points[pointIndices[i]] ;
								if (pointSurfel.confidence < CONFIDENCE_THRESHOLD1) {
									//NaN surfel in the cloud (we do not remove it in order to maintain the structure of indices
									pointSurfel.x = pointSurfel.y = pointSurfel.z = std::numeric_limits<float>::quiet_NaN () ;
									//remove surfel from Octree
									pointIndices[i] = -1 ; //Mark as invalid (designed for future removal)
									nsurfels_removed++ ;
								} else {
									markScanAsCovered(scan_covered, u, v) ;
								}
								nscan_too_far++ ;
							} else
								nscan_too_close++ ;
						} else nsurfels_invalid_reading++ ;
					}
				}
				//The actual removal of marked (negative) indices
				vector<int>::iterator end_valid = remove_if(pointIndices.begin(), pointIndices.end(), IsNegative);
				pointIndices.erase(end_valid, pointIndices.end());
			}
			it++ ;
		}
	}

	std::cout << "Surfel update time (s): [" << timer.getTimeSeconds() << "]" << std::endl ;
	logger.log("surfel_update_time", timer.getTimeSeconds()) ;

	//std::cout << "(u,v)-bounds: [" << umin << "," << umax << "],[" << vmin << "," << vmax << "]" << std::endl ;

	unsigned int nscans_covered = 0 ;
	//Debug - counting positive elements in scan_covered
	for (int i = 0; i < cloud->height ; i++)
		for (int j = 0; j < cloud->width ; j++)
			if (scan_covered[i][j])
				nscans_covered++ ;
	//debug - end

	timer.reset() ;
	/*//Perform surfel-addition step
	//Create temporary point cloud (of surfels) to be concatenated with the scene cloud (TODO we may do without intermediary cloud, perhaps faster)	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::copyPointCloud(*cloud, *cloudTemp) ;
	*/

	Eigen::Matrix4d viewMatrixInv = viewMatrix.inverse() ;

	unsigned int surfels_added = 0 ;
	double distance  = 0.0 ;
	int distance_count = 0 ;
	//Update surfel data in the cloud to add and remove covered measurements
	for (uint32_t i = 0; i < cloud->height ; i++) 
		for (uint32_t j = 0; j < cloud->width ; j++) { 
			pcl::PointXYZRGBNormal pointNormalTrans = (*cloudNormalsTrans)(j, i) ;
			if (!scan_covered[i][j] && pcl::isFinite(pointNormalTrans)) { //We check cloudTrans - since it reflect point invalidations due to distance
				//Add a new point to the scene cloud (and the associated octree)
				pcl::PointXYZRGBNormal pointNormal = (*cloudNormals)(j, i) ;
				PointCustomSurfel pointSurfel ;
				pointSurfel.x = pointNormal.x ; pointSurfel.y = pointNormal.y; pointSurfel.z = pointNormal.z ;
				pointSurfel.normal_x = pointNormal.normal_x; pointSurfel.normal_y = pointNormal.normal_y ; pointSurfel.normal_z = pointNormal.normal_z ;
				pointSurfel.rgba = pointNormal.rgba ;
				pointSurfel.count = 1 ;
				pointSurfel.radius = -pointNormalTrans.z / pointNormalTrans.normal_z * zTor  ;
				pointSurfel.confidence = 1 ;

				octree.addPointToCloud(pointSurfel, cloudScene) ;
				surfels_added++ ;
				//Debug - add point using cloudTrans data
				
					/*float xp = (j - cx) / alpha ;
					float yp = (i - cy) / beta ;
					pcl::PointXYZRGB p = (*cloudTrans)(j, i) ;
					p.x = xp * p.z ; 
					p.y = yp * p.z ;
					pcl::PointXYZRGB p1 ;
					transformPointAffine(p, p1, viewMatrixInv) ;
					//octree.addPointToCloud(p1, cloudScene) ;
					distance += fabs((*cloud)(j, i).x - p1.x) + fabs((*cloud)(j, i).y - p1.y) + fabs((*cloud)(j, i).z - p1.z) ;
					distance_count++ ;*/
				
				//TODO Some other (more complex) processing is required here...
			}
		}

	//ROS_INFO("Average distance between corresponding points [%f]", distance / distance_count) ;

	std::cout << "Surfel addition time (s): [" << timer.getTimeSeconds() << "]" << std::endl ;
	logger.log("surfel_addition_time", timer.getTimeSeconds()) ;

	std::cout << "cloud_scene size (all surfels including removed): [" << cloudScene->width << "]" << std::endl ;
	logger.log("cloud_scene_width", cloudScene->width) ;
	int ncorrect_surfels = getPointCount() ;
	std::cout << "Actual scene size (without removed surfels) [" << ncorrect_surfels << "]" <<  std::endl ;
	logger.log("cloud_scene_actual_size", ncorrect_surfels) ;
	std::cout << "Correct scans [" << ncorrect_scans << "]" << std::endl ;
	std::cout << "Correct scans and normals [" << ncorrect_scans_and_normals << "]" << std::endl ;
	ntotal_scans = nscans_covered + surfels_added ;
	std::cout << "Correct (add-able) scans (inside bounds and frontal-oriented) [" << ntotal_scans << "]"  << std::endl ; 
	logger.log("ntotal_scans", ntotal_scans) ;
	std::cout << "No. of scans covered [" << nscans_covered << "]" << std::endl ;
	logger.log("nscans_covered", nscans_covered) ;
	std::cout << "Surfels inside octree frustum [" << surfels_inside_octree_frustum << "]" << std::endl ;
	logger.log("nsurfels_inside_frustum", surfels_inside_octree_frustum) ;
	std::cout << "Surfels projected on sensor plane [" << surfels_projected_on_sensor << "]" << std::endl ;
	logger.log("nsurfels_projected_on_sensor", surfels_projected_on_sensor) ;
	std::cout << "Projected/inside frustum (%) [" << double(surfels_projected_on_sensor)/surfels_inside_octree_frustum * 100 << "]" << std::endl ;
	std::cout << "Outside frustum/total points (%) [" << double(ncorrect_surfels - surfels_inside_octree_frustum) / ncorrect_surfels * 100 << "]" << std::endl ;
	std::cout << "Octree nodes visited during update [" << octree_nodes_visited << "]" << std::endl ;
	logger.log("octree_nodes_visited", octree_nodes_visited) ;
	std::cout << "Surfels updated [" << nsurfels_updated << "]" << std::endl ;
	logger.log("surfels_updated", nsurfels_updated) ;
	std::cout << "Scans too far for surfel update [" << nscan_too_far << "]" << std::endl ;
	logger.log("scans_too_far", nscan_too_far) ;
	std::cout << "Scans too close for surfel update [" << nscan_too_close << "]" << std::endl ;
	logger.log("scans_too_close", nscan_too_close) ;
	std::cout << "Surfels without matching reading (NaN, outside frame) [" << nsurfels_invalid_reading << "]" << std::endl ;
	std::cout << "Surfels removed during update [" << nsurfels_removed << "]" << std::endl ;
	logger.log("surfels_removed_on_update", nsurfels_removed) ;
	std::cout << "Surfels added [" << surfels_added << "]" << std::endl ;
	logger.log("surfels_added", surfels_added) ;

	//Now downsample scene cloud
	timer.reset() ;	
	downsampleSceneCloud() ;
	std::cout << "Cloud downsampling time(s): [" << timer.getTimeSeconds() << "]" << std::endl ;

	logger.nextRow() ;
}

pcl::PointCloud<PointCustomSurfel>::Ptr &SurfelMapper::getCloudScene()
{
	return cloudScene ;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr &SurfelMapper::getCloudSceneDownsampled()
{
	return cloudSceneDownsampled ;
}

size_t SurfelMapper::getPointCount()
{
	//Convert voxels at fixed depth to points in a downsampled cloud
	pcl::octree::OctreePointCloud<PointCustomSurfel>::LeafNodeIterator it = octree.leaf_begin() ;
	const pcl::octree::OctreePointCloud<PointCustomSurfel>::DepthFirstIterator it_end = octree.leaf_end();
	size_t count = 0 ;
	while(it != it_end) {
		if (it.isLeafNode()) {
			//Examine points in the voxel	
			pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
			count += container.getSize() ;
		} else {
			std::cerr << "LeafNode iterator error - we are not in the leaf node!" << std::endl ;
		}
		it++ ;
	}
	return count ;
}


void SurfelMapper::resetMap()
{
	cloudScene = pcl::PointCloud<PointCustomSurfel>::Ptr(new pcl::PointCloud<PointCustomSurfel>) ;
	cloudSceneDownsampled = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	octree.deleteTree() ;
	octree.setInputCloud(cloudScene) ;
	
}

void SurfelMapper::getBoundingBoxIndices(const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt, std::vector<int> &k_indices)
{
	octree.boxSearch(min_pt, max_pt, k_indices) ;
}
