#ifndef SURFEL_MAPPER_HPP
#define SURFEL_MAPPER_HPP

#include "point_custom_surfel.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/octree/octree.h>

#define CLOUD_WIDTH 640
#define CLOUD_HEIGHT 480

class SurfelMapper {
	protected:
		//Parameters
		double DMAX  = 0.005f ;
		double MIN_KINECT_DIST  = 0.8 ;
		double MAX_KINECT_DIST = 4.0 ;
		double OCTREE_RESOLUTION = 0.2 ;
		double PREVIEW_RESOLUTION = 0.2 ;
		int PREVIEW_COLOR_SAMPLES_IN_VOXEL = 3 ;
		int CONFIDENCE_THRESHOLD1 = 5 ;
		double MIN_SCAN_ZNORMAL = 0.2f ;
		bool USE_FRUSTUM = true ;
		int SCENE_SIZE = 3e7 ;

		//Let us define our main scene cloud (will contain surfels soon...)
		pcl::PointCloud<PointCustomSurfel>::Ptr cloudScene ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneDownsampled ;

		pcl::octree::OctreePointCloudSearch<PointCustomSurfel> octree ;

		static inline void transformPointAffine(PointCustomSurfel &point_in, PointCustomSurfel &point_out, Eigen::Matrix4d transform) ;

		//A modified PCL transformPointCloud function aimed at non-rigid homogenous transformations
		template <typename PointT, typename Scalar> static void transformPointCloudNonRigid (const pcl::PointCloud<PointT> &cloud_in, 
				pcl::PointCloud<PointT> &cloud_out, const Eigen::Matrix<Scalar, 4, 4> &transform) ;

		/**
		 * Gets interpolated -z- value at specified (not necesserily integer) position in organized cloud
		 * negative z - denotes sampling out of depth image bounds, nan - denotes invalid reading at given position of the organized cloud
		 */
		static float getZAtPosition(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, float u, float v) ;
		
		/**
		 * Gets interpolated point at specified (not necesserily integer) position in organized cloud
		 */
		static void getPointAtPosition(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_trans, float u, float v, pcl::PointXYZRGBNormal &point, pcl::PointXYZRGBNormal &point_trans) ;

		static void markScanAsCovered(char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH], float u, float v) ;

		//skipChildVoxels in OctreeDepthFirstIterator actually skips all siblings, we want to skip actual children of the node
		static void skipChildVoxelsCorrect(pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it_end) ;

		void computeVoxelColor(pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it_end, pcl::PointXYZRGB &point) ;

		void filterCloudByDistance(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud) ;

		void downsampleSceneCloud() ;

		void printSettings() ;


	public:
		SurfelMapper(double DMAX, double MIN_KINECT_DIST, double MAX_KINECT_DIST, double OCTREE_RESOLUTION, 
		  	     double PREVIEW_RESOLUTION, int PREVIEW_COLOR_SAMPLES_IN_VOXEL, int CONFIDENCE_THRESHOLD1, double MIN_SCAN_ZNORMAL, 
			     bool USE_FRUSTUM, int SCENE_SIZE) ;
		SurfelMapper() ;
		~SurfelMapper() ;
		void addPointCloudToScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) ;

		pcl::PointCloud<PointCustomSurfel>::Ptr &getCloudScene() ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getCloudSceneDownsampled() ;

		size_t getPointCount() ;

		void resetMap() ;

		void getBoundingBoxIndices(const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt, std::vector<int> &k_indices) ;
} ;

#endif
