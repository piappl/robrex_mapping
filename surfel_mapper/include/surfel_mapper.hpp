#include <pcl/common/common_headers.h>
#include <pcl/octree/octree.h>

#define CLOUD_WIDTH 640
#define CLOUD_HEIGHT 480


class SurfelMapper {
	protected:

		//Let us define our main scene cloud (will contain surfels soon...)
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudScene ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneDownsampled ;

		pcl::octree::OctreePointCloudPointVector<pcl::PointXYZRGB> octree ;

		static inline void transformPointAffine(pcl::PointXYZRGB &point_in, pcl::PointXYZRGB &point_out, Eigen::Matrix4d transform) ;

		//A modified PCL transformPointCloud function aimed at non-rigid homogenous transformations
		template <typename PointT, typename Scalar> static void transformPointCloudNonRigid (const pcl::PointCloud<PointT> &cloud_in, 
				pcl::PointCloud<PointT> &cloud_out, const Eigen::Matrix<Scalar, 4, 4> &transform) ;

		/**
		 * Gets interpolated -z- value at specified (not necesserily integer) position in organized cloud
		 * negative z - denotes sampling out of depth image bounds, nan - denotes invalid reading at given position of the organized cloud
		 */
		static float getZAtPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float u, float v) ;

		static void markScanAsCovered(char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH], float u, float v) ;

		//skipChildVoxels in OctreeDepthFirstIterator actually skips all siblings, we want to skip actual children of the node
		static void skipChildVoxelsCorrect(pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it_end) ;

		void computeVoxelColor(pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it_end, pcl::PointXYZRGB &point) ;

		static void filterCloudByDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) ;

		void downsampleSceneCloud() ;

	public:
		SurfelMapper() ;
		~SurfelMapper() ;
		void addPointCloudToScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) ;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getCloudScene() ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getCloudSceneDownsampled() ;
} ;
