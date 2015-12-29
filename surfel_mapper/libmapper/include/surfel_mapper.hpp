/**
 *  @file surfel_mapper.hpp
 *  @author Artur Wilkowski <ArturWilkowski@piap.pl>
 * 
 *  @section LICENSE
 *
 *  Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
 *  Security and Defence Systems Division <http://www.piap.pl>
 */

#ifndef SURFEL_MAPPER_HPP
#define SURFEL_MAPPER_HPP

#include "point_custom_surfel.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/octree/octree.h>
#include "logger.hpp"

#define CLOUD_WIDTH 640 /**< Default cloud width */
#define CLOUD_HEIGHT 480 /**< Default cloud height */

/**
 * @brief Camera intrinsic parameters
 */
typedef struct {
	double alpha ; /**< @brief x-focal length (fx) */
	double beta ; /**< @brief y-focal length (fy) */
	double cx ; /**< @brief x coordinate of the camera optical center*/
	double cy ; /**< @brief y coordinate of the camera optical center*/
} CameraParams ;

/**
* @brief This is the main class rempresenting surfel map  
*
* This class enables to create surfel maps basing on the data from RGBD sensor. Input is formed by RGBD frames
* with sensor orientation specified. The frames are sequentially integrated into the surfel map. The output
* is either the full surfel cloud or downsampled preview cloud. The class use octree coupled with frustum
* for efficient map update.
*/
class SurfelMapper {
	protected:
		//Logger
		Logger logger /**< @brief logger object*/ ;

		//Parameters
		double DMAX  = 0.005f ; /**< @brief distance threshold for surfel update*/ 
		double MIN_KINECT_DIST  = 0.8 ; /**< @brief reliable minimum sensor reading distance*/
		double MAX_KINECT_DIST = 4.0 ; /**< @brief reliable maximum sensor reading distance*/
		double OCTREE_RESOLUTION = 0.2 ; /**< @brief resolution of underlying octree*/
		double PREVIEW_RESOLUTION = 0.2 ; /**< @brief resolution of output preview map*/
		int PREVIEW_COLOR_SAMPLES_IN_VOXEL = 3 ; /**< @brief number of samples in voxel used for constructing preview point (affects preview efficiency)*/
		int CONFIDENCE_THRESHOLD1 = 5 ; /**< @brief confidence threshold used for establishing reliable surfels*/
		double MIN_SCAN_ZNORMAL = 0.2f ; /**< @brief acceptable minimum z-component of scan normal*/
		bool USE_FRUSTUM = true ; /**< @brief use frustum or no*/
		int SCENE_SIZE = 3e7 ; /**< @brief preallocated size of scene*/
		bool LOGGING = true ; /**< @brief logging turned on or off*/
		bool USE_UPDATE = true ; /**< @brief use surfel update or no*/
		/**
		 * Default camera parameters
		 */
		CameraParams camera_params = { 
			481.2, /**< alpha*/
			480.0, /**< beta*/
			319.5, /**< cx*/
			239.5  /**< cy*/
		};

		pcl::PointCloud<PointCustomSurfel>::Ptr cloudScene ; /**< @brief The main scene cloud */ 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSceneDownsampled ; /**< @brief Downsampled scene cloud */

		pcl::octree::OctreePointCloudSearch<PointCustomSurfel> octree ; /**< @brief Octree organizing surfels in the cloud */

		/**
		 * @brief Performs affine transformation on the input point 
		 *
		 * @param point_in input point
		 * @param point_out output point
		 * @param transform transformation matrix
		 */
		static inline void transformPointAffine(PointCustomSurfel &point_in, PointCustomSurfel &point_out, Eigen::Matrix4d transform) ;

		/**@brief A modified PCL transformPointCloud function aimed at non-rigid homogenous transformations
		 *
		 * @param cloud_in input cloud
		 * @param cloud_out output cloud
		 * @param transform homogenous transformation
		 */
		template <typename PointT, typename Scalar> static void transformPointCloudNonRigid (const pcl::PointCloud<PointT> &cloud_in, 
				pcl::PointCloud<PointT> &cloud_out, const Eigen::Matrix<Scalar, 4, 4> &transform) ;

		/**
		 * @brief Gets interpolated -z- value at the specified (not necesserily integer) position in an organized cloud
		 *
		 * @param cloud input cloud
		 * @param u - image x-coordinate
		 * @param v - image y-coordinate
		 * @return interpolated z-value; negative z - denotes sampling out of depth image bounds, NaN - denotes invalid reading at given position of the organized cloud
		 */
		static float getZAtPosition(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, float u, float v) ;
		
		/**
		 * @brief Gets interpolated point at the specified (not necesserily integer) position in an organized cloud
		 *
		 * @param cloud first input cloud
		 * @param cloud_trans second input cloud
		 * @param u - image x-coordinate
		 * @param v - image y-coordinate
		 * @param point point extracted from the first input cloud
		 * @param point_trans point extracted froom the second input cloud
		 */
		static void getPointAtPosition(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_trans, float u, float v, pcl::PointXYZRGBNormal &point, pcl::PointXYZRGBNormal &point_trans) ;

		/**
		 * @brief Marks position in a scan-array as used
		 *
		 * @param scan_covered scan-array
		 * @param u - image x-coordinate
		 * @param v - image y-coordinate
		 */
		static void markScanAsCovered(char scan_covered[CLOUD_HEIGHT][CLOUD_WIDTH], float u, float v) ;

		/**
		 * @brief Skip all child voxels of the octree node
		 *
		 * This is a corrected version of skipChildVoxels from the pcl::OctreeDepthFirstIterator. The latter actually skips all siblings and children, this version skips only children. 
		 *
		 * @param it iterator pointing at the octree node
		 * @param it_end end iterator of the octree
		 */
		static void skipChildVoxelsCorrect(pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it_end) ;

		/**
		 * @brief Compute an average color for the voxel
		 *
		 * @param it iterator pointing at the octree node associated with the voxel
		 * @param it_end end iterator of the octree
		 * @param point this routine fill the color of this point
		 */
		void computeVoxelColor(pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it, const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::DepthFirstIterator &it_end, pcl::PointXYZRGB &point) ;

		/**
		 * @brief Filters cloud point by a distance from the sensor 
		 *
		 * @param cloud input/output cloud 
		 */
		void filterCloudByDistance(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud) ;

		/**
		 * @brief Computes downsampled version of the cloud 
		 */
		void downsampleSceneCloud() ;

		/**
		 * @brief Prints surfel mapper settings 
		 */
		void printSettings() ;

		/**
		 * @brief Initializes logging facility
		 */
		void initLogger() ;

	public:
		/**
		 * @brief A parametric constructor
		 *
		 * Constructs the SurfelMapper object
		 *
		 * @param DMAX distance threshold for surfel update
		 * @param MIN_KINECT_DIST reliable minimum sensor reading distance
		 * @param MAX_KINECT_DIST reliable maximum sensor reading distance
		 * @param OCTREE_RESOLUTION resolution of underlying octree
		 * @param PREVIEW_RESOLUTION resolution of output preview map
		 * @param PREVIEW_COLOR_SAMPLES_IN_VOXEL number of samples in voxel used for constructing preview point (affects preview efficiency)
		 * @param CONFIDENCE_THRESHOLD1 confidence threshold used for establishing reliable surfels
		 * @param MIN_SCAN_ZNORMAL acceptable minimum z-component of scan normal
		 * @param USE_FRUSTUM use frustum or no
		 * @param SCENE_SIZE preallocated size of scene
		 * @param LOGGING logging turned on or off
		 * @param USE_UPDATE use surfel update or no
		 * @param camera_params use this specific set of camera parameters for projection
		 */
		SurfelMapper(double DMAX, double MIN_KINECT_DIST, double MAX_KINECT_DIST, double OCTREE_RESOLUTION, 
		  	     double PREVIEW_RESOLUTION, int PREVIEW_COLOR_SAMPLES_IN_VOXEL, int CONFIDENCE_THRESHOLD1, double MIN_SCAN_ZNORMAL, 
			     bool USE_FRUSTUM, int SCENE_SIZE, bool LOGGING, bool USE_UPDATE, CameraParams &camera_params) ;
	
		/**
		 * @brief A parametric constructor
		 *
		 * Constructs the SurfelMapper object
		 *
		 * @param SCENE_SIZE preallocated size of scene
		 * @param LOGGING logging turned on or off
		 * @param camera_params use this specific set of camera parameters for projection
		 */
		SurfelMapper(int SCENE_SIZE, bool LOGGING, CameraParams &camera_params) ; 

		/**
		 * @brief A non-parametric constructor
		 *
		 * Constructs the SurfelMapper object with default parameters
		 */
		SurfelMapper() ;
		
		/**
		 * @brief A destructor 
		 *
		 * Finishes the life of the SurfelMapper object 
		 */
		~SurfelMapper() ;

		/**
		 * @brief Add new point cloud to scene 
		 *
		 * Add new point cloud to scene. Input cloud is expected to provide sensor orientation and be transformed to the world frame according to the orientation
		 *
		 * @param cloud input RGBD cloud 
		 */
		void addPointCloudToScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) ;

		/**
		 * @brief Retrieves scene cloud 
		 *
		 * Retrieves scene cloud
		 *
		 * @return the current surfel point cloud 
		 */
		pcl::PointCloud<PointCustomSurfel>::Ptr &getCloudScene() ;

		/**
		 * @brief Retrieves downsample scene cloud 
		 *
		 * Retrieves downsampled scene cloud
		 *
		 * @return the scene cloud downsampled according to the parameters specified 
		 */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getCloudSceneDownsampled() ;

		/**
		 * @brief Retrieves current number of surfels in the scene cloud 
		 *
		 * Retrieves current number of surfels in the scene cloud 
		 *
		 * @return number of points in the scene cloud 
		 */
		size_t getPointCount() ;

		/**
		 * @brief Resets map
		 *
		 * The scene is reset to the blank state (integration of incoming readings is started anew) 
		 */
		void resetMap() ;

		/**
		 * @brief Gets indices for the points from the bounding box 
		 *
		 * Gets indices for the points from the bounding box. The indices refer to the cloud that can be retrieved (at the same time) using
		 * SurfelMapper::getCloudScene()
		 *
		 * @param min_pt minimum corner of the bounding box
		 * @param max_pt maximum corner of the bounding box
		 * @param k_indices selected indices are stored in this argument
		 */
		void getBoundingBoxIndices(const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt, std::vector<int> &k_indices) ;

		/**
		 * @brief Gets indices for all points in the map 
		 *
		 * Gets indices for all points in the map. The indices refer to the cloud that can be retrieved (at the same time) using
		 * SurfelMapper::getCloudScene()
		 *
		 * @param k_indices selected indices are stored in this argument
		 */
		void getAllIndices(std::vector<int> &k_indices) ;
} ;

#endif
