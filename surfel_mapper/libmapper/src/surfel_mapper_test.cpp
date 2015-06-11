#include "surfel_mapper.hpp"
#include <pcl/common/transforms.h>

//TODO: Convert to BOOST TESTS

void constructPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
	// Create a simple input cloud (flat surface) 
	pcl::PointXYZRGB p ;
	p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN() ;
	p.rgba = 0u ;

	//Sample Kincect calibration data	
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(640,480, p)) ;
	double alpha = 518.930578 ; //fx
	double cx = 323.483756 ;
	double beta = 517.211658 ; //fy
	double cy = 260.384697 ;

	for (uint32_t i = 50; i < 150 ; i++) 
		for (uint32_t j = 50 ; j < 150 ; j++) {
			pcl::PointXYZRGB &point = (*cloud)(j, i) ;	

			float xp = (j - cx) / alpha ;
			float yp = (i - cy) / beta ;
			float zp = 2.0f ;

			point.x = xp * zp ;
			point.y = yp * zp ;
			point.z = zp ;

			point.r = point.g = point.b = 100 ;
		}
}

void transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudTrans) {
	cloudTrans.reset(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	Eigen::Matrix4d viewMatrix ;
	viewMatrix << cloud->sensor_orientation_.toRotationMatrix().cast<double>(), cloud->sensor_origin_.topRows<3>().cast<double>(), 0.0, 0.0, 0.0, 1.0 ;

	pcl::transformPointCloud(*cloud, *cloudTrans, viewMatrix) ;
}


void testAddPointCloud() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
	constructPointCloud(cloud) ;

	cloud->sensor_origin_ << 0, 0, 0, 1 ;
	cloud->sensor_orientation_ = Eigen::Quaternionf(1,0,0,0) ;

	boost::shared_ptr<SurfelMapper> mapper(new SurfelMapper())  ;
	mapper->addPointCloudToScene(cloud) ;

	size_t pcount = mapper->getPointCount() ;
	std::cout << "Point count: " << pcount << std::endl ;
	if (pcount > 8500 && pcount < 9000)
		std::cout << "test addPointCloud OK" << std::endl ;
	else
		std::cout << "testAddPointCloud FAILED (number of points outside limit 8500-9000)" << std::endl ;
}

void testAddSingleViewpoint() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
	constructPointCloud(cloud) ;

	cloud->sensor_origin_ << 0, 0, 0, 1 ;
	cloud->sensor_orientation_ = Eigen::Quaternionf(1,0,0,0) ;

	boost::shared_ptr<SurfelMapper> mapper(new SurfelMapper())  ;
	mapper->addPointCloudToScene(cloud) ;
	size_t startcount = mapper->getPointCount() ;
	mapper->addPointCloudToScene(cloud) ;
	mapper->addPointCloudToScene(cloud) ;
	size_t endcount = mapper->getPointCount() ;

	std::cout << "Start point count: " << startcount << ". End point count: " << endcount << std::endl ;
	if (startcount == endcount)
		std::cout << "testAddSingleViewpoint OK" << std::endl ;
	else
		std::cout << "testAddSingleViewpoint FAILED (number of surfels at the end should be the same as at the beginning)" << std::endl ;
}

void testAddMultipleViewpoints() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTrans ;

	constructPointCloud(cloud) ;

	cloud->sensor_origin_ << 0, 0, 0, 1 ;
	cloud->sensor_orientation_ = Eigen::Quaternionf(1,0,0,0) ; //Euler 0 0 0

	boost::shared_ptr<SurfelMapper> mapper(new SurfelMapper())  ;
	mapper->addPointCloudToScene(cloud) ;
	size_t startcount = mapper->getPointCount() ;

	cloud->sensor_orientation_ = Eigen::Quaternionf(0.70710678118654760,0,0.7071067811865476,0) ; //Euler -90 0 0
	transformCloud(cloud, cloudTrans) ; //Cloud is expected to be transformed according to the sensor orientation
	mapper->addPointCloudToScene(cloudTrans) ;

	cloud->sensor_orientation_ = Eigen::Quaternionf(0.70710678118654760,0,-0.7071067811865476,0) ; //Euler 90 0 0
	transformCloud(cloud, cloudTrans) ; //Cloud is expected to be transformed according to the sensor orientation
	mapper->addPointCloudToScene(cloudTrans) ;

	size_t endcount = mapper->getPointCount() ;

	std::cout << "Start point count: " << startcount << ". End point count: " << endcount << std::endl ;
	if (startcount * 3 == endcount)
		std::cout << "testAddMultiplelViewpoints OK" << std::endl ;
	else
		std::cout << "testAddMultipleViewpoints FAILED (number of surfels at the end should 3 times the number at the beginning)" << std::endl ;
}

int main() {
	testAddPointCloud() ;
	testAddSingleViewpoint() ;
	testAddMultipleViewpoints() ;

	return 0 ;
}

