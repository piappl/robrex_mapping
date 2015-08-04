/**
 *  @file surfel_mapper_test.cpp
 *  @author Artur Wilkowski <ArturWilkowski@piap.pl>
 * 
 *  @section LICENSE
 *
 *  Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
 *  Security and Defence Systems Division <http://www.piap.pl>
 */

#define BOOST_TEST_MODULE SurfelMapperTest 
#include <boost/test/unit_test.hpp>
#include "surfel_mapper.hpp"
#include <pcl/common/transforms.h>


////////////////////////////////////////////////////////////////////////
/* seven ways to detect and report the same error:
int add( int i, int j ) { return i+j; }

BOOST_CHECK( add( 2,2 ) == 4 );        // #1 continues on error

BOOST_REQUIRE( add( 2,2 ) == 4 );      // #2 throws on error

if( add( 2,2 ) != 4 )
  BOOST_ERROR( "Ouch..." );            // #3 continues on error

if( add( 2,2 ) != 4 )
  BOOST_FAIL( "Ouch..." );             // #4 throws on error

if( add( 2,2 ) != 4 ) throw "Ouch..."; // #5 throws on error

BOOST_CHECK_MESSAGE( add( 2,2 ) == 4,  // #6 continues on error
                     "add(..) result: " << add( 2,2 ) );

BOOST_CHECK_EQUAL( add( 2,2 ), 4 );      // #7 continues on error*/
////////////////////////////////////////////////////////////////////////

CameraParams camera_params = { //Default camera parameters
	481.2, //alpha
	480.0, //beta
	319.5, //cx
	239.5  //cy
}; //Fixed camera params 

void constructPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
	// Create a simple input cloud (flat surface) 
	pcl::PointXYZRGB p ;
	p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN() ;
	p.rgba = 0u ;

	//Sample Kincect calibration data	
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(640,480, p)) ;

	double alpha = camera_params.alpha ;
	double beta = camera_params.beta ;
	double cx = camera_params.cx ;
	double cy = camera_params.cy ;  
	//double alpha = 518.930578 ; //fx
	//double cx = 323.483756 ;
	//double beta = 517.211658 ; //fy
	//double cy = 260.384697 ;

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

BOOST_AUTO_TEST_CASE(TestAddPointCloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
	constructPointCloud(cloud) ;

	cloud->sensor_origin_ << 0, 0, 0, 1 ;
	cloud->sensor_orientation_ = Eigen::Quaternionf(1,0,0,0) ;
	
	boost::shared_ptr<SurfelMapper> mapper(new SurfelMapper(3e7, false, camera_params))  ;
	mapper->addPointCloudToScene(cloud) ;

	size_t pcount = mapper->getPointCount() ;
	//std::cout << "Point count: " << pcount << std::endl ;

    	BOOST_CHECK(pcount > 8500 && pcount < 9000) ;
}

BOOST_AUTO_TEST_CASE(TestAddSingleViewpoint) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
	constructPointCloud(cloud) ;

	cloud->sensor_origin_ << 0, 0, 0, 1 ;
	cloud->sensor_orientation_ = Eigen::Quaternionf(1,0,0,0) ;

	boost::shared_ptr<SurfelMapper> mapper(new SurfelMapper(3e7, false, camera_params))  ;
	mapper->addPointCloudToScene(cloud) ;
	size_t startcount = mapper->getPointCount() ;
	mapper->addPointCloudToScene(cloud) ;
	mapper->addPointCloudToScene(cloud) ;
	size_t endcount = mapper->getPointCount() ;

	//std::cout << "Start point count: " << startcount << ". End point count: " << endcount << std::endl ;
	
    	BOOST_CHECK(startcount == endcount) ;
}

BOOST_AUTO_TEST_CASE(testAddMultipleViewpoints) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTrans ;

	constructPointCloud(cloud) ;

	cloud->sensor_origin_ << 0, 0, 0, 1 ;
	cloud->sensor_orientation_ = Eigen::Quaternionf(1,0,0,0) ; //Euler 0 0 0

	boost::shared_ptr<SurfelMapper> mapper(new SurfelMapper(3e7, false, camera_params))  ;
	mapper->addPointCloudToScene(cloud) ;
	size_t startcount = mapper->getPointCount() ;

	cloud->sensor_orientation_ = Eigen::Quaternionf(0.70710678118654760,0,0.7071067811865476,0) ; //Euler -90 0 0
	transformCloud(cloud, cloudTrans) ; //Cloud is expected to be transformed according to the sensor orientation
	mapper->addPointCloudToScene(cloudTrans) ;

	cloud->sensor_orientation_ = Eigen::Quaternionf(0.70710678118654760,0,-0.7071067811865476,0) ; //Euler 90 0 0
	transformCloud(cloud, cloudTrans) ; //Cloud is expected to be transformed according to the sensor orientation
	mapper->addPointCloudToScene(cloudTrans) ;

	size_t endcount = mapper->getPointCount() ;

	//std::cout << "Start point count: " << startcount << ". End point count: " << endcount << std::endl ;

    	BOOST_CHECK(startcount * 3 == endcount) ;
}

/*int main() {
	testAddPointCloud() ;
	testAddSingleViewpoint() ;
	testAddMultipleViewpoints() ;

	return 0 ;
}*/

