/**
 *  @file point_custom_surfel.hpp
 *  @author Artur Wilkowski <ArturWilkowski@piap.pl>
 * 
 *  @section LICENSE
 *
 *  Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
 *  Security and Defence Systems Division <http://www.piap.pl>
 */

#ifndef POINT_CUSTOM_SURFEL_HPP
#define POINT_CUSTOM_SURFEL_HPP

#include <pcl/point_types.h>

/**
 * @brief Structure representing properties of a surfel
 */
struct PointCustomSurfel {
	PCL_ADD_POINT4D; /**< Adds 4d point fields */
	PCL_ADD_NORMAL4D; /**< Adds 4d normal fields */
	union
	{
		struct
		{
			// RGB union
			union
			{
				struct
				{
					uint8_t b; /**< blue color component */
					uint8_t g; /**< green color component */
					uint8_t r; /**< red color component */
					uint8_t a; /**< alpha color component */
				};
				float rgb; /**< rgb color component */
				uint32_t rgba; /**< rgba color component */
			};
			float radius; /**< surfel radius */
			uint32_t confidence; /**< surfel confidence */
			uint32_t count; /**< surfel observation count */
		};
		float data_c[4]; /**< data accessor */
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointCustomSurfel,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, normal_x, normal_x)
  (float, normal_y, normal_y)
  (float, normal_z, normal_z)
  (uint32_t, rgba, rgba)
  (float, radius, radius)
  (uint32_t, confidence, confidence)
  (uint32_t, count, count)
 )
				

#endif
