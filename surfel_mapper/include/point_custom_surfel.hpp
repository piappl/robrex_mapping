#ifndef POINT_CUSTOM_SURFEL_HPP
#define POINT_CUSTOM_SURFEL_HPP

#include <pcl/point_types.h>

struct PointCustomSurfel {
	PCL_ADD_POINT4D; 
	PCL_ADD_NORMAL4D; 
	union
	{
		struct
		{
			// RGB union
			union
			{
				struct
				{
					uint8_t b;
					uint8_t g;
					uint8_t r;
					uint8_t a;
				};
				float rgb;
				uint32_t rgba;
			};
			float radius;
			uint32_t confidence;
			uint32_t count;
		};
		float data_c[4];
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
