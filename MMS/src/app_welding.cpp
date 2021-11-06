#include "app_welding.h"



app_welding::app_welding()
{
}


app_welding::~app_welding()
{
}

void app_welding::process(std::vector<measurement_content>& measurement_result_vec)
{
	// make it order, only check the begining and end point
	make_order(measurement_result_vec);
	
	// cut off line that is too long
	cut_off(measurement_result_vec);
}

void app_welding::make_order(std::vector<measurement_content>& measurement_result_vec)
{
	for (size_t i = 0; i < measurement_result_vec.size(); ++i)
	{
		size_t next_i = (i + 1) % measurement_result_vec.size();

		point_3d
			&this_end_p = measurement_result_vec[i].drawable_points.back(),
			&next_begin_p = measurement_result_vec[next_i].drawable_points.back(),
			&next_end_p = measurement_result_vec[next_i].drawable_points.back();

		float dis[2] = { 0,0 };
		distance_point_to_point(this_end_p, next_begin_p, dis[0]);
		distance_point_to_point(this_end_p, next_begin_p, dis[1]);
	
		if (dis[0] > dis[1])
		{
			std::reverse(measurement_result_vec[next_i].drawable_points.begin(),
				measurement_result_vec[next_i].drawable_points.end());
		}
	}
}

void app_welding::cut_off(std::vector<measurement_content>& measurement_result_vec)
{
	// TODO
}
