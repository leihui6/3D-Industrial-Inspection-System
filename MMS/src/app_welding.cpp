#include "app_welding.h"



app_welding::app_welding()
{
}


app_welding::~app_welding()
{
}

void app_welding::process(std::vector<measurement_content>& measurement_result_vec, std::vector<point_3d> & marked_begin_points)
{
	if (!deter_begin_point(measurement_result_vec, marked_begin_points))
	{
		// make it order, only check the begining and end point
		make_order(measurement_result_vec);

		// cut off line that is too long
		cut_off(measurement_result_vec);
	}
}


int app_welding::deter_begin_point(std::vector<measurement_content>& measurement_result_vec, std::vector<point_3d>& marked_begin_points)
{
	if (measurement_result_vec.empty()) return 1;

	point_3d cent_point;
	centroid_from_points(marked_begin_points, cent_point);

	point_3d
		&point_1 = measurement_result_vec[0].drawable_points.front(),
		&point_2 = measurement_result_vec[0].drawable_points.back();

	float distance[2] = { 0.0,0.0 };

	distance_point_to_point(cent_point, point_1, distance[0]);
	distance_point_to_point(cent_point, point_2, distance[1]);

	if (distance[0] > distance[2])
	{
		std::reverse(measurement_result_vec.at(0).drawable_points.begin(),
			measurement_result_vec.at(0).drawable_points.end());
	}
	return 0;
}


void app_welding::make_order(std::vector<measurement_content>& measurement_result_vec)
{
	for (size_t i = 0; i < measurement_result_vec.size(); ++i)
	{
		size_t next_i = (i + 1) % measurement_result_vec.size();

		point_3d
			&this_end_p = measurement_result_vec[i].drawable_points.back(),
			&next_begin_p = measurement_result_vec[next_i].drawable_points.front(),
			&next_end_p = measurement_result_vec[next_i].drawable_points.back();

		float dis[2] = { 0,0 };
		distance_point_to_point(this_end_p, next_begin_p, dis[0]);
		distance_point_to_point(this_end_p, next_end_p, dis[1]);
	
		if (dis[0] > dis[1])
		{
			std::reverse(measurement_result_vec[next_i].drawable_points.begin(),
				measurement_result_vec[next_i].drawable_points.end());
		}
	}
}

void app_welding::cut_off(std::vector<measurement_content>& measurement_result_vec)
{
	if (measurement_result_vec.size() < 3) return;

	size_t this_cut_index = 0, next_cut_index = 0;

	float min_dis = FLT_MAX;

	for (size_t i = 0; i < measurement_result_vec.size(); ++i)
	{
		size_t next_i = i + 1;

		std::vector<point_3d> &this_vec =
			measurement_result_vec[i].drawable_points;

		// the last one
		if (next_i == measurement_result_vec.size())
		{
			float min_ = FLT_MAX;
			size_t this_cut_index_ = 0;//, next_cut_index_ = 0;

			for (size_t j = 0; j < measurement_result_vec.size() - 2; ++j)
			{
				min_vecs(this_vec, measurement_result_vec[j].drawable_points,
					min_dis, this_cut_index, next_cut_index);

				if (min_dis < min_)
				{
					min_ = min_dis;
					this_cut_index_ = this_cut_index;
					//next_cut_index_ = next_cut_index;
				}
			} // end for

			this_vec.resize(this_cut_index);
			break;
		}

		std::vector<point_3d> &next_vec =
			measurement_result_vec[next_i].drawable_points;

		min_vecs(this_vec, next_vec, min_dis, this_cut_index, next_cut_index);

		// it can only be implemented by ordered elements!!!
		this_vec.resize(this_cut_index);
		next_vec.erase(next_vec.begin(), next_vec.begin() + next_cut_index);
	}
}

void app_welding::min_vecs(std::vector<point_3d>& vec_1, std::vector<point_3d>& vec_2, float & min_distance, size_t & vec_1_index, size_t & vec_2_index)
{
	min_distance = FLT_MAX;

	for (size_t j = 0; j < vec_1.size(); ++j)
	{
		for (size_t k = 0; k < vec_2.size(); ++k)
		{
			float dis = 0.0;
			distance_point_to_point(vec_1[j], vec_2[k], dis);

			if (dis < min_distance)
			{
				min_distance = dis;
				vec_1_index = j;
				vec_2_index = k;
			}
		}
	}
}
