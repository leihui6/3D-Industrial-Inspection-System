#ifndef APP_WELDING
#define APP_WELDING

#include "cloud_geometry.h"

class app_welding
{
public:
	app_welding();

	~app_welding();

	void process(std::vector<measurement_content> & measurement_result_vec, std::vector<point_3d> & marked_begin_points);

private:
	int deter_begin_point(std::vector<measurement_content> & measurement_result_vec, std::vector<point_3d> & marked_begin_points);
	
	void make_order(std::vector<measurement_content> & measurement_result_vec);

	void cut_off(std::vector<measurement_content> & measurement_result_vec);

	void min_vecs(std::vector<point_3d> & vec_1, std::vector<point_3d> & vec_2, float & min_distance, size_t &vec_1_index, size_t &vec_2_index);
};

#endif // APP_WELDING
