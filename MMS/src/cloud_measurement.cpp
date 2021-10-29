#include "cloud_measurement.h"

cloud_measurement::cloud_measurement(
	std::vector<point_3d> & point_cloud,
	std::map<std::string, std::vector<point_3d>> & searched_mark_points_map)
	:
	m_point_cloud_tree(point_cloud),
	m_point_cloud(point_cloud),
	m_searched_mark_points_map(searched_mark_points_map)
{

}

cloud_measurement::~cloud_measurement()
{

}

size_t cloud_measurement::read_pair_file(const std::string & filename)
{
	LocalFile local_file;

	if (!check_file(filename, std::ios::in, local_file)) return 1;

	std::fstream & ifile = local_file.m_fileobject;

	m_pair_vec.clear();

	std::string line;

	size_t line_index = 0;
	while (std::getline(ifile, line))
	{
		if (line.empty()) break;

		if (line[0] == '#') continue;

		std::vector<std::string> split_str;

		string_split(line, ':', split_str);

		pair_content pc;

		pc.source_object = split_str[0];
		pc.target_object = split_str[1];

		if (split_str.size() == 3)
		{
			pc.reference_object = split_str[2];
		}
		else if (split_str.size() == 2)
		{
			pc.reference_object.clear();
		}
		else
		{
			std::cerr << "error: the number of measurement pair should be 3 or 2" << std::endl;
			continue;
		}

		m_pair_vec.push_back(pc);
	}

	return m_pair_vec.size();
}

int cloud_measurement::post_process(Eigen::Matrix4f & matrix)
{
	// calculate the available vector as normals for every draw-point
	for (auto & pv : m_mc_vec)
	{
		for (auto &p : pv.drawable_points)
		{
			std::vector<size_t> ret_index;  std::vector<float> out_dist_sqr;
			if (m_point_cloud_tree.search_neighbors_knn(20, p, ret_index, out_dist_sqr))
			{
				std::vector<point_3d> subset_points;
				subset_of_point_cloud(ret_index, m_point_cloud, subset_points);
				fill_available_vector(p, subset_points);
			}
		}
	}

	// transform points back to the original coordinate
	//for (auto & pv : m_mc_vec)
	//	for (auto &p : pv.drawable_points)
	//		p.do_transform(matrix.inverse());

	return 0;
}

void cloud_measurement::measure()
{
	m_mc_vec.clear();

	for (auto & pair: m_pair_vec)
	{
		measurement_content mc;

		if (pair.reference_object.empty())
		{
			mc.m_method = MM_VALUES;
		}
		else
		{
			mc.m_method = MM_POINTS;
		}

		std::vector<size_t> upper(4, 0), lower(4, 0);

		decide_objects(pair.source_object, pair.target_object, upper, lower);

		analyze_points(upper, lower, pair, mc);

		m_mc_vec.push_back(mc);
	}
}

void cloud_measurement::analyse_defect(std::vector<point_3d>& scanned_points, std::vector<point_3d>& standard_model, std::vector<point_3d> & defect_points)
{
	kd_tree kt(scanned_points);

	std::set<size_t> correspondences_index;
	kt.search_points_correspondence(standard_model, correspondences_index, 0.2);

	for (size_t i = 0; i < scanned_points.size(); ++i)
	{
		if (correspondences_index.find(i) == correspondences_index.end())
			defect_points.push_back(scanned_points[i]);
	}
}

void cloud_measurement::fill_available_vector(point_3d & p, std::vector<point_3d>& points)
{
	float total_x = 0.0, total_y = 0.0, total_z = 0.0;

	for (auto & p_t : points)
	{
		float this_x, this_y, this_z;
		this_x = p_t.x - p.x;
		this_y = p_t.y - p.y;
		this_z = p_t.z - p.z;

		total_x += this_x;
		total_y += this_y;
		total_z += this_z;
	}
	Eigen::Vector3f vec;
	vec[0] = total_x / points.size();
	vec[1] = total_y / points.size();
	vec[2] = total_z / points.size();
	vec.normalize();
	p.set_nxyz(vec[0], vec[1], vec[2]);
}

void cloud_measurement::decide_objects(
	const std::string & label_1, const std::string & label_2, 
	std::vector<size_t> &upper, std::vector<size_t> & lower)
{
	if (label_1.find("point") != std::string::npos)
		upper[0] = 1;
	else if (label_1.find("line") != std::string::npos)
		upper[1] = 1;
	else if (label_1.find("plane") != std::string::npos)
		upper[2] = 1;
	else if (label_1.find("cylinder") != std::string::npos)
		upper[3] = 1;

	if (label_2.find("point") != std::string::npos)
		lower[0] = 1;
	else if (label_2.find("line") != std::string::npos)
		lower[1] = 1;
	else if (label_2.find("plane") != std::string::npos)
		lower[2] = 1;
	else if (label_2.find("cylinder") != std::string::npos)
		lower[3] = 1;
}

void cloud_measurement::analyze_points(
	std::vector<size_t>& order_1, std::vector<size_t>& order_2, 
	pair_content & pc, measurement_content & mc)
{
	size_t first_type, second_type;

	for (size_t i = 0; i < order_1.size(); ++i)
	{
		if (order_1[i] != 0)
			first_type = i;

		if (order_2[i] != 0)
			second_type = i;
	}

	std::vector<point_3d> &
		points_1 = m_searched_mark_points_map[pc.source_object],
		points_2 = m_searched_mark_points_map[pc.target_object];
	
	std::vector<point_3d> empty_vec;
	empty_vec.clear();

	std::vector<point_3d> * reference_points = nullptr;
	if (pc.reference_object.empty())
	{
		reference_points = &empty_vec;
	}
	else
	{
		reference_points = & m_searched_mark_points_map[pc.reference_object];
	}


	// point
	if (first_type == 0 && second_type == 0)
	{
		point_to_point(points_1, points_2, mc);
	}
	else if (first_type == 0 && second_type == 1)
	{
		point_to_line(points_1, points_2, mc);
	}
	else if (first_type == 0 && second_type == 2)
	{
		point_to_plane(points_1, points_2, mc);
	}
	else if (first_type == 0 && second_type == 3)
	{
		point_to_cylinder(points_1, points_2, mc);
	}
	// line
	// same as the 0->1, but exchange the parameters
	else if (first_type == 1 && second_type == 0)
	{
		point_to_line(points_2, points_1, mc);
	}
	else if (first_type == 1 && second_type == 1)
	{
		line_to_line(points_1, points_2, mc);
	}
	else if (first_type == 1 && second_type == 2)
	{
		line_to_plane(points_1, points_2, mc);
	}
	else if (first_type == 1 && second_type == 3)
	{
		line_to_cylinder(points_1, points_2, mc);
	}
	// plane
	// same as the 0->2, but exchange the parameters
	else if (first_type == 2 && second_type == 0)
	{
		point_to_plane(points_2, points_1, mc);
	}
	// same as the 1->2, but exchange the parameters
	else if (first_type == 2 && second_type == 1)
	{
		line_to_plane(points_2, points_1, mc);
	}
	else if (first_type == 2 && second_type == 2)
	{
		plane_to_plane(points_1, points_2, *reference_points, mc);
	}
	else if (first_type == 2 && second_type == 3)
	{
		plane_to_cylinder(points_1, points_2, *reference_points, mc);
	}
	// cylinder
	// same as the 0->3, but exchange the parameters
	else if (first_type == 3 && second_type == 0)
	{
		point_to_cylinder(points_2, points_1, mc);
	}
	// same as the 1->3, but exchange the parameters
	else if (first_type == 3 && second_type == 1)
	{
		line_to_cylinder(points_2, points_1, mc);
	}
	// same as the 2->3, but exchange the parameters
	else if (first_type == 3 && second_type == 2)
	{
		plane_to_cylinder(points_2, points_1, *reference_points, mc);
	}
	else if (first_type == 3 && second_type == 3)
	{
		cylinder_to_cylinder(points_1, points_2, mc);
	}
}

std::vector<measurement_content> & cloud_measurement::get_measurement_result()
{
	return m_mc_vec;
}

size_t cloud_measurement::export_measured_data(const std::string & output_filename)
{
	if (m_mc_vec.empty() || m_pair_vec.empty()) return 1;

	if (m_mc_vec.size() != m_pair_vec.size()) return 2;

	LocalFile local_file;

	if (!check_file(output_filename, std::ios::out, local_file)) return 1;

	std::fstream & ofile = local_file.m_fileobject;

	size_t i = 0;
	for (auto & mc : m_mc_vec)
	{
		if (mc.m_method == MM_VALUES)
		{
			ofile << ">values" << "\n";
			if (mc.distance_geometrical == INVALIDVALUE)
				ofile << INVALIDVALUE << " ";
			else
				ofile << mc.distance_geometrical << " ";

			if (mc.distance_scattered == INVALIDVALUE)
				ofile << INVALIDVALUE << " ";
			else
				ofile << mc.distance_scattered << " ";

			if (mc.angle == INVALIDVALUE)
				ofile << INVALIDVALUE << " ";
			else
				ofile << mc.angle << " ";

			ofile << "\n";
		}
		else if (mc.m_method == MM_POINTS)
		{
			ofile << ">points" << "\n";
			for (auto & p : mc.drawable_points)
			{
				ofile
					<< p.x << " " << p.y << " " << p.z << " "
					<< p.nx << " " << p.ny << " " << p.nz
					<< "\n";
			}
		}

		std::string label = "#" + m_pair_vec[i].source_object + "-" + m_pair_vec[i].target_object + "-" + m_pair_vec[i].reference_object;
		ofile << label << "\n";

		++i;
	}
	ofile.close();

	return 0;
}

void cloud_measurement::point_to_point(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "point_to_point\n";

	point_3d centroid_point_1, centroid_point_2;
	centroid_from_points(points_1, centroid_point_1);
	centroid_from_points(points_2, centroid_point_2);

	// geometrical distance
	distance_point_to_point(centroid_point_1, centroid_point_2, mc.distance_geometrical);
	
	// scattered distance
	distance_scattered_points(points_1, points_2, mc.distance_scattered);
}

void cloud_measurement::point_to_line(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "point_to_line\n";

	// calculate the geometrical distance.
	point_3d centroid_point_1;
	centroid_from_points(points_1, centroid_point_1);

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_2, line_func);

	// geometrical distance
	distance_point_to_line(centroid_point_1, line_func, mc.distance_geometrical);

	// scattered distance
	distance_scattered_points(points_1, points_2, line_func, mc.distance_scattered);
}

void cloud_measurement::point_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "point_to_plane\n";

	point_3d centroid_point;
	centroid_from_points(points_1, centroid_point);

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func);

	// geometrical distance
	distance_point_to_plane(centroid_point, plane_func, mc.distance_geometrical);

	// scattered distance
	distance_scattered_points(points_1, points_2, plane_func, mc.distance_scattered);
}

void cloud_measurement::point_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "point_to_cylinder\n";

	point_3d centroid_point;
	centroid_from_points(points_1, centroid_point);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	// geometrical distance
	distance_point_to_line(centroid_point, _cylinder_func.axis, mc.distance_geometrical);
}

void cloud_measurement::line_to_line(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "line_to_line\n";

	line_func_3d line_func_1, line_func_2;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func_1);
	cf.fitting_line_3d_linear_least_squares(points_2, line_func_2);

	// angle
	angle_between_two_vector_3d(line_func_1.direction, line_func_2.direction, mc.angle);
	if (mc.angle > 90)
		mc.angle = 180 - mc.angle;

	// scattered distance
	distance_scattered_points(points_1, line_func_1, points_2, line_func_2, mc.distance_scattered);
}

void cloud_measurement::line_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "line_to_plane\n";

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func);

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func);

	// angle
	float radian = 0.0;
	radian = radian_two_vector_3d(line_func.direction, plane_func.direction());
	mc.angle = radian2degree(radian);
	//angle_between_two_vector_3d(line_func.direction, plane_func.direction<Eigen::Vector3f>(), mc.angle);
	if (mc.angle > 90)
		mc.angle = 180 - mc.angle;

	mc.angle = 90 - mc.angle;

	// scattered distance
	distance_scattered_points(points_1, line_func, points_2, plane_func, mc.distance_scattered);
}

void cloud_measurement::line_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "line_to_cylinder\n";

	line_func_3d line_func;
	cf.fitting_line_3d_linear_least_squares(points_1, line_func);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	// angle
	angle_between_two_vector_3d(line_func.get_direction_point_3d(), _cylinder_func.axis.get_direction_point_3d(), mc.angle);
	if (mc.angle > 90)
		mc.angle = 180 - mc.angle;

	// geometrical distance(point(cylinder) to line)
	distance_point_to_line(_cylinder_func.axis.get_origin_point_3d(), line_func, mc.distance_geometrical);
}

void cloud_measurement::plane_to_plane_values(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, plane_func_3d & plane_func_1, plane_func_3d & plane_func_2, measurement_content & mc)
{
	std::cout << "plane_to_plane with VALUES \n";
	
	// angle
	float radian = 0.0;
	radian = radian_two_vector_3d(plane_func_1.direction(), plane_func_2.direction());
	mc.angle = radian2degree(radian);

	// scattered distance
	distance_scattered_points(points_1, plane_func_1, points_2, plane_func_2, mc.distance_scattered);
	//std::cout << mc.distance_scattered << std::endl;
}

void cloud_measurement::plane_to_plane_points(plane_func_3d & plane_func_1, plane_func_3d & plane_func_2, std::vector<point_3d>& reference_points, measurement_content & mc)
{
	std::cout << "plane_to_plane with POINTS \n";

	line_func_3d intersection_lf;
	line_function_two_planes(plane_func_1, plane_func_2, intersection_lf);

	// make the drawable data
	// 1. project reference point onto a specific line
	std::vector<point_3d> points_on_intersection_line;
	project_points_onto_line(reference_points, intersection_lf, points_on_intersection_line);

	// 2. determine the end point and begin point
	point_3d endpoint_a, endpoint_b;
	endpoints_line(points_on_intersection_line, endpoint_a, endpoint_b);

	// 3. create a line using begin and end point
	produce_line_points(intersection_lf, endpoint_a, endpoint_b, mc.drawable_points, 100);
}

void cloud_measurement::plane_to_plane(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, std::vector<point_3d>& reference_points, measurement_content & mc)
{
	plane_func_3d plane_func_1, plane_func_2;
	cf.fitting_plane_3d_linear_least_squares(points_1, plane_func_1);
	cf.fitting_plane_3d_linear_least_squares(points_2, plane_func_2);

	if (mc.m_method == MM_VALUES)
	{
		plane_to_plane_values(points_1, points_2, plane_func_1, plane_func_2, mc);
	}
	else if (mc.m_method == MM_POINTS)
	{
		plane_to_plane_points(plane_func_1, plane_func_2, reference_points, mc);
	}
}

void cloud_measurement::plane_to_cylinder_values(plane_func_3d & plane_func, cylinder_func & _cylinder_func, measurement_content & mc)
{
	// angle
	float radian = 0.0;
	radian = radian_two_vector_3d(plane_func.direction(), _cylinder_func.axis.direction);
	mc.angle = radian2degree(radian);
	//angle_between_two_vector_3d(plane_func.direction<point_3d>(), cylinder_func.axis.get_direction_point_3d(), mc.angle);
	if (mc.angle > 90)
		mc.angle = 180 - mc.angle;

	mc.angle = 90 - mc.angle;

	// geometrical distance(point(cylinder) to plane)
	distance_point_to_plane(_cylinder_func.axis.get_origin_point_3d(), plane_func, mc.distance_geometrical);
}

void cloud_measurement::plane_to_cylinder_points(plane_func_3d & plane_func, cylinder_func & _cylinder_func, std::vector<point_3d>& reference_points, measurement_content & mc)
{
	if (reference_points.size() < 2)
	{
		std::cerr << "error: the number of reference points in plane_to_cylinder()" << std::endl;
		return;
	}

	Eigen::Vector3f
		plane_direction = plane_func.direction(),
		cylinder_direction = _cylinder_func.axis.direction;

	plane_direction.normalize(); cylinder_direction.normalize();

	// make sure that it has the same direction as Z axis
	if (cylinder_direction.dot(Eigen::Vector3f::UnitZ()) < 0)
		cylinder_direction = -1 * cylinder_direction;

	// 1. construct the new plane near the origin
	// 1.1 get transformation between direction of cylinder and axis-Z
	Eigen::Matrix3f rotate_t;
	// rotate_t -> v1 to v2
	rotation_transformation(cylinder_direction, Eigen::Vector3f::UnitZ(), rotate_t);

	// 1.1 create a new plane near the origin
	// rotate the plane
	Eigen::Vector3f new_plane_direction;
	new_plane_direction = rotate_t.inverse().transpose() * plane_direction;
	///std::cout << new_plane_direction << std::endl;
	///std::cout
	///	<< "rotate_t=" << rotate_t << "\n\n"
	///	<< new_plane_direction << "\n\n";
	plane_func_3d new_plane_func;
	point_3d inter_line_plane_point;
	intersection_line_to_plane(_cylinder_func.axis, plane_func, inter_line_plane_point);
	float d =
		-(new_plane_func.a * inter_line_plane_point.x + new_plane_func.b * inter_line_plane_point.y + new_plane_func.c * inter_line_plane_point.z);

	new_plane_func.set_abcd(new_plane_direction[0], new_plane_direction[1], new_plane_direction[2], d);


	// 2. determine a range restricting the intersection points
	// 2.1 get a matrix transforming the whole point cloud into the origin
	Eigen::Matrix4f t_rotate_(Eigen::Matrix4f::Identity());
	t_rotate_.block<3, 3>(0, 0) = rotate_t;

	Eigen::Matrix4f t_translate(Eigen::Matrix4f::Identity());
	t_translate(0, 3) = -inter_line_plane_point.x;
	t_translate(1, 3) = -inter_line_plane_point.y;
	t_translate(2, 3) = -inter_line_plane_point.z;

	Eigen::Matrix4f t_whole_point_cloud = t_rotate_ * t_translate;
	std::vector<point_3d> ref_pts_normalized;
	transform_points(reference_points, t_whole_point_cloud, ref_pts_normalized);

#ifdef TEST_MEASUREMENT
	std::cout << t_whole_point_cloud << std::endl;
#endif

	// 2.2 get a radian range that includes these points
	std::vector<float> rad_vec;
	for (auto & v : ref_pts_normalized)
	{
		Eigen::Vector3f tmp_v(v.x, v.y, 0);

		float rad = 0.0;
		radian_two_vector_2d_022PI(Eigen::Vector3f::UnitX(), tmp_v, rad);
		rad_vec.push_back(rad);
#ifdef TEST_MEASUREMENT
		std::cout << tmp_v << std::endl;
		std::cout << radian2degree(rad) << std::endl;
#endif
	}
	std::sort(rad_vec.begin(), rad_vec.end());
	float min_rad = rad_vec.front(), max_rad = rad_vec.back();


	// 3. create the intersection points
	// 3.1 set up the parameters 
	float number_points = 100; // number of generated points
	float
		R = _cylinder_func.radius + 1,
		A = new_plane_func.a, B = new_plane_func.b, C = new_plane_func.c, D = new_plane_func.d,
		inc_radian = (max_rad - min_rad) / number_points;

	// 3.2 create virtual points near the origin
	std::vector<point_3d> ineter_points, ineter_points_;
	for (float r/*radian*/ = min_rad; r < max_rad; r += inc_radian)
	{
		float x, y, z;
		x = R * cosf(r);
		y = R * sinf(r);
		z = (A * x + B * y + D) / (-C);
		ineter_points.push_back(point_3d(x, y, z));
	}

	// 4. back these virtual points to the position of original point cloud
	transform_points(ineter_points, t_whole_point_cloud.inverse(), ineter_points_);
	mc.drawable_points = ineter_points_;
	//save_points(ineter_points, "temp.txt");
}


void cloud_measurement::plane_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, std::vector<point_3d>& reference_points, measurement_content & mc)
{
	std::cout << "plane_to_cylinder\n";

	plane_func_3d plane_func;
	cf.fitting_plane_3d_linear_least_squares(points_1, plane_func);

	cylinder_func _cylinder_func;
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func);

	if (mc.m_method == MM_VALUES)
	{
		plane_to_cylinder_values(plane_func, _cylinder_func, mc);
	}
	else if (mc.m_method == MM_POINTS)
	{
		plane_to_cylinder_points(plane_func, _cylinder_func, reference_points, mc);
	}
}


void cloud_measurement::cylinder_to_cylinder(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, measurement_content & mc)
{
	std::cout << "cylinder_to_cylinder\n";

	cylinder_func _cylinder_func_1, _cylinder_func_2;
	cf.fitting_cylinder_linear_least_squares(points_1, _cylinder_func_1);
	cf.fitting_cylinder_linear_least_squares(points_2, _cylinder_func_2);

	// angle
	angle_between_two_vector_3d(_cylinder_func_1.axis.get_direction_point_3d(), _cylinder_func_2.axis.get_direction_point_3d(), mc.angle);
	if (mc.angle > 90)
		mc.angle = 180 - mc.angle;
	
	// geometrical distance(point(cylinder) to line(cylinder))
	distance_point_to_line(_cylinder_func_1.axis.get_origin_point_3d(), _cylinder_func_2.axis, mc.distance_geometrical);
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, float & distance)
{
	float min_dis = FLT_MAX;

	for (size_t i = 0; i < points_1.size(); ++i)
	{
		for (size_t j = 0; j < points_2.size(); ++j)
		{
			float dis = 0.0;
			distance_point_to_point(points_1[i], points_2[j], dis);

			if (dis < min_dis)
				min_dis = dis;
		}
	}
	distance = min_dis;
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, line_func_3d lf_2, float & distance)
{
	std::vector<point_3d> points_line_2;
	points_line(lf_2, points_2, points_line_2);

	distance_scattered_points(points_1, points_line_2, distance);
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, plane_func_3d & plane_func_2, float & distance)
{
	std::vector<point_3d> points_plane_2;
	points_plane(plane_func_2, points_2, points_plane_2);

	//save_points(points_plane_2, "output/plane.txt");

	distance_scattered_points(points_1, points_plane_2, distance);
}

void cloud_measurement::distance_scattered_points(
	std::vector<point_3d>& points_1, line_func_3d &lf_1,
	std::vector<point_3d>& points_2, line_func_3d &lf_2,
	float & distance)
{
	std::vector<point_3d> points_line_1;
	points_line(lf_1, points_1, points_line_1);

	std::vector<point_3d> points_line_2;
	points_line(lf_2, points_2, points_line_2);

	distance_scattered_points(points_line_1, points_line_2, distance);
}

void cloud_measurement::distance_scattered_points(std::vector<point_3d>& points_1, line_func_3d & lf_1, std::vector<point_3d>& points_2, plane_func_3d & plane_func_2, float & distance)
{
	std::vector<point_3d> points_line_1;
	points_line(lf_1, points_1, points_line_1);

	std::vector<point_3d> points_plane_2;
	points_plane(plane_func_2, points_2, points_plane_2);

	distance_scattered_points(points_line_1, points_plane_2, distance);
}

static int t_c = 0;

void cloud_measurement::distance_scattered_points(
	std::vector<point_3d>& points_1, plane_func_3d & plane_func_1, 
	std::vector<point_3d>& points_2, plane_func_3d & plane_func_2,
	float & distance)
{
	std::vector<point_3d> points_plane_1;
	points_plane(plane_func_1, points_1, points_plane_1);
	save_points(points_plane_1, "output/plane_" + std::to_string(++t_c) + ".txt");

	std::vector<point_3d> points_plane_2;
	points_plane(plane_func_2, points_2, points_plane_2);
	save_points(points_plane_2, "output/plane_" + std::to_string(++t_c) + ".txt");

	distance_scattered_points(points_plane_1, points_plane_2, distance);
}

