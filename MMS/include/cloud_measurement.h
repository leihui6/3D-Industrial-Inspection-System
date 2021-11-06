#ifndef CLOUD_MEASUREMENT_H
#define CLOUD_MEASUREMENT_H

#include "common_use.h"
#include "cloud_fitting.h"
#include "cloud_search.h"

// welding application
#include "app_welding.h"

struct pair_content
{
	std::string source_object;
	std::string target_object;
	std::string reference_object;
};

class cloud_measurement
{
public:
	cloud_measurement(std::vector<point_3d> & point_cloud, std::map<std::string, point_shape> & searched_mark_points_map);

	~cloud_measurement();

	void measure();

	size_t read_pair_file(const std::string & filename);

	int post_process(Eigen::Matrix4f & matrix);

	size_t export_measured_data(const std::string & output_filename);

	std::vector<measurement_content> & get_measurement_result();

private:
	void analyse_defect(std::vector<point_3d>& scanned_points, std::vector<point_3d>& standard_model, std::vector<point_3d> & defect_points);
	
	std::vector<measurement_content> m_mc_vec;
	std::vector<pair_content> m_pair_vec;

	std::map<std::string, point_shape> & m_searched_mark_points_map;
	// transformed point cloud
	std::vector<point_3d> & m_point_cloud;
	kd_tree m_point_cloud_tree;

private:
	void decide_objects(const std::string & label_1, const std::string & label_2, std::vector<size_t> &upper, std::vector<size_t> & lower);

	void analyze_points(
		std::vector<size_t> & order_1, std::vector<size_t> & order_2, 
		pair_content & pc, measurement_content & mc);

	void correct_normals(std::vector<point_3d> & points, const Eigen::Vector3f * v1 = nullptr, const Eigen::Vector3f * v2 = nullptr);

	cloud_fitting cf;
	app_welding m_welding;

	size_t m_measured_point_number;
	float m_deviation_length;

private:
	// the first points belongs to the type of point
	void point_to_point(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);
	void point_to_line(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);
	void point_to_plane(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);
	void point_to_cylinder(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);

	// the first points belongs to the type of line
	void line_to_line(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);
	void line_to_plane(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);
	void line_to_cylinder(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);

	// the first points belongs to the type of plane
	void plane_to_plane(point_shape & shape_1, point_shape & shape_2, std::vector<point_3d>& reference_points, measurement_content & mc);
	void plane_to_cylinder(point_shape & shape_1, point_shape & shape_2, std::vector<point_3d>& reference_points, measurement_content & mc);

	// the first points belongs to the type of cylinder
	void cylinder_to_cylinder(point_shape & shape_1, point_shape & shape_2, measurement_content & mc);

private:
	void distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, float & distance);

	// point to line
	void distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, line_func_3d lf_2, float & distance);

	// point to plane
	void distance_scattered_points(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, plane_func_3d &plane_func_2, float & distance);

	// line to line
	void distance_scattered_points(std::vector<point_3d>& points_1, line_func_3d &lf_1, std::vector<point_3d>& points_2, line_func_3d &lf_2, float & distance);

	// line to line
	void distance_scattered_points(std::vector<point_3d>& points_1, line_func_3d &lf_1, std::vector<point_3d>& points_2, plane_func_3d &plane_func_2, float & distance);

	// plane to plane
	void distance_scattered_points(std::vector<point_3d>& points_1, plane_func_3d &plane_func_1, std::vector<point_3d>& points_2, plane_func_3d &plane_func_2, float & distance);

private:
	// process the MM_VALUES and MM_POINTS in plane_to_plane
	void plane_to_plane_values(std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, plane_func_3d & plane_func_1, plane_func_3d & plane_func_2, measurement_content & mc);
	void plane_to_plane_points(plane_func_3d & plane_func_1, plane_func_3d & plane_func_2, std::vector<point_3d>& reference_points, measurement_content & mc);

	// process the MM_VALUES and MM_POINTS in plane_to_cylinder
	void plane_to_cylinder_values(plane_func_3d & plane_func, cylinder_func & _cylinder_func, measurement_content & mc);
	void plane_to_cylinder_points(plane_func_3d & plane_func, cylinder_func & _cylinder_func, std::vector<point_3d>& reference_points, measurement_content & mc);


};

#endif // !CLOUD_MEASUREMENT_H


