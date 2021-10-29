#ifndef CLOUD_MEASUREMENT_H
#define CLOUD_MEASUREMENT_H

#include "common_use.h"
#include "cloud_fitting.h"
#include "cloud_search.h"

#define INVALIDVALUE -1.0f

enum measurement_methods
{
	// calculate distance and angle
	MM_VALUES, 
	// create points
	MM_POINTS
};

struct measurement_content
{
	// INVALIDVALUE -> -1
	measurement_content() : distance_geometrical(INVALIDVALUE), distance_scattered(INVALIDVALUE), angle(INVALIDVALUE) {}

	// 1. values
	// distance based on geometry(function)
	float distance_geometrical;
	// distance based on scattered points
	float distance_scattered;
	// angle in degree 
	float angle;

	// 2. points representing the relationship
	std::vector<point_3d> drawable_points;

	// 3. measurement method
	measurement_methods m_method;

};

struct pair_content
{
	std::string source_object;
	std::string target_object;
	std::string reference_object;
};

class cloud_measurement
{
public:
	cloud_measurement(std::vector<point_3d> & point_cloud, std::map<std::string, std::vector<point_3d>> & searched_mark_points_map);

	~cloud_measurement();

	void measure();

	size_t read_pair_file(const std::string & filename);

	int post_process(Eigen::Matrix4f & matrix);

	size_t export_measured_data(const std::string & output_filename);

	std::vector<measurement_content> & get_measurement_result();

private:
	void analyse_defect(std::vector<point_3d>& scanned_points, std::vector<point_3d>& standard_model, std::vector<point_3d> & defect_points);
	
	void fill_available_vector(point_3d & p, std::vector<point_3d> & points);

	std::vector<measurement_content> m_mc_vec;
	std::vector<pair_content> m_pair_vec;

	std::map<std::string, std::vector<point_3d>> &m_searched_mark_points_map;
	std::vector<point_3d> & m_point_cloud;
	kd_tree m_point_cloud_tree;

private:
	void decide_objects(const std::string & label_1, const std::string & label_2, std::vector<size_t> &upper, std::vector<size_t> & lower);

	void analyze_points(
		std::vector<size_t> & order_1, std::vector<size_t> & order_2, 
		pair_content & pc, measurement_content & mc);

	cloud_fitting cf;

private:
	// the first points belongs to the type of point
	void point_to_point(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);
	void point_to_line(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);
	void point_to_plane(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);
	void point_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);

	// the first points belongs to the type of line
	void line_to_line(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);
	void line_to_plane(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);
	void line_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);

	// the first points belongs to the type of plane
	void plane_to_plane(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, std::vector<point_3d>& reference_points, measurement_content & mc);
	void plane_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, std::vector<point_3d>& reference_points, measurement_content & mc);

	// the first points belongs to the type of cylinder
	void cylinder_to_cylinder(std::vector<point_3d> & points_1, std::vector<point_3d> & points_2, measurement_content & mc);

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


