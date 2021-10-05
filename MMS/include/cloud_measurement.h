#ifndef CLOUD_MEASUREMENT
#define CLOUD_MEASUREMENT

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

class cloud_measurement
{
public:
	cloud_measurement(std::vector<point_3d> & point_cloud);

	~cloud_measurement();

	void measure(
		std::multimap<std::string, std::string> & measurement_pairs_map, 
		std::map<std::string, std::vector<point_3d>>& _m, 
		std::multimap<std::string, std::string> & reference_map, 
		std::vector<measurement_content> & mc_vec);

	void analyse_defect(std::vector<point_3d>& scanned_points, std::vector<point_3d>& standard_model, std::vector<point_3d> & defect_points);

private:
	void measure(std::string & points_1_label, std::string & points_2_label, std::string & reference_label,
		std::map<std::string, std::vector<point_3d>>& _m, measurement_content & mc);

	void analyze_points(std::vector<size_t> & order_1, std::vector<size_t> & order_2, 
		std::vector<point_3d>& points_1, std::vector<point_3d>& points_2, std::vector<point_3d>& reference_points,
		measurement_content & mc);

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



private:
	std::vector<point_3d> m_point_cloud;
	kd_tree m_point_cloud_tree;
};

#endif // !CLOUD_MEASUREMENT


