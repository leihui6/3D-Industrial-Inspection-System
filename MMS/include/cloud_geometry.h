#ifndef CLOUD_GEOMETRY_H
#define CLOUD_GEOMETRY_H

#include <windows.h>

// Stardard library in c++ 11 
#include <vector>
#include <fstream>
#include <math.h>
#include <memory>
#define _USE_MATH_DEFINES

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;

// OpenGR
#include <gr/algorithms/match4pcsBase.h>

// PointMatcher
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>

// OSG
#include <OSG/Vec3d>
#include <osg/Geometry>
#include <osg/Geode>

#include "common_use.h"

point_3d to_point_3d(osg::Vec3d & p);

// convert to cgal point containing x,y,z,n,m,l
void convert_to_CGAL_points(std::vector<point_3d> & points, std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3>> &cgal_points);

// convert to cgal point only containing x,y,z
void convert_to_CGAL_points(std::vector<point_3d> & points, std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points);

void convert_to_original_points(std::vector< CGAL::Simple_cartesian<float>::Point_3> &cgal_points, std::vector<point_3d> & points);

void convert_to_openGR_points(std::vector<point_3d> & points, std::vector<gr::Point3D<float>> & opengr_points);

void convert_to_pointMatcher_points(std::vector<point_3d> & points, PointMatcher<float>::DataPoints & DP);

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> color, osg::ref_ptr<osg::Vec3Array> normals, float r = 0, float g = 0, float b = 0, float w = 1.0);

// osg needs (p,height,radius) to draw cylinder on screen instead of only cylinder function
void cylinder_func_to_osg_structure(std::vector<point_3d> & points, cylinder_func & cl, point_3d & center_p, float &height, float &radius);

void points_to_geometry_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> geometry, float r = 0, float g = 0, float b = 0, float w = 1.0);

// get minimal point and maximal point in a specific point set
void max_min_point_3d_vec(std::vector<point_3d> & points, point_3d & min_p, point_3d & max_p);

// get minimal value and maximal value in a specific array
void max_min_value_array(std::vector<float> & vec, float & min_value, float & max_value);

// calculate the min and max t in line function, min_max_t[0-2]: min_t; min_max[3-5]:max_t
//void man_min_t_line_function(line_func_3d & line_func, point_3d & min_p, point_3d & max_p, std::vector<float>& min_t, std::vector<float> &max_t);

// calculate the appropriate t that could let point be closer to target point
//void get_appropriate_t(line_func_3d & line_func, std::vector<float> t_vec, point_3d target_point, float & real_t);

// void get_distance_points_to_plane(std::vector<float> & points, plane_func_3d & plane_func, std::vector<float> & dis_to_plane);

void transform_points(std::vector<point_3d>& points, const Eigen::Matrix4f & t, std::vector<point_3d>& ret_points);

// get a pedal point from a point to a line
void pedalpoint_point_to_line(const point_3d & point, const line_func_3d & _line_func_3d, point_3d & pedalpoint);

// get a pedal point from a point to a plane
void pedalpoint_point_to_plane(const point_3d & point, const plane_func_3d & plane_func, point_3d & pedalpoint);

void distance_points_to_line(const std::vector<point_3d>& points, const line_func_3d & _line_func_3d, std::vector<float>& points_dis_vec);

void distance_point_to_line(const point_3d& points, const line_func_3d & _line_func_3d, float & points_dis);

void distance_point_to_point(const point_3d & point_1, const point_3d & point_2, float & distance);

void distance_point_to_plane(const point_3d & point, const plane_func_3d & plane_func, float & distance);

void save_points(const std::vector<point_3d>& points, const std::string & filename);

void make_points_ordered_by_distance(std::vector<point_3d>& points, std::vector<point_3d>& ordered_points);

void point_along_with_vector_within_dis(point_3d & point, const Eigen::Vector3f & line_dir, point_3d & result_p, float length);
//void point_along_with_vector_within_dis(point_3d & point, Eigen::Vector3f & line_dir, point_3d & result_p1, point_3d & result_p2, float distance);

bool is_parallel_vector(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2);

void plane_function_from_three_points(point_3d & A, point_3d & B, point_3d & C, plane_func_3d & plane_func);

void intersection_line_to_sphere(line_func_3d & line_func, point_3d & sphere_center, float & sphere_r, point_3d & res_p1, point_3d & res_p2);

void intersection_line_to_plane(line_func_3d & line_func, plane_func_3d & plane_func, point_3d & inter_p);

void points_on_plane(std::vector<point_3d>& points, std::vector<point_3d>& res_points_on_plane, plane_func_3d & plane_func, float distance_threshold);
void points_on_planes(std::vector<point_3d>& points, std::vector<std::vector<point_3d>>& res_points_on_planes, std::vector<plane_func_3d> & plane_funcs, float distance_threshold);

void points_on_line(std::vector<point_3d>& points, std::vector<point_3d>& points_on_line, line_func_3d & line_func, float distance_threshold);
//void points_on_line(std::vector<point_3d>& points, std::vector<int>& index_vec, std::vector<point_3d>& points_on_line, line_func_3d & line_func, float distance_threshold);

// get points constructing this line
void points_line(line_func_3d & line_func, std::vector<point_3d>& points, std::vector<point_3d> & points_line);

// get points constructing this plane
void points_plane(plane_func_3d & plane_func, std::vector<point_3d>& points, std::vector<point_3d> & points_plane);

// find end points among points which are on a line, so please make sure that all these points are on one line
void endpoints_line(std::vector<point_3d>& points, point_3d & point_a, point_3d & point_b);

// produce points based on line function, and you can set the number of points
// v = endpoint_b - endpoint_a;
// the generated line is from endpoint_a to endpoint_b
void produce_line_points(Eigen::Vector3f &v, point_3d & beg_p, point_3d & end_p, std::vector<point_3d>& line_points, size_t point_number);

// get two points (A and B), which is the begin and end point of line respectively.
void segment_point_from_points(point_3d& p_A, point_3d & p_B, std::vector<point_3d>& points, line_func_3d & line_func);

void points_on_cylinder(std::vector<point_3d>& points, std::vector<point_3d>& points_on_cylinder, cylinder_func & _cylinder_func, float threshold);

void points_on_plane_circle(std::vector<point_3d>& points, std::vector<point_3d>& points_on_plane_circle, plane_func_3d & plane_func, point_3d & circle_center, float circle_r, float threshold_on_plane = 0.1f, float threshold_in_circle = 0.1f);

void centroid_from_points(std::vector<point_3d>& points, point_3d & centroid_point);

void combine_vectors(std::vector<point_3d>& points_1, std::vector<point_3d> & points_2, std::vector<point_3d>& combined_result);

void standard_deviation(std::vector<float> & vec, float & deviation);

void line_function_two_planes(plane_func_3d & pf1, plane_func_3d & pf2, line_func_3d & lf);

// consider the ax+by=c
void linear_equation_with_2_unknowns(float a1, float b1, float c1, float a2, float b2, float c2, float & r_x, float & r_y);

// the probability that close to specifical value in vector
void probability_close_to_value(std::vector<float> & vec, float specifical_value, float threshold, float & probability);

void mean_distance_from_point_to_points(std::vector<point_3d>& points, point_3d & point, float & mean_distance);

void longgest_distance_from_point_to_points(std::vector<point_3d>& points, point_3d & point, float & longgest_distance);

float vector_cos(const Eigen::Vector3f & v1, const Eigen::Vector3f &v2);

float angle_between_two_vector_3d(const Eigen::Vector3f & v1, const Eigen::Vector3f &v2);
float angle_between_two_point_3d(const point_3d & p1, const point_3d & p2);
float radian_two_vector_3d(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2);

bool is_in_range_of_two_points(point_3d &p, point_3d &p1, point_3d &p2);

bool equal_float(double v1, double v2, double th = 1e-6);

float radian2degree(float radian);

float degree2radian(float angle);

void project_points_onto_line(std::vector<point_3d>& points, line_func_3d & line, std::vector<point_3d>& projected_points);

/*
	Return the radian between two 2d vectors that represented by 3d vectors,
	the radian is antclockwise from v1 to v2
	Note: the third element should be zero, like v1(1,2,0) and v2(3,6,0)
*/
void radian_two_vector_2d_022PI(const Eigen::Vector3f & v1, const Eigen::Vector3f &v2, float & radian);

// get rotation transformation, from v1 to v2
void rotation_transformation(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2, Eigen::Matrix3f & t);

void subset_of_point_cloud(std::vector<size_t> &index_vec, std::vector<point_3d> & point_cloud, std::vector<point_3d> & subset);

#endif // CLOUD_GEOMETRY_H
