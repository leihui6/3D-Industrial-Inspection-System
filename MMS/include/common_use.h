#ifndef COMMON_USE
#define COMMON_USE

#include <Eigen/Dense>

#include <random>
#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <map>

//#include "cloud_viewer.h"
//#include "cloud_io.h"
//#include "cloud_measurement.h"

// all basic geometrical elements

// base class
struct basic_shape {};

struct point_3d : public basic_shape
{
	point_3d(const point_3d & p);

	point_3d();

	point_3d(float x, float y, float z);

	void set_xyz(float x, float y, float z);

	void set_nxyz(float nx, float ny, float nz);

	void set_rgb(float r, float g, float b);

	Eigen::Vector3f get_vector3f();

	// execute transformation to the other point
	void do_transform(const Eigen::Matrix4f & m, point_3d & p);

	// execute transformation to itself
	void do_transform(const Eigen::Matrix4f & m);

	friend std::ostream & operator << (std::ostream & os, const point_3d & p);

	point_3d & operator = (const point_3d & p);
	point_3d operator + (const point_3d & p);
	point_3d operator - (const point_3d & p);
	point_3d operator / (const float num);
	bool operator == (const point_3d & rp);

	float x, y, z;

	float nx, ny, nz;

	float r, g, b;
};

typedef point_3d point_3d;

struct line_func_3d : public basic_shape
{
	line_func_3d();

	line_func_3d(const line_func_3d & lf);

	void set_xyz(float x, float y, float z);

	void set_nml(float n, float m, float l);

	point_3d get_origin_point_3d();

	point_3d get_direction_point_3d();

	line_func_3d & operator = (const line_func_3d & p);

	//float x, y, z;
	Eigen::Vector3f origin;

	//float n, m, l;
	Eigen::Vector3f direction;
};

struct plane_func_3d : public basic_shape
{
	plane_func_3d();

	plane_func_3d(float _a, float _b, float _c, float _d);

	void set_abcd(float _a, float _b, float _c, float _d);

	void set_abcd(float _a, float _b, float _c, point_3d & p);

	void reverse();

	Eigen::Vector3f direction()
	{
		return Eigen::Vector3f(this->a, this->b, this->c);
	}

	float a, b, c, d;
};

struct cylinder_func : public basic_shape
{
	cylinder_func();

	// the center point and its vector
	line_func_3d axis;

	float radius;

	float height;
};

// as nanoflann required
struct point_cloud
{
	point_cloud() {}

	point_cloud(std::vector<point_3d> & _pts)
	{
		this->pts = _pts;
	}

	std::vector<point_3d>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const
	{
		return pts.size();
	}


	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0)
		{
			return pts[idx].x;
		}
		else if (dim == 1)
		{
			return pts[idx].y;
		}
		else
		{
			return pts[idx].z;
		}
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const
	{
		return false;
	}

	void load_points(std::vector<point_3d> & points);
};

// used in measurement
struct point_shape {
	point_shape() :
		points(), func(std::make_shared<basic_shape>()), shape_property() {}

	std::vector<point_3d> points;
	std::shared_ptr<basic_shape> func;
	std::vector<Eigen::Vector3f> shape_property;
};

// file management
struct LocalFile
{
	LocalFile() : m_fileobject(std::make_shared<std::fstream>()), m_filename() {}
	std::shared_ptr<std::fstream> m_fileobject;
	std::string m_filename;
};

//class cloud_viewer;

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

/*
LocalFile local_file;
if (!check_file(file_name, std::ios::in, local_file)) return;
std::fstream & ifile = local_file.m_fileobject;
*/
extern bool check_file(const std::string filename, std::ios_base::openmode mode, LocalFile & local_file);

// write one matrix(4x4) to file
extern void save_matrix(Eigen::Matrix4f & matrix, const std::string & file_name);

// read one or more matrix(4x4) from file
extern void read_matrix(const std::string & file_name, std::vector<Eigen::Matrix4f> & m_v);

// cut '/' and postfix context of file name string 
extern std::string file_name_without_postfix(std::string & file_name);

// check if the file exists
extern bool is_exist(const std::string & file_name);

// display point cloud in osg repeatly with mroe matrices
//extern void display_point_cloud_from_transformation_vec(cloud_viewer & cv, std::vector<point_3d> & reading_point_cloud, std::vector<Eigen::Matrix4f> &transformation_vec);

// read points from file that containing many points and spilted by '#'
extern void read_points(std::map<std::string, std::vector<point_3d>> & points_map, const std::string & file_name);

// read marked points which is the result of the labeling step
extern void read_marked_points(std::map<std::string, point_shape> & point_shape_map, const std::string & filename);

// export marked points which is a map
extern void export_marked_points(std::map<std::string, point_shape>& marked_points, const std::string & export_file_name);

extern void transform_marked_points(std::map <std::string, std::vector<point_3d>> & marked_points, Eigen::Matrix4f & m);

extern void string_split(std::string &str, char character, std::vector<std::string> & res);

// read data from file in which all information should follow rules as folloing:
// 1) all information in one line were divided by ":
// 2) you can use # to comment but only be one line that different from other normal information
extern void read_file_as_map(const std::string & file_name, std::map<std::string, std::string> & str_flt_map);
extern void read_file_as_map(const std::string & file_name, std::multimap<std::string, std::string> & str_flt_map);

// convert string represeting te vector4 to a float-vec4
//extern osg::Vec4 str_to_vec4(const std::string & s);

extern std::string current_date_time(bool need_date = false, bool need_time = true);

extern int obtain_random(int range_min, int range_max);

extern void obtain_random(int range_min, int range_max, std::vector<int> &random_vec);

#endif // COMMON_USE
