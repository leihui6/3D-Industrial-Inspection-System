#ifndef COMMON_USE
#define COMMON_USE

#include <Eigen/Dense>

#include <filesystem>
#include <map>

#include "cloud_viewer.h"

#include "cloud_io.h"

#include "cloud_measurement.h"

struct LocalFile
{
	LocalFile() : m_fileobject(), m_filename() {}
	std::fstream m_fileobject;
	std::string m_filename;
};

class cloud_viewer;

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
extern void display_point_cloud_from_transformation_vec(cloud_viewer & cv, std::vector<point_3d> & reading_point_cloud, std::vector<Eigen::Matrix4f> &transformation_vec);

// read points from file that containing many points and spilted by '#'
extern void read_points(std::map<std::string, std::vector<point_3d>> & points_map, const std::string & file_name);

// export marked points which is a map
extern void export_marked_points(std::map <std::string, std::vector<point_3d>> & marked_points, const std::string & export_file_name);

extern void export_measured_data(std::multimap<std::string, std::string> & measurement_pairs_map, std::vector<measurement_content> &mc_vec, const std::string & output_file_name);

extern void transform_marked_points(std::map <std::string, std::vector<point_3d>> & marked_points, Eigen::Matrix4f & m);

extern void string_split(std::string &str, char character, std::vector<std::string> & res);

// read data from file in which all information should follow rules as folloing:
// 1) all information in one line were divided by ":
// 2) you can use # to comment but only be one line that different from other normal information
extern void read_file_as_map(const std::string & file_name, std::map<std::string, std::string> & str_flt_map);
extern void read_file_as_map(const std::string & file_name, std::multimap<std::string, std::string> & str_flt_map);
extern void read_file_as_map(const std::string & file_name, std::multimap<std::string, std::string> & str_flt_map, std::multimap<std::string, std::string> & reference_map);

// convert string represeting te vector4 to a float-vec4
extern osg::Vec4 str_to_vec4(const std::string & s);

#endif // COMMON_USE
