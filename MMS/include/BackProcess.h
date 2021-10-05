#ifndef BACKPROCESS_H
#define BACKPROCESS_H

#include "BackProcessCom.h"

#include "common_use.h"
#include "cloud_io.h"
#include "cloud_search.h"
#include "cloud_registration.h"
#include "cloud_measurement.h"

class BackProcess : public BackProcessCom
{
public:
	BackProcess();

	~BackProcess();

	virtual int initial_parameter(const std::string & config_filename);

	virtual int registration();

	virtual int searching();

	virtual int measurement();

private:

	// Registration
	// filename
	std::string
		m_reading_point_cloud_filename, m_reference_point_cloud_filename,
		m_icp_configuration_filename, m_coarse_configuration;

	// standard and scaned point cloud object
	std::vector<point_3d> m_reading_point_cloud, m_reference_point_cloud;
	// matrix generated during registration
	Eigen::Matrix4f m_coarse_ret_mat, m_fine_ret_mat, m_final_mat;


	// Searching
	std::vector<point_3d> m_transformed_point_cloud;
	std::map<std::string, std::vector<point_3d>> m_marked_points_map, m_new_mark_points_map;


	// Measurement
	std::multimap<std::string, std::string> m_measurement_pairs_map, m_reference_map;
	std::string m_measurement_pairs;


	// Common object
	std::string m_output_folder;

};
#endif // BACKPROCESS_H



