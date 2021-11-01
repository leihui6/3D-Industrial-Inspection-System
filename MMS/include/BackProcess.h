#ifndef BACKPROCESS_H
#define BACKPROCESS_H

#include "BackProcessCom.h"

#include "common_use.h"
#include "cloud_io.h"
#include "cloud_search.h"
#include "cloud_registration.h"
#include "cloud_processing.h"
#include "cloud_measurement.h"
#include "cloud_evaluation.h"
#include "process_report.h"

class BackProcess : public BackProcessCom
{
public:
	BackProcess();

	~BackProcess();

	virtual int initial_parameter(const std::string & config_filename);

	virtual int registration();

	virtual int searching();

	virtual int measurement();

	virtual int evaluation();

private:

	void load_data();

	void export_report();

private:

	// Registration
	// filename
	std::string
		m_reading_point_cloud_filename, m_reference_point_cloud_filename,
		m_icp_configuration_filename, m_coarse_configuration_filename;

	// standard and scaned point cloud object
	std::vector<point_3d>
		m_reading_point_cloud_ori, m_reading_point_cloud, // original and filtered data
		m_reference_point_cloud_ori, m_reference_point_cloud, // standard and filtered data
		m_coarse_transformed_point_cloud; // coarse -> fine

	// matrix generated during registration
	Eigen::Matrix4f m_coarse_ret_mat, m_fine_ret_mat, m_final_mat;


	// Searching
	std::vector<point_3d> m_transformed_point_cloud;
	std::map<std::string, point_shape>
		m_marked_points_map, m_searched_mark_points_map;


	// Measurement
	std::vector<std::string> m_pair_order;
	std::string m_measurement_pairs_filename;


	// Common object
	report_data m_rd;
	evaluation_results m_registration_er, m_searching_er;
	std::map<std::string, std::string> m_parameters;
};
#endif // BACKPROCESS_H



