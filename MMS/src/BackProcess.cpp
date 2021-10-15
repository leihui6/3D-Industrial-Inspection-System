#include "BackProcess.h"



BackProcess::BackProcess()
	:m_coarse_ret_mat(Eigen::Matrix4f::Identity()),
	m_fine_ret_mat(Eigen::Matrix4f::Identity()),
	m_final_mat(Eigen::Matrix4f::Identity())
{

}


BackProcess::~BackProcess()
{

}

int BackProcess::initial_parameter(const std::string & config_filename)
{
	std::map<std::string, std::string> parameters;
	read_file_as_map(config_filename, parameters);

	// point cloud files
	m_reading_point_cloud_filename = parameters["reading_data"];
	m_reference_point_cloud_filename = parameters["reference_data"];

	// measurement files
	m_measurement_pairs = parameters["measurement_pairs"];

	// registration files
	m_icp_configuration_filename = parameters["icp_configuration"];
m_coarse_configuration = parameters["4pcs_configuration"];

m_output_folder = parameters["output_folder"];

// load data
load_point_cloud_txt(m_reading_point_cloud_filename, m_reading_point_cloud, false);

load_point_cloud_txt(m_reference_point_cloud_filename, m_reference_point_cloud, false);

read_points(m_marked_points_map, m_output_folder + "/marked_points.txt");

read_file_as_map(m_measurement_pairs, m_measurement_pairs_map, m_reference_map);

return 0;
}

int BackProcess::registration()
{
	cloud_registration cloud_registration;

	// Coarse registration
	// final transformation for fine(icp) registration
	//Eigen::Matrix4f coarse_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	std::vector<point_3d> coarse_transformed_point_cloud;

	// implement 4pcs algorithm that align reading point cloud to reference point cloud
	cloud_registration.coarse_registration(m_reading_point_cloud, m_reference_point_cloud, m_coarse_ret_mat, m_coarse_configuration);

	// matrix transforming reading point cloud to reference point cloud
	//std::cout << "coarse registration matrix is:\n" << coarse_ret_mat << "\n";

	// save matrix to file
	save_matrix(m_coarse_ret_mat, m_output_folder + "/coarse_matrix.txt");

	transform_points(m_reading_point_cloud, m_coarse_ret_mat, coarse_transformed_point_cloud);

	// final transformation for fine(icp) registration
	//Eigen::Matrix4f fine_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	//std::vector<point_3d> fine_transformed_point_cloud;

	// implement iterative closest point algorithm that align reading point cloud to reference point cloud
	cloud_registration.fine_registration(coarse_transformed_point_cloud, m_reference_point_cloud, m_icp_configuration_filename, m_fine_ret_mat);

	// matrix transforming reading point cloud to reference point cloud
	//std::cout << "fine registration matrix is:\n" << fine_ret_mat << "\n";

	// save matrix to file
	save_matrix(m_fine_ret_mat, m_output_folder + "/fine_matrix.txt");

	m_final_mat = m_fine_ret_mat * m_coarse_ret_mat;

	save_matrix(m_final_mat, m_output_folder + "/final_matrix.txt");

	return 0;
}

int BackProcess::searching()
{
	transform_points(m_reading_point_cloud, m_final_mat, m_transformed_point_cloud);

	kd_tree kt(m_transformed_point_cloud);

	std::map<std::string, std::vector<point_3d>>::iterator it;

	for (it = m_marked_points_map.begin(); it != m_marked_points_map.end(); it++)
	{
		std::vector<point_3d> correspondence;

		kt.search_points_correspondence(it->second, correspondence);

		//it->second = correspondence;
		m_new_mark_points_map.insert(std::pair< std::string, std::vector<point_3d>>(it->first, correspondence));
	}

	export_marked_points(m_new_mark_points_map, m_output_folder + "/marked_points_searched.txt");

	return 0;
}

int BackProcess::measurement()
{
	cloud_measurement cm(m_transformed_point_cloud);

	std::vector<measurement_content> mc_vec;

	cm.measure(m_measurement_pairs_map, m_new_mark_points_map, m_reference_map, mc_vec);

	transform_measured_results(mc_vec);

	export_measured_data(m_measurement_pairs_map, mc_vec, m_output_folder + "/measurement_results.txt");

	return 0;
}


void BackProcess::transform_measured_results(std::vector<measurement_content> & mc_vec)
{
	for (auto & pv : mc_vec)
		for (auto &p : pv.drawable_points)
			p.do_transform(m_final_mat.inverse());
}