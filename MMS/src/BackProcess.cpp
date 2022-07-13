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
	read_file_as_map(config_filename, m_parameters);

	// point cloud files
	m_reading_point_cloud_filename =
		m_parameters["input_folder"] + m_parameters["reading_data"];
	m_reference_point_cloud_filename =
		m_parameters["input_folder"] + m_parameters["reference_data"];

	// measurement files
	m_measurement_requirement_filename =
		m_parameters["input_folder"] + m_parameters["measurement_requirement"];

	// registration files
	m_icp_configuration_filename =
		m_parameters["input_folder"] + m_parameters["icp_configuration"];
	m_coarse_configuration_filename =
		m_parameters["input_folder"] + m_parameters["4pcs_configuration"];

	//m_output_folder = m_parameters["output_folder"];

	load_data();

	return 0;
}

void BackProcess::load_data()
{
	clock_t beg_t = clock();
	// load data
	load_point_cloud_txt(m_reading_point_cloud_filename, m_reading_point_cloud_ori, false);
	load_point_cloud_txt(m_reference_point_cloud_filename, m_reference_point_cloud_ori, false);
	m_rd.reading_data_ori = &m_reading_point_cloud_ori;
	m_rd.reference_data_ori = &m_reference_point_cloud_ori;

	// filter point cloud
	cloud_processing cp;
	m_reading_point_cloud = m_reading_point_cloud_ori;
	m_reference_point_cloud = m_reference_point_cloud_ori;
	cp.filter_simplify_grid(m_reading_point_cloud, 0.5);
	cp.filter_simplify_grid(m_reference_point_cloud, 0.5);

	m_rd.data_load_time = double(clock() - beg_t) / CLOCKS_PER_SEC;
}

int BackProcess::registration()
{
	if (m_reading_point_cloud.empty() || m_reference_point_cloud.empty()) return 1;

	cloud_registration cloud_registration;

	clock_t begin = clock();

	// Coarse registration
	// final transformation for fine(icp) registration
	//Eigen::Matrix4f coarse_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	//std::vector<point_3d> coarse_transformed_point_cloud;

	// implement 4pcs algorithm that align reading point cloud to reference point cloud
	cloud_registration.coarse_registration(m_reading_point_cloud, m_reference_point_cloud, m_coarse_ret_mat, m_coarse_configuration_filename);

	// matrix transforming reading point cloud to reference point cloud
	//std::cout << "coarse registration matrix is:\n" << coarse_ret_mat << "\n";

	m_rd.registration_coarse_time = double((clock() - begin)) / CLOCKS_PER_SEC;
	begin = clock();
	//save_matrix(m_coarse_ret_mat, m_parameters["output_folder"] + m_parameters["coarse_registration_result"]);

	transform_points(m_reading_point_cloud, m_coarse_ret_mat, m_coarse_transformed_point_cloud);

	// final transformation for fine(icp) registration
	//Eigen::Matrix4f fine_ret_mat;

	// transformed point cloud from reading point cloud, it will be saved locally
	//std::vector<point_3d> fine_transformed_point_cloud;

	// implement iterative closest point algorithm that align reading point cloud to reference point cloud
	cloud_registration.fine_registration(m_coarse_transformed_point_cloud, m_reference_point_cloud, m_icp_configuration_filename, m_fine_ret_mat);

	// matrix transforming reading point cloud to reference point cloud
	//std::cout << "fine registration matrix is:\n" << fine_ret_mat << "\n";

	// save matrix to file
	//save_matrix(m_fine_ret_mat, m_parameters["output_folder"] + m_parameters["fine_registration_result"]);
	m_rd.registration_fine_time = double((clock() - begin)) / CLOCKS_PER_SEC;

	m_final_mat = m_fine_ret_mat * m_coarse_ret_mat;

	//save_matrix(m_final_mat, m_parameters["output_folder"] + m_parameters["final_registration_result"]);

	return 0;
}

int BackProcess::searching()
{
	clock_t beg_t = clock();

	read_marked_points(m_marked_points_map, m_parameters["output_folder"] + m_parameters["marked_points_result"]);

	if (m_marked_points_map.empty()) return 1;

	transform_points(m_reading_point_cloud, m_final_mat, m_transformed_point_cloud);

	kd_tree kt(m_transformed_point_cloud);

	std::map<std::string, point_shape>::iterator it;

	for (it = m_marked_points_map.begin(); it != m_marked_points_map.end(); it++)
	{
		//skip the reference points
		if (it->first.find("reference") != std::string::npos)
		{
			m_searched_mark_points_map.insert(*it);
			continue;
		}

		point_shape ps;

		kt.search_points_correspondence(it->second.points, ps.points);
		// no changing
		ps.shape_property = it->second.shape_property;

		//it->second = correspondence;
		m_searched_mark_points_map.insert(std::pair< std::string, point_shape>(it->first, ps));

		//ps.clear();
	}

	export_marked_points(m_searched_mark_points_map, m_parameters["output_folder"] + m_parameters["marked_points_searched_result"]);

	m_rd.searching_time = double(clock() - beg_t) / CLOCKS_PER_SEC;

	return 0;
}

int BackProcess::measurement()
{
	clock_t beg_t = clock();

	cloud_measurement cm(m_transformed_point_cloud, m_searched_mark_points_map);
	
	LocalFile local_file;

	if (!check_file(m_measurement_requirement_filename, std::ios::in, local_file)) return 1;;

	std::fstream & ifile = *local_file.m_fileobject;

	while (!ifile.eof())
	{
		std::string measurment_pair_filename;

		std::getline(ifile, measurment_pair_filename);
		
		if (measurment_pair_filename.find("#") != std::string::npos)
		{
			continue;
		}

		measurment_pair_filename =
			m_parameters["input_folder"] + measurment_pair_filename;
		
		std::cout << "processing " << measurment_pair_filename << std::endl;

		size_t pair_num = 
			cm.read_pair_file(measurment_pair_filename);

		cm.measure();

		cm.post_process(m_final_mat);

		cm.export_measured_data(m_parameters["output_folder"] + m_parameters["measurement_result"]);
	}
	
	m_rd.measurement_time = double(clock() - beg_t) / CLOCKS_PER_SEC;

	return 0;
}

int BackProcess::evaluation()
{
	cloud_evaluation ce;

	// evaluate the registration result
	std::vector<point_3d> reading_to_standard_point_cloud;

	transform_points(m_reading_point_cloud, m_final_mat, reading_to_standard_point_cloud);

	int evaluation_results =
		ce.mean_distance_point_clouds(reading_to_standard_point_cloud, m_reference_point_cloud, m_registration_er);
	
	//ce.export_result(m_parameters["output_folder"] + m_parameters["evaluation_result"], m_registration_er);


	// evaluate the searching result
	std::vector<point_3d> original_points, new_points;
	for (auto & item : m_marked_points_map)
	{
		original_points.insert(original_points.end(), item.second.points.begin(), item.second.points.end());

		new_points.insert(new_points.end(), 
			m_searched_mark_points_map[item.first].points.begin(), 
			m_searched_mark_points_map[item.first].points.end());
	}
	evaluation_results =
		ce.mean_distance_points(original_points, new_points, m_searching_er);

	//ce.export_result(m_parameters["output_folder"] + m_parameters["evaluation_result"], m_searching_er);

	export_report();

	return 0;
}

void BackProcess::export_report()
{
	m_rd.reading_filename = m_reading_point_cloud_filename;
	m_rd.reference_filename = m_reference_point_cloud_filename;

	m_rd.reading_data = &m_reading_point_cloud;
	m_rd.reference_data = &m_reference_point_cloud;

	// registration elapsed time was add in registration function
	// ... 

	m_rd.registration_matrix.push_back(m_coarse_ret_mat);
	m_rd.registration_matrix.push_back(m_fine_ret_mat);
	m_rd.registration_matrix.push_back(m_final_mat);

	m_rd.RMS_registration = m_registration_er.rms_val;
	m_rd.RMS_searching = m_searching_er.rms_val;

	m_rd.total_time =
		m_rd.data_load_time + m_rd.registration_coarse_time +
		m_rd.registration_fine_time + m_rd.searching_time + m_rd.measurement_time;

	process_report pr;
	pr.export_report(m_parameters["output_folder"] + m_parameters["export_report"], m_rd);
}

