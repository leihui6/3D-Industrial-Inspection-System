#include "cloud_evaluation.h"


cloud_evaluation::cloud_evaluation()
{
}


cloud_evaluation::~cloud_evaluation()
{
}


int cloud_evaluation::mean_distance_point_clouds(std::vector<point_3d>& pc1, std::vector<point_3d>& pc2, evaluation_results & er)
{
	if (pc1.empty() || pc2.empty()) return 1;

	kd_tree pc_tree;
	std::vector<point_3d> correspondences;

	std::vector<point_3d> * searched_points = nullptr;

	if (pc1.size() < pc2.size())
	{
		pc_tree.load_points(pc1);
		pc_tree.search_points_correspondence(pc2, correspondences);
		searched_points = &pc2;
	}
	else
	{
		pc_tree.load_points(pc2);
		pc_tree.search_points_correspondence(pc1, correspondences);
		searched_points = &pc1;
	}

	float total_val = 0.0, sqrt_total_val = 0.0;

	if (searched_points->size() != correspondences.size())
	{
		std::cerr << "pc_tree number != correspondences" << std::endl;
		return 2;
	}

	const std::vector<point_3d> & pc = *searched_points;
	std::cout << "point size:" << pc.size() << "  " << "searched point(for evaluation):" << correspondences.size() << std::endl;

	for (size_t i = 0; i < pc.size(); ++i)
	{
		float this_distance = 0.0;
		distance_point_to_point(pc[i], correspondences[i], this_distance);
		total_val += this_distance;
	}

	//er.total_val = total_val;
	//er.mean_val = (total_val / searched_points->size());
	er.rms_val = sqrtf(total_val / searched_points->size());

	return 0;
}

int cloud_evaluation::mean_distance_points(std::vector<point_3d>& pc1, std::vector<point_3d>& pc2, evaluation_results & er)
{
	if (pc1.size() != pc2.size()) return 1;

	if (pc1.empty() || pc2.empty()) return 2;

	size_t total_size = pc1.size();
	double total_dis = 0.0;

	for (size_t i = 0; i < total_size; ++i)
	{
		float this_dis = 0.0;
		distance_point_to_point(pc1[i], pc2[i], this_dis);
		total_dis += this_dis;
	}
	//er.total_val = total_dis;
	//er.mean_val = (total_dis / total_size);
	er.rms_val = sqrtf(total_dis / total_size);

	return 0;
}

int cloud_evaluation::export_result(const std::string filename, evaluation_results & er)
{
	LocalFile local_file;

	if (!check_file(filename, std::ios::out | std::ios::app, local_file)) return 1;

	std::fstream & ofile = *local_file.m_fileobject;

	std::string ctime = current_date_time(true, true);

	ofile << "RootMeamSquare:" << er.rms_val << std::endl;

	ofile.close();

	return 0;
}

