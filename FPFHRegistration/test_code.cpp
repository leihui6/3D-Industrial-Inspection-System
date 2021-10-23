#include "fpfh_registration.h"


bool load_txt_as_pcd(const std::string filename, pcl::PointCloud<pcl::PointXYZ>& pcl_points);

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	load_txt_as_pcd("data/medical_blade_standard.txt", *source);
	load_txt_as_pcd("data/medical_blade_01_downsample.txt", *cloud);

	fpfh_registration fpfh_reg;
	Eigen::Matrix4f res_m;
	fpfh_reg.do_reg(cloud, source, res_m);

	return 0;
}

bool load_txt_as_pcd(const std::string filename, pcl::PointCloud<pcl::PointXYZ>& pcl_points)
{
	ifstream ifile(filename);

	if (!ifile.is_open()) return false;

	std::string str;

	while (getline(ifile, str))
	{
		pcl::PointXYZ p;

		std::stringstream s(str);

		int i = 0;
		float tmp = 0.0, value[3] = { 0,0,0 };
		while (s >> tmp)
		{
			value[i] = tmp;
			i++;
			if (i == 3)
			{
				//std::cout <<"only support reading (x,y,z) points"<<std::endl;
				break;
			}
		}

		p.x = value[0];
		p.y = value[1];
		p.z = value[2];
		pcl_points.push_back(p);
	}
	return true;
}

/*
fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(20);
	est_normal.compute(*point_normal);

	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8); //÷∏∂®4∫Àº∆À„
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}
void findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences)
{
	std::cout << "findCorrespondences" << std::endl;
	correspondences.resize(source->size());

	// Use a KdTree to search for the nearest matches in feature space
	pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
	descriptor_kdtree.setInputCloud(target);

	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = 1;
	std::vector<int> k_indices(k);
	std::vector<float> k_squared_distances(k);
	for (std::size_t i = 0; i < source->size(); ++i)
	{
		descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_squared_distances);
		correspondences[i] = k_indices[0];
	}
}

void filterCorrespondences()
{
	std::cout << "correspondence rejection..." << std::flush;

	std::vector<std::pair<unsigned, unsigned> > correspondences;

	for (unsigned cIdx = 0; cIdx < source2target_index.size(); ++cIdx)
	{
		if (static_cast<unsigned int>(target2source_index[source2target_index[cIdx]]) == cIdx)
		{

			correspondences.push_back(std::make_pair(cIdx, source2target_index[cIdx]));
		}
	}

	correspondences_->resize(correspondences.size());
	for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
	{
		(*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
		(*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
	rejector.setInputSource(source_keypoints_);
	rejector.setInputTarget(target_keypoints_);
	rejector.setInputCorrespondences(correspondences_);
	rejector.getCorrespondences(*correspondences_);
}
*/