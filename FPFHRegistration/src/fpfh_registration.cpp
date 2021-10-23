#include "fpfh_registration.h"


fpfh_registration::fpfh_registration()
	:m_source_features(new pcl::PointCloud<pcl::FPFHSignature33>),
	m_target_features(new pcl::PointCloud<pcl::FPFHSignature33>),
	m_correspondences(new pcl::Correspondences)
{
}


fpfh_registration::~fpfh_registration()
{
}

int fpfh_registration::do_reg(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4f & final_matrix)
{
	m_source_cloud = source_cloud;
	m_target_cloud = target_cloud;

	extractDescriptors(source_cloud, m_source_features);
	extractDescriptors(target_cloud, m_target_features);

	findCorrespondences(m_source_features, m_target_features, m_source2target);
	findCorrespondences(m_target_features, m_source_features, m_target2source);

	filterCorrespondences();

	determineInitialTransformation();

	return 0;
}

int fpfh_registration::extractDescriptors(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
	std::cout << "Computing feature points" << std::endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	pcl::PointCloud<pcl::Normal> normals;
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(20);
	norm_est.setInputCloud(input);
	norm_est.compute(normals);

	// Estimate the FPFH features for the source cloud
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setNumberOfThreads(8);
	fpfh_est.setSearchMethod(tree);
	fpfh_est.setKSearch(40);
	fpfh_est.setInputCloud(input);
	fpfh_est.setInputNormals(normals.makeShared());
	fpfh_est.compute(*features);

	return 0;
}

int fpfh_registration::findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences)
{
	std::cout << "correspondence assignment..." << std::flush;
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
		if (0 != descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_squared_distances))
		{
			correspondences[i] = k_indices[0];
		}
	}
	std::cout << "OK" << std::endl;
	return 0;
}

int fpfh_registration::filterCorrespondences()
{
	std::cout << "correspondence rejection..." << std::flush;

	std::vector<std::pair<unsigned, unsigned> > correspondences;

	for (unsigned cIdx = 0; cIdx < m_source2target.size(); ++cIdx)
	{
		if (static_cast<unsigned int>(m_target2source[m_source2target[cIdx]]) == cIdx)
		{
			correspondences.push_back(std::make_pair(cIdx, m_source2target[cIdx]));
		}
	}

	m_correspondences->resize(correspondences.size());
	for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
	{
		(*m_correspondences)[cIdx].index_query = correspondences[cIdx].first;
		(*m_correspondences)[cIdx].index_match = correspondences[cIdx].second;
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
	
	rejector.setInputSource(m_source_cloud);
	rejector.setInputTarget(m_target_cloud);
	
	rejector.setInlierThreshold(50.0);

	rejector.setInputCorrespondences(m_correspondences);
	rejector.getCorrespondences(*m_correspondences);
	std::cout << "OK" << std::endl;
	return 0;
}

int fpfh_registration::determineInitialTransformation()
{
	std::cout << "initial alignment..." << std::flush;

	pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr transformation_estimation
	(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>);

	transformation_estimation->estimateRigidTransformation(*m_source_cloud, *m_target_cloud, *m_correspondences, m_transformation_matrix);

	std::cout << m_correspondences->size() << std::endl;
	std::cout << m_transformation_matrix << std::endl;
	return 0;
}
