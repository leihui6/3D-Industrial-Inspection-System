#ifndef FPFH_REGISTRATION
#define FPFH_REGISTRATION

#include "pcl_header.h"

class fpfh_registration
{
public:
	fpfh_registration();

	~fpfh_registration();

	int do_reg(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4f & final_matrix);

private:

	int extractDescriptors(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

	int findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences);

	int filterCorrespondences();

	int determineInitialTransformation();

private:

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_target_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_source_cloud;

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_source_features;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_target_features;

	std::vector<int> m_source2target;
	std::vector<int> m_target2source;

	pcl::CorrespondencesPtr m_correspondences;

	Eigen::Matrix4f m_transformation_matrix;
};


#endif //FPFH_REGISTRATION
