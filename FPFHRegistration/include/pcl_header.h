#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h>// OMP containing fpfh accelerated computing (multi-core parallel computing)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>//feature error correspondence removal
#include <pcl/registration/correspondence_rejection_sample_consensus.h>//Random Sampling Consistency Removal
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>