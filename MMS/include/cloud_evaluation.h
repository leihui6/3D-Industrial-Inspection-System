#ifndef CLOUD_EVALUATION_H
#define CLOUD_EVALUATION_H

// KD-Tree
#include "cloud_search.h"
#include "common_use.h"

struct evaluation_results
{
	evaluation_results() :
		mean_val(0.0), total_val(0.0), rms_val(0.0) {}

	double mean_val;

	double total_val;

	// RMS Root Mean Square
	double rms_val;
};

class cloud_evaluation
{
public:
	cloud_evaluation();

	~cloud_evaluation();

	// using kd-tree calculate the correpondences
	int mean_distance_point_clouds(std::vector<point_3d>& pc1, std::vector<point_3d>& pc2, evaluation_results & er);

	// calculate the correspondences point_1[0] -> point_2[1]
	int mean_distance_points(std::vector<point_3d>& pc1, std::vector<point_3d>& pc2, evaluation_results & er);

	int export_result(const std::string filename, evaluation_results& er);
};

#endif // CLOUD_EVALUATION_H

