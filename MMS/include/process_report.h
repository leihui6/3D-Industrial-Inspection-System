#ifndef PROCESS_REPORT_H
#define PROCESS_REPORT_H

#include "common_use.h"

struct report_data
{
	std::string reading_filename, reference_filename;

	std::vector<point_3d> *reading_data, *reference_data;

	float registration_coarse_time, registration_fine_time;

	// coarse, coarse->fine, fine*coarse
	std::vector<Eigen::Matrix4f> registration_matrix;

	float RMS_registration, RMS_searching;
};

class process_report
{
public:
	process_report();

	~process_report();

	int export_report(const std::string & filename, const report_data & rd);

private:
	std::string matrix_to_str(const Eigen::Matrix4f & m);
};
#endif
