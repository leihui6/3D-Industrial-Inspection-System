#include "process_report.h"



process_report::process_report()
{
}


process_report::~process_report()
{
}

int process_report::export_report(const std::string & filename, const report_data & rd)
{
	LocalFile local_file;

	if (!check_file(filename, std::ios::out | std::ios::app, local_file)) return 1;

	std::fstream & ofile = local_file.m_fileobject;

	std::vector<std::string> content;

	// add date
	content.push_back(">Date\n");
	content.push_back("\t");
	content.push_back(current_date_time(true, true) + "\n");

	// add filename
	content.push_back(">Data\n");
	content.push_back("\t");
	content.push_back(rd.reading_filename + " " + std::to_string(rd.reading_data_ori->size()) + "->" + std::to_string(rd.reading_data->size()) + " -");
	content.push_back(std::to_string(1 - double(rd.reading_data->size()) / rd.reading_data_ori->size()).substr(0, 6) + "%\n");

	content.push_back("\t");
	content.push_back(rd.reference_filename + " " + std::to_string(rd.reference_data_ori->size()) + "->" + std::to_string(rd.reference_data->size()) + " -");
	content.push_back(std::to_string(1 - double(rd.reference_data->size()) / rd.reference_data_ori->size()).substr(0, 6) + "%\n");

	content.push_back(">Transformation 4x4 (Coarse, Coarse->Fine, and Final Matrix)\n");
	for (auto & m : rd.registration_matrix) content.push_back(matrix_to_str(m) + "\n");

	content.push_back(">Time Elapsed\n");
	content.push_back(
		"\tData Loading     :\t" + std::to_string(rd.data_load_time).substr(0, 6) + "s\n" +
		"\tRegistration(1/2):\t" + std::to_string(rd.registration_coarse_time).substr(0, 6) + "s\n" +
		"\tRegistration(2/2):\t" + std::to_string(rd.registration_fine_time).substr(0, 6) + "s\n" +
		"\tSearching        :\t" + std::to_string(rd.searching_time).substr(0, 6) + "s\n" +
		"\tMeasurement      :\t" + std::to_string(rd.measurement_time).substr(0, 6) + "s\n" +
		"\tTotal Time       :\t" + std::to_string(rd.total_time).substr(0, 6) + "s\n");

	content.push_back(">Root Mean Square(Registration and Searching)\n");
	content.push_back("\t");
	content.push_back(
		std::to_string(rd.RMS_registration).substr(0, 6) + " " +
		std::to_string(rd.RMS_searching).substr(0, 6) + "\n");

	content.push_back(">End\n");

	for (auto & c : content) ofile << c;
	ofile.close();
	return 0;
}

std::string process_report::matrix_to_str(const Eigen::Matrix4f & m)
{
	std::stringstream ss;
	ss << m;
	return ss.str();
}
