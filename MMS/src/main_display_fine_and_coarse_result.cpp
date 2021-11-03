/*
*	The purpose of this program is to visualize the process of coarse-to-fine stage.
*
*	Author: leihui.li#outlook.com
*	Date: 06/06/2020
*/

//#define DISPLAY
#ifdef DISPLAY

#include "cloud_io.h"
#include "cloud_viewer.h"
#include "common_use.h"

void display_point_cloud_from_transformation_vec(cloud_viewer & cv, std::vector<point_3d>& reading_point_cloud, std::vector<Eigen::Matrix4f>& transformation_vec)
{
	// transformation_vec[1] is a identify matrix
	std::swap(transformation_vec[0], transformation_vec[1]);

	std::vector<std::vector<point_3d>> transformed_points;

	for (size_t i = 2; i < transformation_vec.size(); ++i)
	{
		transformation_vec[i] = transformation_vec[i] * transformation_vec[1];
	}

	for (size_t i = 0; i < transformation_vec.size(); ++i)
	{
		std::vector<point_3d> t_points;

		transform_points(reading_point_cloud, transformation_vec[i], t_points);

		transformed_points.push_back(t_points);
	}

	size_t i = 0;

	while (true)
	{
		cv.update_reading_point_cloud(transformed_points[i], 0, 255, 0, 4.0);

		std::this_thread::sleep_for(std::chrono::milliseconds(400));

		if (i + 1 == transformation_vec.size())
		{
			i = 0;

			continue;
		}

		i++;
	}
}

int main(int argc, char *argv[])
{
	if (argc < 2) return 1;

	std::string configuration_file_name;
	configuration_file_name = std::string(argv[1]);
	std::map<std::string, std::string> parameters;
	read_file_as_map(configuration_file_name, parameters);

	std::string
		reading_point_cloud_file_name,
		reference_point_cloud_file_name,
		output_folder,
		fine_registration_filename,
		coarse_registration_filename;

	reading_point_cloud_file_name	= parameters["reading_data"];
	reference_point_cloud_file_name = parameters["reference_data"];
	output_folder = parameters["output_folder"];
	coarse_registration_filename = parameters["coarse_registration_filename"];
	fine_registration_filename = parameters["fine_registration_filename"];

	std::vector<point_3d> reading_point_cloud, reference_point_cloud;

	// read reading point cloud from local file
	load_point_cloud_txt(reading_point_cloud_file_name, reading_point_cloud, false);

	// read reference point cloud from local file
	load_point_cloud_txt(reference_point_cloud_file_name, reference_point_cloud, false);

	std::cout
		<< "file information:\n"

		<< "read point cloud:\n\t" << reading_point_cloud_file_name << " point size:" << reading_point_cloud.size() << "\n"

		<< "reference point cloud:\n\t" << reference_point_cloud_file_name << " point size:" << reference_point_cloud.size() << "\n"

		<< "ioutput_folder:\n\t" << output_folder << "\n";
	
	std::vector<Eigen::Matrix4f> transformation_vec;

	// load coarse registration matrix
	std::string coarse_registration_matrix = output_folder + "/" + coarse_registration_filename;
	read_matrix(coarse_registration_matrix, transformation_vec);

	// load fine registration matrics
	std::string fine_registration_matrix = output_folder + "/" + fine_registration_filename;
	read_matrix(fine_registration_matrix, transformation_vec);

	// display all transformation repeatly
	cloud_viewer m_cloud_viewer("process of coarse and fine registration");

	m_cloud_viewer.add_point_cloud_with_color(reference_point_cloud, Eigen::Matrix4f::Identity(), 255, 0, 0);

	boost::thread update_reading_thread(display_point_cloud_from_transformation_vec, m_cloud_viewer, reading_point_cloud, transformation_vec);

	m_cloud_viewer.display();

	return 0;
}


#endif // DISPLAY
