#include "common_use.h"

std::string file_name_without_postfix(std::string & file_name)
{
	size_t first_pos = file_name.find('/', 0), second_pos = file_name.find('.', 0);

	if (second_pos > first_pos)
	{
		return file_name.substr(first_pos + 1, second_pos - first_pos - 1);
	}
	return file_name;
}

bool is_exist(const std::string & file_name)
{
	return std::filesystem::exists(file_name);;
}

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

void read_points(std::map<std::string, std::vector<point_3d>> & points_map, const std::string & file_name)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::in, local_file)) return;

	std::fstream & ifile = local_file.m_fileobject;

	std::string line;
	std::vector<point_3d> points;

	points_map.clear();

	while (std::getline(ifile, line))
	{
		if (line.size() < 1)
			continue;

		if (line[0] == '#' && line.size() > 2)
		{
			points_map[line.substr(1, line.size() - 1)] = points;

			points.clear();

			continue;
		}

		std::stringstream s(line);

		float value[3] = { 0,0,0 };

		for (size_t i = 0; i < 3; i++)
			s >> value[i];

		points.push_back(point_3d(value[0], value[1], value[2]));
	}

	ifile.close();
}

void export_marked_points(std::map<std::string, std::vector<point_3d>>& marked_points, const std::string & export_file_name)
{
	LocalFile local_file;

	if (!check_file(export_file_name, std::ios::out, local_file)) return;

	std::fstream & point_file = local_file.m_fileobject;

	std::map <std::string, std::vector<point_3d>>::iterator it;

	for (it = marked_points.begin(); it != marked_points.end(); it++)
	{
		std::vector<point_3d> &ps = it->second;

		for (size_t j = 0; j < ps.size(); j++)
		{
			point_file << ps[j].x << " " << ps[j].y << " " << ps[j].z << "\n";
		}
		point_file << "#" << it->first << "\n";
	}

	point_file.close();
}

void export_measured_data(std::multimap<std::string, std::string>& measurement_pairs_map, std::vector<measurement_content>& mc_vec, const std::string & output_file_name)
{
	if (mc_vec.empty() || measurement_pairs_map.empty()) return;

	if (mc_vec.size() != measurement_pairs_map.size()) return;

	LocalFile local_file;

	if (!check_file(output_file_name, std::ios::out, local_file)) return;

	std::fstream & ofile = local_file.m_fileobject;

	std::multimap<std::string, std::string>::iterator it;

	size_t i = 0;
	for (it = measurement_pairs_map.begin(); it != measurement_pairs_map.end(); ++it)
	{
		if (mc_vec[i].m_method == MM_VALUES)
		{
			ofile << ">values" << "\n";
			if (mc_vec[i].distance_geometrical == INVALIDVALUE)
				ofile << INVALIDVALUE << " ";
			else
				ofile << mc_vec[i].distance_geometrical << " ";

			if (mc_vec[i].distance_scattered == INVALIDVALUE)
				ofile << INVALIDVALUE << " ";
			else
				ofile << mc_vec[i].distance_scattered << " ";

			if (mc_vec[i].angle == INVALIDVALUE)
				ofile << INVALIDVALUE << " ";
			else
				ofile << mc_vec[i].angle << " ";

			ofile << "\n";
		}
		else if (mc_vec[i].m_method == MM_POINTS)
		{
			ofile << ">points" << "\n";
			for (auto & p : mc_vec[i].drawable_points)
			{
				ofile << p.x << " " << p.y << " " << p.z << "\n";
			}
		}

		std::string label = "#" + it->first + "-" + it->second;
		ofile << label << "\n";

		++i;
	}
	ofile.close();
}

void transform_marked_points(std::map<std::string, std::vector<point_3d>>& marked_points, Eigen::Matrix4f & m)
{
	std::map<std::string, std::vector<point_3d>>::iterator it;

	for (it = marked_points.begin(); it != marked_points.end(); it++)
	{
		std::vector<point_3d> & ptsv = it->second;

		for (size_t i = 0; i < ptsv.size(); ++i)
		{
			ptsv[i].do_transform(m);
		}
	}
}

void string_split(std::string & str, char character, std::vector<std::string>& res)
{
	if (str.empty()) return;

	std::vector<size_t> character_index_vec;

	for (size_t i = 0; i < str.size(); ++i)
		if (str[i] == character)
			character_index_vec.push_back(i);

	if (character_index_vec.empty()) return;

	if (str.size() != character_index_vec.back())
		character_index_vec.push_back(str.size());

	size_t start_index = 0;
	for (size_t i = 0; i < character_index_vec.size(); ++i)
	{
		res.push_back(str.substr(start_index, character_index_vec[i] - start_index));
		start_index = character_index_vec[i] + 1;
	}
}

void read_file_as_map(const std::string & file_name, std::map<std::string, std::string> & str_flt_map)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::in, local_file)) return;

	std::fstream & ifile = local_file.m_fileobject;

	std::string line;

	while (std::getline(ifile, line))
	{
		if (line.empty()) continue;

		if (line[0] == '#') continue;

		size_t divided_flag = line.find(":");

		if (divided_flag == std::string::npos) continue;

		std::string
			key_ = line.substr(0, divided_flag++);

		std::string
			value_ = line.substr(divided_flag, line.size() - divided_flag);

		str_flt_map[key_] = value_;
	}
	std::cout << "read " << str_flt_map.size() << " parameters from local file.\n";
}

void read_file_as_map(const std::string & file_name, std::multimap<std::string, std::string> & str_flt_map)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::in, local_file)) return;

	std::fstream & ifile = local_file.m_fileobject;

	std::string line;

	while (std::getline(ifile, line))
	{
		if (line.empty()) break;

		if (line[0] == '#') continue;

		size_t divided_flag = line.find(":");

		if (divided_flag == std::string::npos) continue;

		std::string
			key_ = line.substr(0, divided_flag++);

		std::string
			value_ = line.substr(divided_flag, line.size() - divided_flag);

		str_flt_map.insert(std::pair<std::string, std::string>(key_, value_));
	}
}

void read_file_as_map(const std::string & file_name, 
	std::multimap<std::string, std::string>& str_flt_map, 
	std::multimap<std::string, std::string> & reference_map)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::in, local_file)) return;

	std::fstream & ifile = local_file.m_fileobject;

	str_flt_map.clear(); reference_map.clear();

	std::string line;

	std::vector<std::string> split_str;

	while (std::getline(ifile, line))
	{
		if (line.empty()) break;

		if (line[0] == '#') continue;

		string_split(line, ':', split_str);

		if (split_str.size() == 3)
		{
			str_flt_map.insert(std::pair<std::string, std::string>(split_str[0], split_str[1]));
			
			std::string measurement_pair = split_str[0] + "-" + split_str[1];
			reference_map.insert(std::pair<std::string, std::string>(measurement_pair, split_str.back()));
		}
		else if (split_str.size() == 2)
		{
			str_flt_map.insert(std::pair<std::string, std::string>(split_str[0], split_str[1]));
		}
		else
		{
			std::cerr << "error: the number of measurement pair should be 3 or 2" << std::endl;
			continue;
		}
		split_str.clear();
	}
}

osg::Vec4 str_to_vec4(const std::string & s)
{
	std::stringstream ss(s);
	
	osg::Vec4 vec4;
	
	float tmp; size_t i = 0;
	
	while (ss >> tmp) vec4[i++] = tmp;
	
	if (i != 4)	
		return osg::Vec4();
	else	
		return vec4;
}

std::string current_date_time(bool need_date, bool need_time)
{
	time_t ttime = time(0);
	tm *local_time = localtime(&ttime);

	std::string
		year = std::to_string(1900 + local_time->tm_year),
		month = std::to_string(1 + local_time->tm_mon),
		day = std::to_string(local_time->tm_mday),
		hour = std::to_string(1 + local_time->tm_hour),
		minute = std::to_string(1 + local_time->tm_min),
		second = std::to_string(1 + local_time->tm_sec);

	std::string current_time;

	if (need_date)
	{
		current_time = year + '-' + month + '-' + day + ' ';
	}

	if (need_time)
	{
		current_time += (hour + ':' + minute + ':' + second + ' ');
	}

	return current_time;
}

bool check_file(const std::string filename, std::ios_base::openmode mode, LocalFile & local_file)
{
	local_file.m_fileobject.open(filename, mode);

	if (!local_file.m_fileobject.is_open())
	{
		std::cerr << "file (" << filename << ") doesn't exist\n";
		return false;
	}

	return true;
}

void save_matrix(Eigen::Matrix4f & matrix, const std::string & file_name)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::out, local_file)) return;

	std::fstream & of = local_file.m_fileobject;

	of << matrix << "\n#\n";

	of.close();
}

void read_matrix(const std::string & file_name, std::vector<Eigen::Matrix4f> & m_v)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::in, local_file)) return;

	std::fstream & ifile = local_file.m_fileobject;

	std::string line;

	std::vector<float> matrix_value;

	while (std::getline(ifile, line))
	{
		if (line.empty())
		{
			continue;
		}

		if (line[0] == '#')
		{
			continue;
		}

		std::stringstream s(line);

		float tmp;

		for (size_t i = 0; i < 4; ++i)
		{
			s >> tmp;
			matrix_value.push_back(tmp);
		}

		if (matrix_value.size() == 16)
		{
			Eigen::Matrix4f m_tmp(matrix_value.data());

			m_tmp.transposeInPlace();

			m_v.push_back(m_tmp);

			matrix_value.clear();
		}
	}

	ifile.close();
}