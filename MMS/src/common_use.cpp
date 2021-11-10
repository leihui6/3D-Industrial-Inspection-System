#include "common_use.h"


point_3d::point_3d(const point_3d & p)
{
	this->set_xyz(p.x, p.y, p.z);
	this->set_nxyz(p.nx, p.ny, p.nz);
	this->set_rgb(p.r, p.g, p.b);
}

point_3d::point_3d()
	:x(0), y(0), z(0),
	nx(0), ny(0), nz(0),
	r(0), g(0), b(0)
{
}

point_3d::point_3d(float x, float y, float z)
{
	this->set_xyz(x, y, z);
	this->set_nxyz(0, 0, 0);
	this->set_rgb(0, 0, 0);
}

void point_3d::set_xyz(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void point_3d::set_nxyz(float nx, float ny, float nz)
{
	this->nx = nx;
	this->ny = ny;
	this->nz = nz;
}

void point_3d::set_rgb(float r, float g, float b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}

Eigen::Vector3f point_3d::get_vector3f()
{
	return Eigen::Vector3f(this->x, this->y, this->z);
}

void point_3d::do_transform(const Eigen::Matrix4f & m, point_3d & p)
{
	Eigen::Vector4f t_p(x, y, z, 1);
	Eigen::Vector3f t_n(nx, ny, nz);

	t_p = m * t_p;
	t_n = m.block<3, 3>(0, 0).inverse().transpose() * t_n;

	p.set_xyz(t_p[0], t_p[1], t_p[2]);
	p.set_nxyz(t_n[0], t_n[1], t_n[2]);
}

void point_3d::do_transform(const Eigen::Matrix4f & m)
{
	Eigen::Vector4f p(x, y, z, 1);
	Eigen::Vector3f n(nx, ny, nz);

	p = m * p;
	n = m.block<3, 3>(0, 0).inverse().transpose() * n;

	set_xyz(p[0], p[1], p[2]);
	set_nxyz(n[0], n[1], n[2]);
}

point_3d & point_3d::operator=(const point_3d & p)
{
	if (this != &p)
	{
		this->set_xyz(p.x, p.y, p.z);
		this->set_nxyz(p.nx, p.ny, p.nz);
		this->set_rgb(p.r, p.g, p.b);
	}
	return *this;
}

point_3d point_3d::operator+(const point_3d & p)
{
	point_3d tp;

	tp.set_xyz(this->x + p.x, this->y + p.y, this->z + p.z);

	return tp;
}

point_3d point_3d::operator-(const point_3d & p)
{
	point_3d tp;

	tp.set_xyz(this->x - p.x, this->y - p.y, this->z - p.z);

	return tp;
}

point_3d point_3d::operator/(const float num)
{
	point_3d tp;

	tp.set_xyz(this->x / num, this->y / num, this->z / num);

	return tp;
}

bool point_3d::operator==(const point_3d & rp)
{
	if (this->x == rp.x &&
		this->y == rp.y &&
		this->z == rp.z)
		return true;
	return false;
}

std::ostream & operator << (std::ostream & os, const point_3d & p)
{
	os
		<< "(x,y,z,nx,ny,nz,r,g,b)="
		<< p.x << " " << p.y << " " << p.z << " "
		<< p.nx << " " << p.ny << " " << p.nz << " "
		<< p.r << " " << p.g << " " << p.b << " ";
	return os;
}

line_func_3d::line_func_3d()
	:origin(),
	direction()
{

}

line_func_3d::line_func_3d(const line_func_3d & lf)
{
	this->origin = lf.origin;
	this->direction = lf.direction;
}

void line_func_3d::set_xyz(float x, float y, float z)
{
	this->origin[0] = x;
	this->origin[1] = y;
	this->origin[2] = z;
}

void line_func_3d::set_nml(float n, float m, float l)
{
	this->direction[0] = n;
	this->direction[1] = m;
	this->direction[2] = l;
}

point_3d line_func_3d::get_origin_point_3d()
{
	return point_3d(origin[0], origin[1], origin[2]);
}

point_3d line_func_3d::get_direction_point_3d()
{
	return point_3d(direction[0], direction[1], direction[2]);
}

line_func_3d & line_func_3d::operator=(const line_func_3d & lf)
{
	if (this != &lf)
	{
		this->origin = lf.origin;
		this->direction = lf.direction;
	}
	return *this;
}


//Eigen::Vector3f line_func_3d::direction()
//{
//	return Eigen::Vector3f(this->n, this->m, this->l);
//}
//
//Eigen::Vector3f line_func_3d::origin_point()
//{
//	return Eigen::Vector3f(this->x, this->y, this->z);
//}

plane_func_3d::plane_func_3d()
	: a(0), b(0), c(0), d(0)
{

}

plane_func_3d::plane_func_3d(float _a, float _b, float _c, float _d)
{
	this->set_abcd(_a, _b, _c, _d);
}

void plane_func_3d::set_abcd(float _a, float _b, float _c, float _d)
{
	a = _a;
	b = _b;
	c = _c;
	d = _d;
}

void plane_func_3d::set_abcd(float _a, float _b, float _c, point_3d & p)
{
	a = _a;
	b = _b;
	c = _c;
	d = -(a*p.x + b * p.y + c * p.z);
}

void plane_func_3d::reverse()
{
	set_abcd(-1 * a, -1 * b, -1 * c, -1 * d);
}


//Eigen::Vector3f plane_func_3d::direction()
//{
//	return Eigen::Vector3f(this->a, this->b, this->c);
//}

cylinder_func::cylinder_func()
	: radius(0),
	height(0),
	axis()
{

}

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

void read_points(std::map<std::string, std::vector<point_3d>> & points_map, const std::string & filename)
{
	LocalFile local_file;

	if (!check_file(filename, std::ios::in, local_file)) return;

	std::fstream & ifile = *local_file.m_fileobject;

	std::string line;
	std::vector<point_3d> points;

	points_map.clear();

	float value[6] = { 0,0,0,0,0,0 };
	point_3d current_p;

	while (std::getline(ifile, line))
	{
		if (line.empty()) continue;

		if (line[0] == '>') continue;

		if (line[0] == '#' && line.size() > 2)
		{
			std::string label_name = line.substr(1, line.size() - 1);
			//std::cout << label_name << std::endl;
			points_map[label_name] = points;

			points.clear();

			continue;
		}

		std::stringstream s(line);

		for (size_t i = 0; i < 6; i++)	s >> value[i];

		current_p.set_xyz(value[0], value[1], value[2]);
		current_p.set_nxyz(value[3], value[4], value[5]);
		
		points.push_back(current_p);
	}

	if (!points.empty())
	{
		points_map["unknown-points"] = points;
	}

	ifile.close();
}

void read_measurement_points(std::vector<point_3d> & measurement_points, const std::string & filename)
{
	LocalFile local_file;

	if (!check_file(filename, std::ios::in, local_file)) return;

	std::fstream & ifile = *local_file.m_fileobject;

	std::string line;

	while (std::getline(ifile, line))
	{
		if (line.empty()) continue;

		if (line.find(">") != std::string::npos
			||
			line.find("#") != std::string::npos)
		{
			continue;
		}

		std::stringstream s(line);

		float value[3] = { 0,0,0 };

		for (size_t i = 0; i < 3; i++) s >> value[i];

		measurement_points.push_back(point_3d(value[0], value[1], value[2]));
	}
	ifile.close();
}

void read_marked_points(std::map<std::string, point_shape> & point_shape_map, const std::string & filename)
{
	LocalFile local_file;

	if (!check_file(filename, std::ios::in, local_file)) return;

	std::fstream & ifile = *local_file.m_fileobject;

	point_shape_map.clear();

	point_shape ps;
	std::string line;
	bool flag_points = false, flag_property = false;

	while (std::getline(ifile, line))
	{
		if (line.empty()) continue;
		
		if (line.find(">points") != std::string::npos)
		{
			flag_points = true;
			flag_property = false;
			continue;
		}
		else if (line.find(">property") != std::string::npos)
		{
			flag_property = true;
			flag_points = false;
			continue;
		}
		else if (line[0] == '#' && line.size() > 2)
		{
			point_shape_map[line.substr(1, line.size() - 1)] = ps;
			ps.points.clear();
			ps.shape_property.clear();
			continue;
		}

		std::stringstream s(line);

		float value[3] = { 0,0,0 };

		for (size_t i = 0; i < 3; i++) s >> value[i];

		if (flag_points)
			ps.points.push_back(point_3d(value[0], value[1], value[2]));
		if(flag_property)
			ps.shape_property.push_back(Eigen::Vector3f(value[0], value[1], value[2]));
	}

	ifile.close();
}

void export_marked_points(std::map<std::string, point_shape>& marked_points, const std::string & export_file_name)
{
	LocalFile local_file;

	if (!check_file(export_file_name, std::ios::out, local_file)) return;

	std::fstream & ofile = *local_file.m_fileobject;

	std::map <std::string, point_shape>::iterator it;

	for (it = marked_points.begin(); it != marked_points.end(); it++)
	{
		ofile << ">property\n";
		for (auto &v : it->second.shape_property)
		{
			ofile << v[0] << " " << v[1] << " " << v[2] << "\n";
		}

		ofile << ">points\n";
		std::vector<point_3d> &ps = it->second.points;

		for (size_t j = 0; j < ps.size(); j++)
		{
			ofile << ps[j].x << " " << ps[j].y << " " << ps[j].z << "\n";
		}
		// # label-name
		ofile << "#" << it->first << "\n";
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

	std::fstream & ifile = *local_file.m_fileobject;

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

	std::fstream & ifile = *local_file.m_fileobject;

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

std::string current_date_time(bool need_date, bool need_time)
{
	std::time_t now =
		std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

	std::string s(30, '\0');

	std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

	if (need_date && !need_time)
	{
		std::strftime(&s[0], s.size(), "%Y-%m-%d", std::localtime(&now));
	}
	else if (!need_date && need_time)
	{
		std::strftime(&s[0], s.size(), "%H:%M:%S", std::localtime(&now));
	}
	else if (need_date && need_time)
	{
		std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
	}
	else
	{
		// ... 
	}

	return s.substr(0, s.find('\0'));
}

bool check_file(const std::string filename, std::ios_base::openmode mode, LocalFile & local_file)
{
	local_file.m_fileobject->open(filename, mode);

	if (!local_file.m_fileobject->is_open())
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

	std::fstream & of = *local_file.m_fileobject;

	of << matrix << "\n#\n";

	of.close();
}

void read_matrix(const std::string & file_name, std::vector<Eigen::Matrix4f> & m_v)
{
	LocalFile local_file;

	if (!check_file(file_name, std::ios::in, local_file)) return;

	std::fstream & ifile = *local_file.m_fileobject;

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

int obtain_random(int range_min, int range_max)
{
	std::random_device dev;
	std::mt19937 rng(dev());

	std::uniform_int_distribution<std::mt19937::result_type> dist6(range_min, range_max); 

	return dist6(rng);
}


void obtain_random(int range_min, int range_max, std::vector<int> &random_vec)
{
	for (auto & i : random_vec)
	{
		i = obtain_random(range_min, range_max);
	}
}