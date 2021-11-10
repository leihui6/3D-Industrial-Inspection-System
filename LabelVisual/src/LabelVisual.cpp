#include"LabelVisual.h"

LabelVisual::LabelVisual()
	: m_root(new osg::Group()),
	m_geode(new osg::Geode())
{

	m_fontKai = osgText::readFontFile("C:\\WINDOWS\\Fonts\\simkai.ttf");
	//test_i = 0;
}

LabelVisual::~LabelVisual()
{

}

void LabelVisual::add_points(std::map<std::string, std::vector<point_3d>> &points_map, bool show_normals)
{
	point_render_parameters parameters;
	parameters.color = osg::Vec4(255, 255, 255, 1);

	if (!show_normals)
	{
		for (auto &pv : points_map)
		{
			osg::ref_ptr<osg::Geode> geode_points = new osg::Geode();
			make_points_node(pv.second, geode_points, parameters);
			this->m_geode->addChild(geode_points);
		}
	}
	else
	{
		add_points(points_map, false);

		for (auto & item : points_map)
		{
			for (auto & p : item.second)
			{
				Eigen::Vector3f v(p.nx, p.ny, p.nz);
				osg::ref_ptr<osg::Geode> geode_normal = new osg::Geode();
				parameters.color = osg::Vec4(0, 0, 255, 1);
				make_normals_node(p, v, geode_normal, parameters);
				this->m_geode->addChild(geode_normal);
			}
		}
	}
}

void LabelVisual::add_marked_points(std::map<std::string, point_shape> & marked_points_map)
{
	if (marked_points_map.empty()) return;

	point_render_parameters parameters;
	parameters.color = osg::Vec4(255, 255, 255, 1);

	for (auto &ps : marked_points_map)
	{
		point_3d centroid_point;
		cal_centroid_point(ps.second.points, centroid_point);

		std::vector<int> random_color(3);
		obtain_random(0, 255, random_color);
		parameters.color = osg::Vec4(
			random_color[0] / 255.0,
			random_color[1] / 255.0,
			random_color[2] / 255.0, 1.0);

		// add normal
		if (!ps.second.shape_property.empty())
		{
			Eigen::Vector3f v = ps.second.shape_property[0];
			osg::ref_ptr<osg::Geode> geode_normal = new osg::Geode();
			parameters.point_size = 3;
			make_normals_node(centroid_point, v, geode_normal, parameters);
			m_geode->addChild(geode_normal);
		}

		// add marked points
		osg::ref_ptr<osg::Geode> geode_marked_points = new osg::Geode();
		parameters.point_size = 8;
		make_points_node(ps.second.points, geode_marked_points, parameters);
		this->m_geode->addChild(geode_marked_points);

		// add label-name
		osg::ref_ptr<osg::Geode> geode_text = new osg::Geode();
		make_text_node(centroid_point, geode_text, ps.first);
		this->m_geode->addChild(geode_text);
	}
}

void LabelVisual::add_measurement_points(std::map<std::string, std::vector<point_3d>> &measurement_points_map)
{
	add_points(measurement_points_map, true);

	point_render_parameters parameters;
	parameters.color = osg::Vec4(255, 255, 255, 1);

	for (auto & item : measurement_points_map)
	{
		point_3d centroid_point;
		cal_centroid_point(item.second, centroid_point);

		osg::ref_ptr<osg::Geode> geode_text = new osg::Geode();
		make_text_node(centroid_point, geode_text, item.first);
		this->m_geode->addChild(geode_text);
	}
}

void LabelVisual::initial(const std::string & file_1, const std::string & file_2, int flag)
{
	std::map<std::string, std::vector<point_3d>> points_map;

	read_points(points_map, file_1);

	add_points(points_map, false);

	// show marked points
	if (flag == 0)
	{
		std::map<std::string, point_shape> marked_points_map;

		read_marked_points(marked_points_map, file_2);
		//std::cout << "read " << marked_points_vec.size() << " marked point group" << std::endl;

		add_marked_points(marked_points_map);
	}

	// show result points
	else if (flag == 1)
	{
		std::map<std::string, std::vector<point_3d>> measurement_points_map;

		read_points(measurement_points_map, file_2);

		add_measurement_points(measurement_points_map);

		read_measurement_points(m_measurement_points, file_2);
	}

	return;
}

void LabelVisual::visual()
{
	m_root->addChild(m_geode.get());

	// preparation for visualization
	osg::ref_ptr<timeViewer> viewer = new timeViewer;

	//dynamic_cast<Derived*>
	window_initilization(viewer, m_root);

	viewer->initial(std::vector<std::vector<point_3d>>{m_measurement_points});

	viewer->realize();

	viewer->run();
}

void LabelVisual::cal_centroid_point(std::vector<point_3d> & points, point_3d & centroid_point)
{
	float t_x = 0, t_y = 0, t_z = 0;
	for (auto & p : points)
	{
		t_x += p.x;
		t_y += p.y;
		t_z += p.z;
	}
	centroid_point.x = t_x / points.size();
	centroid_point.y = t_y / points.size();
	centroid_point.z = t_z / points.size();
}
//
//void LabelVisual::read_labelfile(const std::string & filename,
//	std::vector<point_3d> & all_points, std::map<std::string, point_3d > & label_points,
//	std::vector<point_3d> & property_content, bool step_subpoints)
//{
//	std::ifstream ifile(filename);
//
//	if (!ifile.is_open())
//	{
//		std::cout << "Opening " + filename + " failed." << std::endl;
//		return;
//	}
//	std::string line;
//
//	point_3d centroid_point;
//	std::vector<point_3d> points;
//	point_3d normal;
//	bool has_pro = false;
//	//std::vector<std::vector<point_3d>> step_points_unorder;
//
//	bool flag_points = false, flag_property = false;
//
//	while (std::getline(ifile, line))
//	{
//		if (line.empty()) continue;
//
//		if (line.find(">points") != std::string::npos)
//		{
//			flag_points = true;
//			flag_property = false;
//			continue;
//		}
//		else if (line.find(">property") != std::string::npos)
//		{
//			flag_property = true;
//			flag_points = false;
//			continue;
//		}
//
//		else if (line[0] == '#' && line.size() > 2)
//		{
//			// get the centroid point among points
//			cal_centroid_point(points, centroid_point);
//
//			// get the label
//			std::string label = line.substr(1, line.size() - 1);
//
//			label_points.insert(std::pair<std::string, point_3d>(label, centroid_point));
//
//			all_points.insert(all_points.end(), points.begin(), points.end());
//
//			if (has_pro)
//			{
//				property_content.push_back(normal);
//				property_content.push_back(centroid_point);
//				has_pro = false;
//			}
//
//			if (step_subpoints)
//			{
//				//step_points_unorder.push_back(points);
//			}
//
//			points.clear();
//			continue;
//		}
//
//		std::stringstream s(line);
//
//		float value[3] = { 0,0,0 };
//
//		for (size_t i = 0; i < 3; i++) s >> value[i];
//
//		if (flag_points)
//			points.push_back(point_3d(value[0], value[1], value[2]));
//
//		if (flag_property)
//		{
//			has_pro = true;
//			normal.x = value[0];
//			normal.y = value[1];
//			normal.z = value[2];
//		}
//	}
//
//	//// do not forget the rest points that have no label
//	//if (!points.empty())
//	//{
//	//	std::string label = "unknown-points";
//
//	//	cal_centroid_point(points, centroid_point);
//	//	label_points.insert(std::pair<std::string, point_3d>(label, centroid_point));
//
//	//	all_points.insert(all_points.end(), points.begin(), points.end());
//
//	//	if (step_subpoints)
//	//	{
//	//		//step_points_unorder.push_back(points);
//	//	}
//
//	//	points.clear();
//	//}
//
//	ifile.close();
//	//make_order(step_points_unorder);
//}

void LabelVisual::window_initilization(osg::ref_ptr<timeViewer> & viewer, osg::ref_ptr<osg::Group> & root)
{
	viewer->addEventHandler(new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()));

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 40;
	traits->y = 40;
	traits->width = 600;
	traits->height = 480;
	traits->windowName = "Label visualization";
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Camera> camera = viewer->getCamera();
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	GLenum buffer = (traits->doubleBuffer) ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);
	camera->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));

	osg::CullStack::CullingMode cullingMode = camera->getCullingMode();
	cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
	camera->setCullingMode(cullingMode);

	osgUtil::Optimizer optimizer;
	optimizer.optimize(root.get());
	viewer->setSceneData(root.get());
}



void LabelVisual::clear()
{
	m_root = new osg::Group;

	m_geode = new osg::Geode;
}


void LabelVisual::make_text_node(point_3d & position, osg::ref_ptr<osg::Geode> geode_label_points, const std::string & text)
{
	osg::ref_ptr<osgText::Text> text_osg(new osgText::Text);
	text_setupProperties(*text_osg, m_fontKai, 2, osg::Vec3(position.x, position.y, position.z));
	text_createContent(*text_osg, text);
	geode_label_points->addChild(text_osg);
}

void LabelVisual::text_setupProperties(
	osgText::Text& textObject, osgText::Font* font,
	float size, const osg::Vec3& pos)
{
	textObject.setFont(font);
	textObject.setCharacterSize(size);
	textObject.setPosition(pos);
	textObject.setColor(osg::Vec4(255, 255, 0.0, 1.0));
	textObject.setAlignment(osgText::Text::CENTER_BOTTOM);//文字显示方向
	textObject.setAxisAlignment(osgText::Text::SCREEN);//文字对称成方式正对屏幕方向
	//textObject.setCharacterSizeMode(osgText::Text::SCREEN_COORDS);//跟随视角不断变化，离物体越远，文字越大
	textObject.setAutoRotateToScreen(true);//跟随视角不断变化，但离物体越远，文字越小，和现实当中像类似
	//textObject.setBackdropType(osgText::Text::OUTLINE);//对文字进行描边
	//textObject.setBackdropColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));//描边颜色
	textObject.setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);//添加文字边框
	//textObject.setAxisAlignment(osgText::Text::XZ_PLANE);//获取文字对称成方式
}

void LabelVisual::text_createContent(osgText::Text& textObject, const std::string & text/*char* string*/)
{
	//int requiredSize = mbstowcs(NULL, string, 0);//如果mbstowcs第一参数为NULL那么返回字符串的数目
	//wchar_t* wText = new wchar_t[requiredSize + 1];
	//mbstowcs(wText, string, requiredSize + 1);//由char转换成wchar类型
	//textObject.setText(wText);
	//delete wText;
	textObject.setText(text);
}

//rgb LabelVisual::rgbstr2rgb(std::string rgbstr)
//{
//	std::stringstream stream(rgbstr);
//
//	float rgb_[3] = { 0,0,0 };
//	for (size_t i = 0; i < 3; i++)
//		stream >> rgb_[i];
//
//	return rgb(rgb_[0], rgb_[1], rgb_[2]);
//}

//float LabelVisual::str2float(std::string num)
//{
//	float res;
//	std::stringstream stream(num);
//	stream >> res;
//	return res;
//}
//
//float LabelVisual::str2int(std::string num)
//{
//	int res;
//	std::stringstream stream(num);
//	stream >> res;
//	return res;
//}

//void LabelVisual::read_config(const std::string config_filename)
//{
	// filename r g b point_size step?
	//std::ifstream ifile(config_filename);

	//if (!ifile.is_open())
	//{
	//	std::cout << "Opening " + config_filename + " failed." << std::endl;
	//	return;
	//}

	//std::string line;

	//while (std::getline(ifile, line))
	//{
	//	if (line.front() == '#') 
	//		continue;
	//	else if (line.empty()) 
	//		continue;

	//	std::vector<size_t> blank_index;
	//	for (size_t i = 0; i < line.size(); i++)
	//	{
	//		if (line[i] == ' ')
	//			blank_index.push_back(i);
	//	}

	//	//std::cout << line << std::endl;
	//	if (blank_index.size() == 5)
	//	{
	//		filename_vec.push_back(line.substr(0, blank_index[0]));
	//		cloud_color_vec.push_back(rgbstr2rgb(line.substr(blank_index[0] + 1, blank_index[3] - blank_index[0])));
	//		cloud_size_vec.push_back(str2float(line.substr(blank_index[3] + 1, blank_index[4] - blank_index[3])));
	//		display_step_vec.push_back(str2int(line.substr(blank_index[4] + 1, line.size() - blank_index[4])));
	//	}
	//}
	//ifile.close();
//}

//float LabelVisual::distance(point_3d & p1, point_3d & p2)
//{
//	return sqrtf(
//		(p1.x - p2.x) * (p1.x - p2.x) +
//		(p1.y - p2.y) * (p1.y - p2.y) +
//		(p1.z - p2.z) * (p1.z - p2.z));
//}

//void LabelVisual::make_order(std::vector<std::vector<point_3d>>& step_points_unorder)
//{
//	if (step_points_unorder.empty()) return;
//
//	std::vector<point_3d> beg_end_vec;
//	for (auto & v : step_points_unorder)
//	{
//		beg_end_vec.push_back(v.front());
//		beg_end_vec.push_back(v.back());
//	}
//
//	// add the first one
//	step_points_vec.push_back(step_points_unorder.front());
//
//	std::vector<bool> step_used_vec(step_points_unorder.size(), 0);
//	step_used_vec[0] = 1;
//
//	for (size_t k = 0; k < step_points_vec.size(); k++)
//	{
//		point_3d fix_p;
//		fix_p = step_points_vec[k].back();
//
//		float d_1 = 0, d_2 = 0, min_distance = FLT_MAX;
//		point_3d s_beg_p, s_end_p;
//		size_t min_index = 0;
//
//		// find the min distance one
//		for (size_t i = 0; i < beg_end_vec.size(); i += 2)
//		{
//			if (step_used_vec[i / 2] == 0)
//			{
//				s_beg_p = beg_end_vec[i]; s_end_p = beg_end_vec[i + 1];
//
//				d_1 = distance(fix_p, s_beg_p); d_2 = distance(fix_p, s_end_p);
//
//				if (d_1 < min_distance)
//				{
//					min_distance = d_1;
//					min_index = i;
//				}
//				else if (d_2 < min_distance)
//				{
//					min_distance = d_2;
//					min_index = i + 1;
//				}
//			}
//		}// end for find
//
//		size_t main_i = min_index / 2;
//		if (step_used_vec[main_i] == 0)
//		{
//			if (min_index % 2 == 1)
//			{
//				std::vector<point_3d> tmp(step_points_unorder[main_i].rbegin(), step_points_unorder[main_i].rend());
//				step_points_vec.push_back(tmp);
//			}
//			else
//			{
//				step_points_vec.push_back(step_points_unorder[main_i]);
//			}
//
//			step_used_vec[main_i] = 1;
//		}// end if
//	}// end for
//
//	std::vector<point_3d> step_vec;
//	for (auto &v : step_points_vec)
//		for (auto &p : v)
//			step_vec.push_back(p);
//	step_points_vec.clear();
//	step_points_vec.push_back(step_vec);
//}
