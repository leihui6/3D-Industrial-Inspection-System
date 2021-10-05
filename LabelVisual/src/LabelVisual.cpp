#include"LabelVisual.h"

LabelVisual::LabelVisual()
	: root(new osg::Group()), geode(new osg::Geode())
{

}

LabelVisual::~LabelVisual()
{

}

void LabelVisual::initial_label_info(const std::string & config_filename)
{
	read_config(config_filename);

	for (size_t i = 0; i < filename_vec.size(); i++)
	{
		std::vector<point_3d> point_cloud; std::map<std::string, point_3d > label_points_map;
		read_labelfile(filename_vec[i], point_cloud, label_points_map, ((display_step_vec[i] == 1) ? true : false));
		std::cout << "point cloud:" << i << " size:" << point_cloud.size() << std::endl;

		point_render_parameters parameters;
		parameters.set_point_color(cloud_color_vec[i]);
		parameters.set_point_size(cloud_size_vec[i]);
		
		// add points
		osg::ref_ptr<osg::Geometry> geode_point_cloud = new osg::Geometry();
		make_geometry_node(point_cloud, geode_point_cloud, parameters);
		geode->addDrawable(geode_point_cloud.get());

		// add its labels
		if (label_points_map.empty()) continue;
		osg::ref_ptr<osg::Geometry> geode_label_points = new osg::Geometry();
		osgText::Font* fontKai = osgText::readFontFile("C:\\WINDOWS\\Fonts\\simkai.ttf");
		std::vector< osg::ref_ptr<osgText::Text> > title_vec;
		title_vec.resize(label_points_map.size());
		size_t j = 0;
		for (auto & item : label_points_map)
		{	// string : position
			title_vec[j] = new osgText::Text;
			setupProperties(*title_vec[j], fontKai, 2, osg::Vec3(item.second.x, item.second.y, item.second.z));
			createContent(*title_vec[j], item.first);
			j++;
		}
		for (auto & text : title_vec) geode->addDrawable(text.get());
	}
}

void LabelVisual::visual_label()
{
	root->addChild(geode.get());

	// preparation for visualization
	osg::ref_ptr<timeViewer> viewer = new timeViewer;

	//dynamic_cast<Derived*>
	window_initilization(viewer, root);

	viewer->initial(points_vec);

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

void LabelVisual::read_labelfile(const std::string & filename, std::vector<point_3d> & all_points, std::map<std::string, point_3d > & label_points, bool step_subpoints)
{
	std::ifstream ifile(filename);

	if (!ifile.is_open())
	{
		std::cout << "Opening " + filename + " failed." << std::endl;
		return;
	}
	std::string line;

	std::vector<point_3d> points;
	point_3d centroid_point;

	bool skip = false;

	while (std::getline(ifile, line))
	{
		if (skip)
		{
			skip = false;
			continue;
		}
		if (line == ">points")
		{
			continue;
		}
		if (line == ">values")
		{
			skip = true;
			continue;
		}

		if (line.empty()) continue;

		if (line[0] == '#')
		{
			// get the centroid point among points
			cal_centroid_point(points, centroid_point);

			// get the label
			std::string label = line.substr(1, line.size() - 1);

			label_points.insert(std::pair<std::string, point_3d>(label, centroid_point));
			
			all_points.insert(all_points.end(), points.begin(), points.end());

			if (step_subpoints)
			{
				// add point cloud required to show by steps
				points_vec.push_back(points);
			}

			points.clear();
			continue;
		}

		std::stringstream s(line);

		float value[3] = { 0,0,0 };

		for (size_t i = 0; i < 3; i++) s >> value[i];

		points.push_back(point_3d(value[0], value[1], value[2]));
	}

	// do not forget the rest points that have no label
	if (!points.empty())
	{
		std::string label = "unknown-points";

		cal_centroid_point(points, centroid_point);
		label_points.insert(std::pair<std::string, point_3d>(label, centroid_point));

		all_points.insert(all_points.end(), points.begin(), points.end());

		if (step_subpoints)
			points_vec.push_back(points);

		points.clear();
	}

	ifile.close();
}

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

void LabelVisual::setupProperties(osgText::Text& textObject, osgText::Font* font, float size, const osg::Vec3& pos)
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

void LabelVisual::createContent(osgText::Text& textObject, const std::string & text/*char* string*/)
{
	//int requiredSize = mbstowcs(NULL, string, 0);//如果mbstowcs第一参数为NULL那么返回字符串的数目
	//wchar_t* wText = new wchar_t[requiredSize + 1];
	//mbstowcs(wText, string, requiredSize + 1);//由char转换成wchar类型
	//textObject.setText(wText);
	//delete wText;
	textObject.setText(text);
}

rgb LabelVisual::rgbstr2rgb(std::string rgbstr)
{
	std::stringstream stream(rgbstr);

	float rgb_[3] = { 0,0,0 };
	for (size_t i = 0; i < 3; i++)
		stream >> rgb_[i];

	return rgb(rgb_[0], rgb_[1], rgb_[2]);
}

float LabelVisual::str2float(std::string num)
{
	float res;
	std::stringstream stream(num);
	stream >> res;
	return res;
}

float LabelVisual::str2int(std::string num)
{
	int res;
	std::stringstream stream(num);
	stream >> res;
	return res;
}

void LabelVisual::read_config(const std::string config_filename)
{
	// filename r g b point_size step?
	std::ifstream ifile(config_filename);

	if (!ifile.is_open())
	{
		std::cout << "Opening " + config_filename + " failed." << std::endl;
		return;
	}

	std::string line;

	while (std::getline(ifile, line))
	{
		if (line.front() == '#') 
			continue;
		else if (line.empty()) 
			continue;

		std::vector<size_t> blank_index;
		for (size_t i = 0; i < line.size(); i++)
		{
			if (line[i] == ' ')
				blank_index.push_back(i);
		}

		//std::cout << line << std::endl;
		if (blank_index.size() == 5)
		{
			filename_vec.push_back(line.substr(0, blank_index[0]));
			cloud_color_vec.push_back(rgbstr2rgb(line.substr(blank_index[0] + 1, blank_index[3] - blank_index[0])));
			cloud_size_vec.push_back(str2float(line.substr(blank_index[3] + 1, blank_index[4] - blank_index[3])));
			display_step_vec.push_back(str2int(line.substr(blank_index[4] + 1, line.size() - blank_index[4])));
		}
	}
	ifile.close();
}