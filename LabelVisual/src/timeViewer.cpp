#include "TimeViewer.h"

timeViewer::timeViewer()
{
	m_trigger_interval = 150;
	m_tstart = clock();
}


timeViewer::~timeViewer()
{
}

void timeViewer::advance(double simulationTime)
{
	Viewer::advance();
	m_tend = clock();
	if (m_tend - m_tstart < m_trigger_interval) return;

	for (auto &node : m_point_cloud_vec)
		m_root->removeChild(node);

	update_node();

	m_tstart = clock();
}

void timeViewer::initial(std::vector<std::vector<point_3d>> & points_vec)
{
	srand((unsigned int)time(0));
	m_index_vec.resize(points_vec.size(), 0);
	for (auto &i : m_index_vec) i = rand() % 100;

	m_points_vec = points_vec;

	Viewer::advance();
	osg::ref_ptr<osg::Group> v_root = this->getSceneData()->asGroup();
	osg::ref_ptr<osg::Geode> gnode = new osg::Geode();
	v_root->addChild(gnode);
	m_root = gnode;
}

void timeViewer::update_node()
{
	point_render_parameters parameters;
	parameters.set_point_color(rgb(255, 255, 0));
	parameters.set_point_size(10);

	for (size_t i = 0; i < m_points_vec.size(); ++i)
	{
		osg::ref_ptr<osg::Geometry> geode_point_cloud = new osg::Geometry();

		// only one point actually
		std::vector<point_3d> point;
		point.push_back(m_points_vec.at(i)[m_index_vec.at(i)]);
		m_index_vec.at(i)++;
		if (m_index_vec.at(i) >= m_points_vec.at(i).size())
			m_index_vec.at(i) = 0;
		//std::cout << m_index_vec.at(i) << std::endl;
		make_geometry_node(point, geode_point_cloud, parameters);

		// will be deleted and empty
		m_point_cloud_vec.push_back(geode_point_cloud);
		m_root->addDrawable(geode_point_cloud.get());	
	}
}
