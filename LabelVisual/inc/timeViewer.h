#pragma once

#include"header.h"

class timeViewer : public osgViewer::Viewer
{
public:
	timeViewer();
	~timeViewer();

	// will be called every frame
	//virtual void advance(double simulationTime = USE_REFERENCE_TIME);

public:
	void initial(std::vector<std::vector<point_3d>> & gnode_vec);

	void update_node();

private:
	osg::ref_ptr<osg::Geode> m_root;
	clock_t m_tstart, m_tend;
	// ms
	double m_trigger_interval;

	std::vector<size_t> m_index_vec;
	std::vector<std::vector<point_3d>> m_points_vec;
	std::vector<osg::ref_ptr<osg::Geometry>> m_point_cloud_vec;
};
