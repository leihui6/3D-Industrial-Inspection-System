#include "header.h"

void make_geometry_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> & geometry, point_render_parameters & parameters)
{
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();

	colors->push_back(parameters.get_color());
	normals->push_back(parameters.get_normal());

	for (auto & p : points) coords->push_back(osg::Vec3(p.x, p.y, p.z));

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.size()));

	geometry->setVertexArray(coords.get());

	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

	osg::StateSet* stateSet = geometry->getOrCreateStateSet();
	osg::Point* state_point_size = new osg::Point;
	state_point_size->setSize(parameters.get_point_size());
	stateSet->setAttribute(state_point_size);
}