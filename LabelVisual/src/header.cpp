#include "header.h"

void make_points_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> & geometry, point_render_parameters & parameters)
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

void make_normals_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geode> &geode, point_render_parameters & parameters)
{
	std::cout << points.size() << std::endl;

	if (points.size() % 2 != 0 || points.empty()) return;

	for (size_t i = 0; i < points.size(); i += 2)
	{
		Eigen::Vector3f v(points[i].x, points[i].y, points[i].z);

		point_3d r_p;
		v.normalize();
		point_along_with_vector_within_dis(points[i + 1], v, r_p, 5);

		std::vector<point_3d> normals{ points[i + 1], r_p };
		geode->addChild(add_arrow(normals));
	}
}

float vector_cos(const Eigen::Vector3f & v1, const Eigen::Vector3f &v2)
{
	float
		dot_value = v1.dot(v2),
		lenSq1 = v1.norm(),
		lenSq2 = v2.norm();

	return dot_value / (lenSq1 * lenSq2);
}

osg::ref_ptr<osg::Geode> add_arrow(std::vector<point_3d> & arrow_points)
{
	point_3d & beg_point = arrow_points[0], end_point = arrow_points[1];

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	// line
	osg::ref_ptr<osg::Geometry> g_line = new osg::Geometry();
	points_to_geometry_node(arrow_points, g_line, 255, 0, 0);
	g_line->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, arrow_points.size()));
	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(3);
	g_line->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);

	//cone 
	osg::ref_ptr<osg::ShapeDrawable> sc = new osg::ShapeDrawable;
	sc->setShape(new osg::Cone(osg::Vec3(0.0f, 0.0f, 0.0), 0.5, 2));
	sc->setColor(osg::Vec4(255, 0, 0, 1.0f));

	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform(osg::Matrix::identity());
	osg::Matrix mRotate(osg::Matrix::identity()), mTrans(osg::Matrix::identity());
	mTrans.makeTranslate(osg::Vec3f(end_point.x, end_point.y, end_point.z));
	mRotate.makeRotate(
		osg::Vec3f(osg::Z_AXIS),
		osg::Vec3f(end_point.x - beg_point.x, end_point.y - beg_point.y, end_point.z - beg_point.z));

	mt->setMatrix(mRotate*mTrans);

	mt->addChild(sc);

	geode->addChild(mt);
	geode->addChild(g_line);

	return geode;
}

void points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float r, float g, float b, float w)
{
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();

	points_to_osg_structure(points, coords, colors, normals, r, g, b, w);

	// use color of each point
	if (r == 0 && g == 0 && b == 0)
	{
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}
	else
	{
		geometry->setColorArray(colors.get());
		geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	}

	geometry->setVertexArray(coords.get());

	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
}

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> colors, osg::ref_ptr<osg::Vec3Array> normals, float r, float g, float b, float w)
{
	// use point's color 
	if (r == 0 && g == 0 && b == 0)
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));

			colors->push_back(osg::Vec4(points[i].r, points[i].g, points[i].b, w));
		}
	}
	// use specific color
	else
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			coords->push_back(osg::Vec3(points[i].x, points[i].y, points[i].z));
		}
		colors->push_back(osg::Vec4(r, g, b, w));
	}

	normals->push_back(osg::Vec3(0.0f, 1.0f, 0.0f));
}

void point_along_with_vector_within_dis(point_3d & point, const Eigen::Vector3f & line_dir, point_3d & result_p, float length)
{
	result_p.x = length * vector_cos(Eigen::Vector3f::UnitX(), line_dir) + point.x;
	result_p.y = length * vector_cos(Eigen::Vector3f::UnitY(), line_dir) + point.y;
	result_p.z = length * vector_cos(Eigen::Vector3f::UnitZ(), line_dir) + point.z;
}