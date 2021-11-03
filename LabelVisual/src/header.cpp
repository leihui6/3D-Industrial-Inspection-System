#include "header.h"

float vector_cos(const Eigen::Vector3f & v1, const Eigen::Vector3f &v2)
{
	float
		dot_value = v1.dot(v2),
		lenSq1 = v1.norm(),
		lenSq2 = v2.norm();

	return dot_value / (lenSq1 * lenSq2);
}

osg::ref_ptr<osg::Geode> add_arrow(std::vector<point_3d> & arrow_points, float r, float g, float b)
{
	point_3d & beg_point = arrow_points[0], end_point = arrow_points[1];

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	// line
	osg::ref_ptr<osg::Geometry> g_line = new osg::Geometry();
	points_to_geometry_node(arrow_points, g_line, r, g, b, 1.0);
	g_line->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, arrow_points.size()));
	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(2);
	g_line->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);
	g_line->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	//cone 
	osg::ref_ptr<osg::ShapeDrawable> sc = new osg::ShapeDrawable;
	sc->setShape(new osg::Cone(osg::Vec3(0.0f, 0.0f, 0.0), 0.8, 2));
	sc->setColor(osg::Vec4(r, g, b, 1.0f));

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