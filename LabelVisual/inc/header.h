#ifndef HEADER_H
#define HEADER_H

#include <windows.h>

#include <ctime>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <OSG/Vec3d>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osgUtil/SceneView>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Renderer>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer> 
#include <osgUtil/DelaunayTriangulator> 
#include <osgGA/TrackballManipulator>
#include <osg/ComputeBoundsVisitor>

struct rgb { rgb() {} rgb(float _r, float _g, float _b) :r(_r), g(_g), b(_b) {} float r, g, b; };

struct point_3d
{
	point_3d() : x(0), y(0), z(0), r(0), g(0), b(0) {}

	point_3d(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	float x, y, z;
	float r, g, b;
};

// only support for size and color
class point_render_parameters
{
public:
	point_render_parameters()
		:color(0.0f, 0.0f, 0.0f, 1.0f), normal(0.0f, 0.0f, 0.0f)
	{
		point_size = 1;
	}

	void set_point_size(float size) { this->point_size = size; }

	// [0:255]
	void set_point_color(rgb _rgb) { color = osg::Vec4(_rgb.r, _rgb.g, _rgb.b, 1.0f); }

	float get_point_size() { return this->point_size; }

	osg::Vec4 get_color() { return this->color; }

	osg::Vec3 get_normal() { return this->normal; }
private:
	float point_size;
	osg::Vec4 color;
	osg::Vec3 normal;
};

extern void make_points_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> & geometry, point_render_parameters & parameters);

extern void make_normals_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geode> &geode, point_render_parameters & parameters);

// these funtions are copied from CloudProcess
extern void point_along_with_vector_within_dis(point_3d & point, const Eigen::Vector3f & line_dir, point_3d & result_p, float length);

extern float vector_cos(const Eigen::Vector3f & v1, const Eigen::Vector3f &v2);

extern osg::ref_ptr<osg::Geode> add_arrow(std::vector<point_3d> & arrow_points);

extern void points_to_geometry_node(std::vector<point_3d>& points, osg::ref_ptr<osg::Geometry> geometry, float r, float g, float b, float w=1);

void points_to_osg_structure(std::vector<point_3d>& points, osg::ref_ptr<osg::Vec3Array> coords, osg::ref_ptr<osg::Vec4Array> colors, osg::ref_ptr<osg::Vec3Array> normals, float r, float g, float b, float w);
// end



#endif // !HEADER_H

