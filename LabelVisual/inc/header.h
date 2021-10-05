#pragma once

#include <windows.h>

#include <ctime>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

struct rgb { rgb() {} rgb(float _r, float _g, float _b) :r(_r), g(_g), b(_b) {} float r, g, b; };

struct point_3d
{
	point_3d() : x(0), y(0), z(0) {}

	point_3d(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	float x, y, z;
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

extern void make_geometry_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geometry> & geometry, point_render_parameters & parameters);