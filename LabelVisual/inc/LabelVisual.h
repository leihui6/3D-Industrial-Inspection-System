#ifndef LABELVISUAL_H
#define LABELVISUAL_H

#include "LabelVisualCom.h"

#include "header.h"

#include "timeViewer.h"

class LabelVisual : public LabelVisualCom
{
public:
	LabelVisual();

	~LabelVisual();

	/*
	flag:
		0: visualize the labeled points "marked_points.txt" and "standard points"
		1: visualize the measurement result "measurement_result.txt" and "original points"
	*/
	virtual void initial(
		const std::string & file_1, // it usually is the standard or original points
		const std::string & file_2, // it usually is the marked or measurement points
		int flag
	);

	virtual void visual();

private:
	std::vector<std::vector<point_3d>> step_points_vec;

	void cal_centroid_point(std::vector<point_3d> & points, point_3d & centroid_point);

	void read_labelfile(const std::string & filename,
		std::vector<point_3d> & all_points, std::map<std::string, point_3d > & label_points,
		std::vector<point_3d> & property_content, bool step_subpoints);

	void window_initilization(osg::ref_ptr<timeViewer> & viewer, osg::ref_ptr<osg::Group> & root);

	void text_setupProperties(osgText::Text& textObject, osgText::Font* font, float size, const osg::Vec3& pos);

	void text_createContent(osgText::Text& textObject, const std::string & text/*char* string*/);

	rgb rgbstr2rgb(std::string rgbstr);

	float str2float(std::string num);

	float str2int(std::string num);

	void read_config(const std::string config_filename);

	float distance(point_3d & p1, point_3d & p2);

	void make_order(std::vector<std::vector<point_3d>> & step_points_vec);

	void make_text_node(point_3d & position, osg::ref_ptr<osg::Geode> geode_label_points, const std::string & text);

	void make_points_node(std::vector<point_3d> & points, osg::ref_ptr<osg::Geode> & geode, point_render_parameters & parameters);

	void make_normals_node(point_3d & point, Eigen::Vector3f &v, osg::ref_ptr<osg::Geode> &geode, point_render_parameters & parameters);

private:
	osg::ref_ptr<osg::Group> m_root;
	// all points are added on this node
	osg::ref_ptr<osg::Geode> m_geode;

	osg::ref_ptr < osgText::Font> m_fontKai;

	std::vector<std::string> filename_vec;
	std::vector <rgb> cloud_color_vec;
	std::vector<float> cloud_size_vec;
	std::vector<int> display_step_vec;
};

#endif // !LABELVISUAL_H

