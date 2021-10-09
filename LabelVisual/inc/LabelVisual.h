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

	virtual void initial_label_info(const std::string & config_filename);

	virtual void visual_label();

private:
	std::vector<std::vector<point_3d>> step_points_vec;

	void cal_centroid_point(std::vector<point_3d> & points, point_3d & centroid_point);

	void read_labelfile(const std::string & filename, std::vector<point_3d> & all_points, std::map<std::string, point_3d > & label_points, bool step_subpoints);

	void window_initilization(osg::ref_ptr<timeViewer> & viewer, osg::ref_ptr<osg::Group> & root);

	void setupProperties(osgText::Text& textObject, osgText::Font* font, float size, const osg::Vec3& pos);

	void createContent(osgText::Text& textObject, const std::string & text/*char* string*/);

	rgb rgbstr2rgb(std::string rgbstr);

	float str2float(std::string num);

	float str2int(std::string num);

	void read_config(const std::string config_filename);

	float distance(point_3d & p1, point_3d & p2);

	void make_order(std::vector<std::vector<point_3d>> & step_points_vec);

private:
	osg::ref_ptr<osg::Group> root;
	osg::ref_ptr<osg::Geode> geode;

	std::vector<std::string> filename_vec;
	std::vector <rgb> cloud_color_vec;
	std::vector<float> cloud_size_vec;
	std::vector<int> display_step_vec;
};

#endif // !LABELVISUAL_H

