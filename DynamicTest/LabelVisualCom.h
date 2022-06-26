#ifndef LABELVISUALCOM_H
#define LABELVISUALCOM_H

#include<vector>
#include<string>

class LabelVisualCom
{
public:
	LabelVisualCom() {}

	virtual ~LabelVisualCom() {}

	virtual void initial_label_info(const std::string & config_filename) = 0;

	virtual void visual_label() = 0;
};

#endif
