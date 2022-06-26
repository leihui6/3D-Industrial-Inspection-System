#ifndef LABELVISUALCOM_H
#define LABELVISUALCOM_H

#include <vector>
#include <string>

class LabelVisualCom
{
public:
	LabelVisualCom() {}

	virtual ~LabelVisualCom() {}

	virtual void initial(const std::string & file_1, const std::string & file_2, int flag) = 0;

	virtual void visual() = 0;
};

#endif
