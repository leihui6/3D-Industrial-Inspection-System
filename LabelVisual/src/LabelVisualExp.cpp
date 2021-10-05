#include "LabelVisualExp.h"
#include "LabelVisual.h"

LabelVisualCom *getLabelVisualCom()
{
	LabelVisualCom* t_LabelVisualCom = new LabelVisual();
	return t_LabelVisualCom;
}

const char *getVersion()
{
	return "This is a msvc2017 dll! version= 0.1";
}