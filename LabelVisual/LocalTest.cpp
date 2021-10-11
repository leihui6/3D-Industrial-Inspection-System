//#define MAKEDLL
#ifdef MAKEDLL

#include "inc/LabelVisual.h"

int main()
{
	LabelVisual * label_visual_p = new LabelVisual;

	label_visual_p->initial_label_info("visual_config_labeling.txt");

	label_visual_p->visual_label();

	return 0;
}

#endif // !MAKEDLL
