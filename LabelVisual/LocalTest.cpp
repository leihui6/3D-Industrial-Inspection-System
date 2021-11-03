#define MAKEDLL
#ifdef MAKEDLL

#include "inc/LabelVisual.h"

int main()
{
	LabelVisual * label_visual_p = new LabelVisual;

	label_visual_p->initial("data/medical_blade_standard.txt", "output/marked_points.txt", 0);

	label_visual_p->visual();

	return 0;
}

#endif // !MAKEDLL
