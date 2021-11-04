//#define LOCALTEST
#ifdef LOCALTEST

#include "inc/LabelVisual.h"

int main()
{
	LabelVisual * label_visual_p = new LabelVisual;

	while (1)
	{
		label_visual_p->initial("data/medical_blade_standard.txt", "output/marked_points.txt", 0);
		label_visual_p->visual();

		label_visual_p->initial("data/medical_blade_01.txt", "output/measurement_result.txt", 1);
		label_visual_p->visual();
	}
	

	return 0;
}

#endif // !LOCALTEST
