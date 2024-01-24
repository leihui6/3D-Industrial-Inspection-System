// TODO: This function is planned to be replaced by Open3D
// Ref: https://github.com/isl-org/Open3D/issues/3894
#define LOCALTEST
#ifdef LOCALTEST

#include "inc/LabelVisual.h"

int main(int argc, char* argv[])
{
	LabelVisual * label_visual_p = new LabelVisual;

	if (argc < 3)
	{
		std::cout << "please input arguments such like, filename1, filename2, flag" << std::endl;
		std::cout << "flag is 0: visualize the standard point cloud and marked points. For example, data/medical_blade_standard.txt output/marked_points_searched.txt 0" << std::endl;
		std::cout << "flag is 1: visualize the original point cloud and inspection results. For example, data/medical_blade_03.txt output/measurement_result.txt 1" << std::endl;

		return -1;
	}
	std::string filename1, filename2, flag;

	filename1 = std::string(argv[1]);
	filename2 = std::string(argv[2]);
	flag = std::string(argv[3]);

	std::cout << "input arguments: " << filename1 << " " << filename2 << " " << flag << std::endl;

	//while (1)
	//{
		//label_visual_p->initial("data/medical_blade_03.txt", "output/marked_points_searched.txt", 0);
		//label_visual_p->visual();

		//label_visual_p->initial("data/medical_blade_03.txt", "output/measurement_result.txt", 1);
		//label_visual_p->visual();
	//}
	label_visual_p->initial(filename1, filename2, std::stoi(flag));
	label_visual_p->visual();

	
	return 0;
}

#endif // !LOCALTEST
