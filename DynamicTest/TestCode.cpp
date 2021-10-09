#include "LabelVisualCom.h"
#include "LabelVisualExp.h"

#include "BackProcessCom.h"
#include "BackProcessExp.h"
#include <windows.h>

#define test_visual
//#define test_backprocess

int main()
{
#ifdef test_visual
	HINSTANCE hDll = LoadLibrary("../LabelVisual/x64/Release/LabelVisual.dll");

	if (!hDll)
	{
		printf("error 1");
	}

	while (1)
	{
		typedef LabelVisualCom*(*LabelVisualComfunc)();
		LabelVisualComfunc dllFunc = (LabelVisualComfunc)GetProcAddress(hDll, "getLabelVisualCom");

		if (!dllFunc)
		{
			printf("error 2");
		}

		const std::string filename = "visual_config_labeling.txt";
		LabelVisualCom * label_visual_p;
		label_visual_p = (LabelVisualCom*)dllFunc();

		label_visual_p->initial_label_info(filename);
		label_visual_p->visual_label();
	}
	FreeLibrary(hDll);
#endif // test_visual


#ifdef test_backprocess
	HINSTANCE hDll = LoadLibrary("../MMS/x64/Release/CloudProcess.dll");

	if (!hDll)
	{
		printf("error 1");
	}

	typedef BackProcessCom*(*BackProcessComfunc)();
	while (1)
	{
		BackProcessComfunc dllFunc = (BackProcessComfunc)GetProcAddress(hDll, "getBackProcessCom");

		if (!dllFunc)
		{
			printf("error 2");
		}

		const std::string filename = "3d_measurement_configuration.txt";
		BackProcessCom * back_process_p;
		back_process_p = (BackProcessCom*)dllFunc();

		back_process_p->initial_parameter(filename);

		back_process_p->registration();

		back_process_p->searching();

		back_process_p->measurement();

		delete back_process_p;
	}
	FreeLibrary(hDll);
#endif // test_backprocess
	
	return 0;
}
