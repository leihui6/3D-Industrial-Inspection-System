//#define BACK_LOCALTEST
#ifdef BACK_LOCALTEST

#include "BackProcess.h"

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cout << "please insert configuration file as argument value." << std::endl;
		return -1;
	}

	BackProcess* back_process_p = new BackProcess;

	std::string configuration_file_name = std::string(argv[1]);

	//while (1)
	//{
	back_process_p->initial_parameter(configuration_file_name);

	back_process_p->registration();

	back_process_p->searching();

	back_process_p->measurement();

	back_process_p->evaluation();
	//}

	return 0;
}

#endif // !BACK_LOCALTEST
