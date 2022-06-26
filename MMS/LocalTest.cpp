#define BACK_LOCALTEST
#ifdef BACK_LOCALTEST

#include "BackProcess.h"

int main() 
{
	BackProcess * back_process_p = new BackProcess;

	while (1)
	{
		back_process_p->initial_parameter("data/3d_measurement_configuration.txt");

		back_process_p->registration();
	
		back_process_p->searching();
		
		back_process_p->measurement();
		
		back_process_p->evaluation();
	}

	return 0;
}

#endif // !BACK_LOCALTEST
