#ifndef APP_WELDING
#define APP_WELDING

#include "common_use.h"

class app_welding
{
public:
	app_welding();

	~app_welding();

	void process(std::vector<measurement_content> & measurement_result_vec);

private:

};

#endif // APP_WELDING
