#ifndef APP_WELDING
#define APP_WELDING

#include "cloud_geometry.h"

class app_welding
{
public:
	app_welding();

	~app_welding();

	void process(std::vector<measurement_content> & measurement_result_vec);

private:
	void make_order(std::vector<measurement_content> & measurement_result_vec);

	void cut_off(std::vector<measurement_content> & measurement_result_vec);
};

#endif // APP_WELDING
