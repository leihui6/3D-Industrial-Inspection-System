#include "interface_command.h"

interface_command::interface_command(cloud_viewer * cloud_view_ptr)
	:m_cs(CS_NORMAL),
	m_dt(DT_EMPTY)
{
	m_cloud_view_ptr = cloud_view_ptr;

	m_cloud_view_ptr->set_the_interface_command(this);

	marked_points_count.resize(5, 0);
}


interface_command::~interface_command()
{
}

void interface_command::run()
{
	boost::thread run_thread(boost::bind(&interface_command::run_itself, this));

	run_thread.detach();
}

void interface_command::run_itself()
{
	char i_c;

	while (true)
	{
		print_menu();

		std::cin >> i_c;

		if (i_c == 'q')
		{
			std::cout << "exited labeling process manually" << std::endl;
			m_cs = CS_QUIT;
			m_dt = DT_EMPTY;

			exit(0);
		}
		// points
		else if (i_c == 'a')
		{
			print_menu_in_steps(DT_POINT, 0);
			process_specific_type(DT_POINT);
			print_menu_in_steps(DT_POINT, 1);
		}
		// line
		else if (i_c == 'b')
		{
			print_menu_in_steps(DT_LINE, 0);
			process_specific_type(DT_LINE);
			print_menu_in_steps(DT_LINE, 1);
		}
		// plane
		else if (i_c == 'c')
		{
			print_menu_in_steps(DT_PLANE, 0);
			process_specific_type(DT_PLANE);
			print_menu_in_steps(DT_PLANE, 1);
		}
		// cylinder
		else if (i_c == 'd')
		{
			print_menu_in_steps(DT_CYLINDER, 0);
			process_specific_type(DT_CYLINDER);
			print_menu_in_steps(DT_CYLINDER, 1);
		}
		// reference points
		else if (i_c == 'r')
		{
			print_menu_in_steps(DT_REFER, 0);
			process_specific_type(DT_REFER);
			print_menu_in_steps(DT_REFER, 1);
		}
		// end for c
		else if (i_c == 's')
		{
			m_cloud_view_ptr->print_marked_info();
		}
		else if (i_c == 'e')
		{
			m_cloud_view_ptr->export_data();
		}
	}
}

void interface_command::print_menu()
{
	std::cout
		<< "######## MENU ########\n"
		<< "please choice the shape that you want to detect, press \"q\" to quit" << "\n"
		<< "a) 3d point" << "\n"
		<< "b) 3d line" << "\n"
		<< "c) 3d plane" << "\n"
		<< "d) 3d cylinder" << "\n"
		<< "r) pick reference points" << "\n"
		<< "s) show all shapes marked" << "\n"
		<< "e) export all marked points" << "\n"
		<< "choose an option:";
}

void interface_command::print_menu_in_steps(detected_type dt, int flag)
{
	// point
	if (dt == DT_POINT && flag == 0)
	{
		std::cout
			<< "you choosed  \"POINT\" type, please select the points in the scene\n"
			<< "you can press 'q' to quit current step if you are done!. \n";
	}
	else if (dt == DT_POINT && flag == 1)
	{
		std::cout << "quit marking the points step.\n";
	}
	
	// line
	if (dt == DT_LINE && flag == 0)
	{
		std::cout
			<< "you choosed  \"LINE\" type, please select the points in the scene\n"
			<< "you can press 'q' to quit current step if you are done!. \n";

	}
	else if (dt == DT_LINE && flag == 1)
	{
		std::cout << "quit marking the lines step.\n";
	}

	// plane
	if (dt == DT_PLANE && flag == 0)
	{
		std::cout
			<< "you choosed  \"PLANE\" type, please select the points in the scene\n"
			<< "you can press 'q' to quit current step if you are done!. \n";

	}
	else if (dt == DT_PLANE && flag == 1)
	{
		std::cout << "quit marking the planes step.\n";
	}

	// cylinder
	if (dt == DT_CYLINDER && flag == 0)
	{
		std::cout
			<< "note that before marking cylinder, please mark the bottom plane of cylinder, i.e., picking points representing a plane.\n"
			<< "points building a cylinder will be updated in real time, please check if it is a good representation\n"
			<< "(white) points mean picked points\n"
			<< "(green) plane means the bottom of cylinder\n"
			<< "(black) point means the center point on bottom of cylinder\n"
			<< "(blue) points on cylinder, please check it \n"
			<< "you choosed  \"CYLINDER\" type, please select the points in the scene\n"
			<< "you can press 'q' to quit current step if you are done!. \n";
	}
	else if (dt == DT_CYLINDER && flag == 1)
	{
		std::cout << "quit marking the cylinders step.\n";
	}

	if (dt == DT_REFER && flag == 0)
	{
		std::cout
			<< "you choosed  \"REFERENCE POINTS\" type, please select the points in the scene\n"
			<< "you can press 'q' to quit current step if you are done!. \n";

	}
	else if (dt == DT_REFER && flag == 1)
	{
		std::cout << "quit marking the planes step.\n";
	}

}

void interface_command::clear_picked_points()
{
	m_cloud_view_ptr->clear_picked_points();
}

void interface_command::clear_shapes()
{
	m_cloud_view_ptr->clear_shapes();
}

void interface_command::print_marked_info()
{
	m_cloud_view_ptr->print_marked_info();
}

void interface_command::input_marked_name(std::string & marked_name, const std::string  & default_name)
{
	std::string input;

	while (true)
	{
		std::cout << "please enter the name of these points:(default:" << default_name << ")";

		int c;
		while ((c = getchar()) != '\n' && c != EOF) {}

		std::getline(std::cin, input);

		if (input.empty())
		{
			marked_name = default_name;

			break;
		}
		else
		{
			marked_name = input;
		}
	}
}

void interface_command::process_specific_type(detected_type _dt)
{
	clear_picked_points();

	clear_shapes();

	m_cs = CS_SELECTING_POINTS;

	m_dt = _dt;

	char i_c;

	while (true)
	{
		std::cin >> i_c;

		if (i_c == 'q')
		{
			std::string marked_name;

			m_cs = CS_NORMAL;

			m_dt = DT_EMPTY;

			if (_dt == DT_POINT)
			{
				if (m_cloud_view_ptr->m_point.points.empty()) break;

				input_marked_name(marked_name, "point_" + std::to_string(marked_points_count[0]++));

				m_cloud_view_ptr->m_marked_points_map[marked_name] = m_cloud_view_ptr->m_point;

				//m_cloud_view_ptr->save_points_to_vec(marked_name, m_cloud_view_ptr->m_points);
			}
			else if (_dt == DT_LINE)
			{
				if (m_cloud_view_ptr->m_line.points.empty()) break;

				input_marked_name(marked_name, "line_" + std::to_string(marked_points_count[1]++));

				m_cloud_view_ptr->m_marked_points_map[marked_name] = m_cloud_view_ptr->m_line;

				//m_cloud_view_ptr->save_points_to_vec(marked_name, m_cloud_view_ptr->m_marked_points_vec);
			}
			else if (_dt == DT_PLANE)
			{
				if (m_cloud_view_ptr->m_plane.points.empty()) break;

				input_marked_name(marked_name, "plane_" + std::to_string(marked_points_count[2]++));

				m_cloud_view_ptr->m_marked_points_map[marked_name] = m_cloud_view_ptr->m_plane;

				//m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_plane.points, marked_name, m_cloud_view_ptr->m_marked_points_vec);
			}
			else if (_dt == DT_CYLINDER)
			{
				if (m_cloud_view_ptr->m_cylinder.points.empty()) break;

				input_marked_name(marked_name, "cylinder_" + std::to_string(marked_points_count[3]++));

				m_cloud_view_ptr->m_marked_points_map[marked_name] = m_cloud_view_ptr->m_cylinder;

				//m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_cylinder.points, marked_name, m_cloud_view_ptr->m_marked_points_vec);
			}
			else if (_dt == DT_REFER)
			{
				if (m_cloud_view_ptr->m_reference_point.points.empty()) break;

				input_marked_name(marked_name, "reference_" + std::to_string(marked_points_count[4]++));

				m_cloud_view_ptr->m_marked_points_map[marked_name] = m_cloud_view_ptr->m_reference_point;

				//m_cloud_view_ptr->save_points_to_vec(m_cloud_view_ptr->m_reference_points.points, marked_name, m_cloud_view_ptr->m_marked_points_vec);
			}

			break;
		}
		else if (i_c == 'c')
		{
			change_shape_property(_dt, 0);
		}

		else
		{
			// do nothing
		}
	}
}

void interface_command::change_shape_property(detected_type dt, int flag)
{
	if (dt = DT_PLANE)
	{
		std::cout << "Change plane property: the direction of normal" << std::endl;

		m_cloud_view_ptr->m_plane_property_flag = flag;
	}
}
