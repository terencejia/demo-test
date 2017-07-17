
#include <plf_drv/display_dev.h>

#include <exemodel/poll_tools.hpp>

#include "device/display.hpp"

#include <type_traits>

display::display(const char * path, const uint16_t win_width, const uint16_t win_height)
: m_f(path)
, m_win_width(win_width)
, m_win_height(win_height)
{
	this->zoom_init();
}

display::~display()
{
}

void display::set(const disp_data & data)
{
	disp_info info;

	info.disp_ctl = (data.cmos0_first ? 0x1 : 0x0);
	info.cmos0_disp_x = data.cmos0_disp_x;
	info.cmos0_disp_y = data.cmos0_disp_y;
	info.cmos0_disp_w = data.cmos0_disp_w;
	info.cmos0_disp_h = data.cmos0_disp_h;
	info.cmos1_disp_x = data.cmos1_disp_x;
	info.cmos1_disp_y = data.cmos1_disp_y;
	info.cmos1_disp_w = data.cmos1_disp_w;
	info.cmos1_disp_h = data.cmos1_disp_h;

	int ret = m_f.io_ctl(DISPLAY_IOC_SET_ALL, &info);
	exemodel::validate_ret(ret, "display set info");
}

void display::get(disp_data & data)
{
	disp_info info;

	int ret = m_f.io_ctl(DISPLAY_IOC_GET_ALL, &info);
	exemodel::validate_ret(ret, "display get info");

	data.cmos0_first = (info.disp_ctl & 0x1);
	data.cmos0_disp_x = info.cmos0_disp_x;
	data.cmos0_disp_y = info.cmos0_disp_y;
	data.cmos0_disp_w = info.cmos0_disp_w;
	data.cmos0_disp_h = info.cmos0_disp_h;
	data.cmos1_disp_x = info.cmos1_disp_x;
	data.cmos1_disp_y = info.cmos1_disp_y;
	data.cmos1_disp_w = info.cmos1_disp_w;
	data.cmos1_disp_h = info.cmos1_disp_h;
}

void display::set_zoom(const zoom_data & data)
{
	zoom_info info;

	info.cmos0_zoom_x = data.cmos0_zoom_x;
	info.cmos0_zoom_y = data.cmos0_zoom_y;
	info.cmos0_zoom_length = data.cmos0_zoom_length;
	info.cmos0_zoom_width = data.cmos0_zoom_width;
	info.cmos1_zoom_x = data.cmos1_zoom_x;
	info.cmos1_zoom_y = data.cmos1_zoom_y;
	info.cmos1_zoom_length = data.cmos1_zoom_length;
	info.cmos1_zoom_width = data.cmos1_zoom_width;

	int ret = m_f.io_ctl(DISPLAY_IOC_SET_ZOOM_ALL, &info);
	exemodel::validate_ret(ret, "display set zoom info");
}

void display::set_zoom(const single_zoom_data & data, bool is_x)
{
	zoom_data info;
	this->get_zoom(info);

	if (is_x) {
		info.cmos0_zoom_x = data.zoom_x;
		info.cmos0_zoom_y = data.zoom_y;
		info.cmos0_zoom_length = data.zoom_length;
		info.cmos0_zoom_width = data.zoom_width;
	} else {
		info.cmos1_zoom_x = data.zoom_x;
		info.cmos1_zoom_y = data.zoom_y;
		info.cmos1_zoom_length = data.zoom_length;
		info.cmos1_zoom_width = data.zoom_width;
	}

	int ret = m_f.io_ctl(DISPLAY_IOC_SET_ZOOM_ALL, &info);
	exemodel::validate_ret(ret, "display set single zoom info");
}


void display::get_zoom(zoom_data & data)
{
	zoom_info info;
	int ret = m_f.io_ctl(DISPLAY_IOC_GET_ZOOM_ALL, &info);
	exemodel::validate_ret(ret, "display set zoom info");

	data.cmos0_zoom_x = info.cmos0_zoom_x;
	data.cmos0_zoom_y = info.cmos0_zoom_y;
	data.cmos0_zoom_length = info.cmos0_zoom_length;
	data.cmos0_zoom_width = info.cmos0_zoom_width;
	data.cmos1_zoom_x = info.cmos1_zoom_x;
	data.cmos1_zoom_y = info.cmos1_zoom_y;
	data.cmos1_zoom_length = info.cmos1_zoom_length;
	data.cmos1_zoom_width = info.cmos1_zoom_width;
}

void display::zoom_init()
{
	this->set_zoom({0,0,m_win_width, m_win_height, 0,0,m_win_width, m_win_height});
}
