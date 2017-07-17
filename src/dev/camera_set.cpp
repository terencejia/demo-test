#include "device/camera_set.hpp"

camera_set::camera_set(const hw_info & hwinfo)
: m_camera(hwinfo)
{
	this->add(m_camera);
}

camera_set::~camera_set()
{
	this->del(m_camera);
}

void camera_set::push_waiter(const camera::waiter_t waiter, uint32_t delay_ms, cmosId_t cmosid)
{
	m_camera.push_waiter(waiter, delay_ms);
	///\todo add waiter
	switch (cmosid) {
		case cmosId_t::X:
			break;
		case cmosId_t::Y:
			break;
		default:
			break;
	}
}

void camera_set::pop_waiter(const camera::waiter_t waiter, cmosId_t cmosid)
{
	m_camera.pop_waiter(waiter);
	///\todo del waiter
	switch (cmosid) {
		case cmosId_t::X:
			break;
		case cmosId_t::Y:
			break;
		default:
			break;
	}
}

void camera_set::push_waiter_video(const camera::waiter_t waiter)
{
	m_camera.push_waiter_video(waiter);
	///\todo add waiter
}

void camera_set::pop_waiter_video(const camera::waiter_t waiter)
{
	m_camera.pop_waiter_video(waiter);
	///\todo del waiter
}

void camera_set::set_window_pos_x(int32_t c, int32_t r)
{
	m_camera.set_window_pos_x(c, r);
}

void camera_set::set_window_pos_y(int32_t c, int32_t r)
{
	m_camera.set_window_pos_y(c, r);
}

void camera_set::get_window_pos_x(int32_t & c, int32_t & r)
{
	m_camera.get_window_pos_x(c, r);
}

void camera_set::get_window_pos_y(int32_t & c, int32_t & r)
{
	m_camera.get_window_pos_y(c, r);
}

void camera_set::set_window_size(int32_t w, int32_t h)
{
	m_camera.set_window_size(w, h);
}

void camera_set::move_window_pos_x(int32_t c, int32_t r)
{
	m_camera.move_window_pos_x(c, r);
}

void camera_set::move_window_pos_y(int32_t c, int32_t r)
{
	m_camera.move_window_pos_y(c, r);
}

void camera_set::dump()
{
	m_camera.dump();
}
