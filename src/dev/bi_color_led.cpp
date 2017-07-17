#include <fs_log.hpp>

#include "device/bi_color_led.hpp"

void open_led(exemodel::simple_file_rw & dev)
{
	static const char on_str[] = "1";
	ssize_t ret = dev.write(&on_str, sizeof(on_str));
	if (ret < 0) {
		log_err("open led '%s' fail", dev.path());
		throw;
	}
}

void close_led(exemodel::simple_file_rw & dev)
{
	static const char off_str[] = "0";
	ssize_t ret = dev.write(&off_str, sizeof(off_str));
	if (ret < 0) {
		log_err("close led '%s' fail", dev.path());
		throw;
	}
}

bi_color_led::bi_color_led(
	const char* red_led_path,
	const char* green_led_path)
: m_red(red_led_path)
, m_green(green_led_path)
, m_state(bi_color_state::off)
{
	close_led(m_red);
	close_led(m_green);
}

bi_color_led::~bi_color_led()
{
}

void bi_color_led::set(const bi_color_state new_state)
{
	if (m_state == new_state) {
		return;
	}

	switch (m_state) {
	case bi_color_state::green:
		close_led(m_green);
		break;
	case bi_color_state::red:
		close_led(m_red);
		break;
	default:
		break;
	}

	switch (new_state) {
	case bi_color_state::green:
		open_led(m_green);
		break;
	case bi_color_state::red:
		open_led(m_red);
		break;
	default:
		break;
	}

	m_state = new_state;
}
