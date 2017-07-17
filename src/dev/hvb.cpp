#include <exemodel/poll_tools.hpp>
#include <fs_log.hpp>
#include <plf_drv/hvb_dev.h>

#include "fs_spec.hpp"

#include "device/hvb.hpp"

hvb::hvb(const char * path, const char * switch_path)
: m_val(path)
, m_switch(switch_path)
, m_running(false)
, m_discharge_count(0)
, m_magnitude(0.0)
{
}

hvb::~hvb()
{
}

void hvb::start()
{
	++m_discharge_count;
	m_switch.enable();
	m_running = true;
	return;
}

void hvb::stop()
{
	m_switch.disable();
	m_running = false;
	return;

}

bool hvb::is_running() const
{
	return m_running;
}

void hvb::set_magnitude(double magnitude)
{
	static constexpr double max_volt = 5;
	static constexpr uint16_t max_raw = 1023;
	const uint16_t raw_mag = static_cast<uint16_t>(magnitude / max_volt * max_raw);
	if (raw_mag > max_raw) {
		throw std::invalid_argument("hvb magnitude too big");
	}

	m_val.write(raw_mag);
	m_magnitude = magnitude;

	log_debug("set hvb magnitude as %f", magnitude);
	return;
}

double hvb::get_magnitude(void) const
{
	return m_magnitude;
}

void hvb::reset_count(void)
{
	m_discharge_count = 0;
}

uint32_t hvb::get_count(void) const
{
	return m_discharge_count;
}
