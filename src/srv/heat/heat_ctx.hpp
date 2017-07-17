#pragma once

#include <memory>

#include <exemodel/timeree.hpp>

#include "fsconf.h"

#include "hw_info.hpp"

#include "heat_cmm.hpp"

#include "device/heater.hpp"
#include "device/bi_color_led.hpp"
#include "device/hall_switch.hpp"

namespace svcHeat {

class heat_ctx {
public:
	heat_ctx(const hw_info & hwinfo);
public:
	exemodel::monotonic_timeree m_check_tmr;
	exemodel::monotonic_timeree m_heat_tmr;
	std::unique_ptr< hall_switch>	m_cover;

	heater		m_heater;
	bi_color_led	m_led;

	cfg_data_t	m_cfg;
};

}
