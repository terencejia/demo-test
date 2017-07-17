#pragma once

#include <memory>

#include <exemodel/timeree.hpp>
#include <exemodel/dev_attr.hpp>

#include "fsconf.h"
#include "hw_info.hpp"

namespace svcDevMng {

class dev_mng_ctx final {
public:
	dev_mng_ctx(const hw_info & hwinfo);
public:
	exemodel::dev_attr_adv_ro<long, 100000> m_pressure;
	std::unique_ptr< exemodel::dev_attr_adv_ro<long, 1000> > m_humidity;
	exemodel::dev_attr_adv_ro<long, 1000> m_int_temp;
	exemodel::dev_attr_adv_ro<long, 1000> m_env_temp;
	exemodel::dev_attr_ro<long> m_heat_temp;
	exemodel::dev_attr_adv_rw<long, 1000> m_lcd_brightness;
};

}
