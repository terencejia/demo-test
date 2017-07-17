#include "fsconf.h"

#include "heat_ctx.hpp"

namespace svcHeat {

heat_ctx::heat_ctx(const hw_info & hwinfo)
: m_check_tmr({{0,10*1000*1000}, {0,10*1000*1000}})	/// \todo fix the check cycle
, m_heat_tmr()
, m_cover(hwinfo.heater_cover_path ? new hall_switch(hwinfo.heater_cover_path) : nullptr)
, m_heater(hwinfo.slow_heater_path, hwinfo.fast_heater_path, hwinfo.heater_temp_path)
, m_led(hwinfo.heater_green_led_path, hwinfo.heater_red_led_path)
, m_cfg()
{
}

}
