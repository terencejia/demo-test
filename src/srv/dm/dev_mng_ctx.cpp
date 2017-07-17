#include "dev_mng_ctx.hpp"

namespace svcDevMng {

dev_mng_ctx::dev_mng_ctx(const hw_info & hwinfo)
: m_pressure(hwinfo.pressure_path)
, m_humidity(hwinfo.humidity_path ? new exemodel::dev_attr_adv_ro<long, 1000>(hwinfo.humidity_path) : nullptr)
, m_int_temp(hwinfo.internal_temp_path)
, m_env_temp(hwinfo.env_temp_path)
, m_heat_temp(hwinfo.heater_temp_path)
, m_lcd_brightness(hwinfo.lcd_brightness_path)
{
}

}
