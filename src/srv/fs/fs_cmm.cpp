#include "fs_spec.hpp"
#include "fs_cmm.hpp"

#include "img_process.hpp"

namespace svcFS {

fs_devs::fs_devs(const hw_info & hwinfo)
: m_hwinfo(hwinfo)
, m_camera(hwinfo)
, m_ledX(hwinfo.x_led_path)
, m_ledY(hwinfo.y_led_path)
, m_ledL(hwinfo.lighting_led_path)
, m_motorLZ(hwinfo.lz_motor_path ? new motor(hwinfo.lz_motor_path, hwinfo.push_motor_speed_ratio) : nullptr)
, m_motorRZ(hwinfo.rz_motor_path ? new motor(hwinfo.rz_motor_path, hwinfo.push_motor_speed_ratio) : nullptr)
, m_motorX(hwinfo.x_motor_path ? new motor(hwinfo.x_motor_path) : nullptr)
, m_motorY(hwinfo.y_motor_path ? new motor(hwinfo.y_motor_path) : nullptr)
, m_hvb(hwinfo.hvb_mag_path, hwinfo.hvb_switch_path)
, m_peLZ(hwinfo.lz_end_path)
, m_peRZ(hwinfo.rz_end_path)
, m_cover(hwinfo.cover_path)
, m_display(hwinfo.display_path, static_cast<uint16_t>(hwinfo.cmos_win_width), static_cast<uint16_t>(hwinfo.cmos_win_height))
, m_x_focus(hwinfo.x_liquid_lens_path ? new exemodel::dev_attr_nor_rw<long>(hwinfo.x_liquid_lens_path, hwinfo.x_liquid_lens_max_val) : nullptr)
, m_y_focus(hwinfo.y_liquid_lens_path ? new exemodel::dev_attr_nor_rw<long>(hwinfo.y_liquid_lens_path, hwinfo.y_liquid_lens_max_val) : nullptr)
, m_x_exposure(::access(hwinfo.x_cmos_exposure_path, F_OK) == 0 ? new exemodel::dev_attr_rw<long>(hwinfo.x_cmos_exposure_path) : nullptr)
, m_y_exposure(::access(hwinfo.y_cmos_exposure_path, F_OK) == 0 ? new exemodel::dev_attr_rw<long>(hwinfo.y_cmos_exposure_path) : nullptr)
, m_env_temp(hwinfo.env_temp_path)
, m_env_pressure(hwinfo.pressure_path)
{
	/// \note: use fixed brightness for led control, don't change it in any time.
	m_ledX.set_brightness(1.0);
	m_ledY.set_brightness(1.0);

	this->stop();
	this->disable();

	bool cover_openned = m_cover.get_state();
	if (cover_openned) {
		m_ledX.disable();
		m_ledY.disable();
		m_ledL.enable();
	}
	else {
		m_ledX.enable();
		m_ledY.enable();
		m_ledL.disable();
	}
}

fs_devs::~fs_devs()
{
	m_ledL.disable();
	m_ledX.disable();
	m_ledY.disable();

	this->stop();
}

void fs_devs::reset(const fs_spec & spec)
{
	if(m_x_exposure)
		m_x_exposure->write(static_cast<long>(spec.led_brightness[ledId_t::CMOS_X] * m_hwinfo.x_cmos_exposure_max));
	if(m_y_exposure)
		m_y_exposure->write(static_cast<long>(spec.led_brightness[ledId_t::CMOS_Y] * m_hwinfo.y_cmos_exposure_max));

	m_camera.set_window_pos_x(spec.window_x_col, spec.window_x_row);
	m_camera.set_window_pos_y(spec.window_y_col, spec.window_y_row);

	if (m_x_focus) {
		m_x_focus->write(spec.x_focal_distance);
	}
	if (m_y_focus) {
		m_y_focus->write(spec.y_focal_distance);
	}

	if (m_motorLZ) {
		m_motorLZ->set_speed_limit(
			spec.motor_max_speed[motorId_t::LZ],
			spec.motor_min_speed[motorId_t::LZ]);
	}
	if (m_motorRZ) {
		m_motorRZ->set_speed_limit(
			spec.motor_max_speed[motorId_t::RZ],
			spec.motor_min_speed[motorId_t::RZ]);
	}
	if (m_motorX) {
		m_motorX->set_speed_limit(
			spec.motor_max_speed[motorId_t::X],
			spec.motor_min_speed[motorId_t::X]);
	}
	if (m_motorY) {
		m_motorY->set_speed_limit(
			spec.motor_max_speed[motorId_t::Y],
			spec.motor_min_speed[motorId_t::Y]);
	}
}

void fs_devs::enable()
{
	if (m_motorLZ) {
		m_motorLZ->enable();
	}
	if (m_motorRZ) {
		m_motorRZ->enable();
	}
	if (m_motorX) {
		m_motorX->enable();
	}
	if (m_motorY) {
		m_motorY->enable();
	}
}

void fs_devs::disable()
{
	if (m_motorLZ) {
		m_motorLZ->disable();
	}
	if (m_motorRZ) {
		m_motorRZ->disable();
	}
	if (m_motorX) {
		m_motorX->disable();
	}
	if (m_motorY) {
		m_motorY->disable();
	}
	//m_hvb.disable();
}

void fs_devs::stop()
{
	if (m_motorLZ) {
		m_motorLZ->stop();
	}
	if (m_motorRZ) {
		m_motorRZ->stop();
	}
	if (m_motorX) {
		m_motorX->stop();
	}
	if (m_motorY) {
		m_motorY->stop();
	}
	m_hvb.stop();
}

fs_auxs::fs_auxs()
: m_discharge_tmr()
, m_hvb_pwm()
, m_cone_waiter()
, m_mt_tmr()
, m_push_timeout_tmr()
, m_stair_tmr()
, m_user_tmr()
, m_cover_delay_tmr()
{
}

void fs_auxs::reset(const fs_spec & spec)
{
	m_push_timeout_tmr.set_spec({ { 0, 0 }, exemodel::ms_to_timespec(
				spec.moter_push_timeout_time()) });

	m_cover_delay_tmr.set_spec({ { 0, 0 }, exemodel::ms_to_timespec(spec.cover_delay_time) });

}

void fs_auxs::stop()
{
	m_discharge_tmr.stop();
	m_hvb_pwm.stop();

	m_cone_waiter.stop();

	///\note only use for motor test
	m_mt_tmr.stop();

	m_stair_tmr.stop();
	m_user_tmr.stop();
	m_cover_delay_tmr.stop();
}

} /* namespace svcFS */
