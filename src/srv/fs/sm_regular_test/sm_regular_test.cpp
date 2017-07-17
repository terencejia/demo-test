#include <zmsg/zmsg_discharge.hpp>
#include <zmsg/zmsg_fs_spec.hpp>
#include <zmsg/zmsg_cmos_focal.hpp>
#include <fs_log.hpp>

#include "img_process.hpp"

#include "sm_regular_test.hpp"

namespace svcFS {

namespace smRegularTest {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.rt.cfg)
, m_data(context<fs_sm>().m_ctx.rt.data)
, m_dt_x_ready(false)
, m_dt_y_ready(false)
, m_wave_x_ready(false)
, m_wave_y_ready(false)
{
	FS_DEV.enable();

	FS_DEV.m_motorLZ->disable();
	FS_DEV.m_motorRZ->disable();
}

stRunning::~stRunning()
{
	/// update spec
	FS_DEV.m_camera.get_window_pos_x(FS_SPEC.window_x_col, FS_SPEC.window_x_row);
	FS_DEV.m_camera.get_window_pos_y(FS_SPEC.window_y_col, FS_SPEC.window_y_row);
	FS_SPEC.led_brightness[ledId_t::CMOS_X] = static_cast<double>(FS_DEV.m_x_exposure->read()) / FS_HWINFO.x_cmos_exposure_max;
	FS_SPEC.led_brightness[ledId_t::CMOS_Y] = static_cast<double>(FS_DEV.m_y_exposure->read()) / FS_HWINFO.y_cmos_exposure_max;

	/// send msg to save config
	{
		DCL_ZMSG(update_window_position) msg;
		msg.is_pos_x = true;
		msg.row = FS_SPEC.window_x_row;
		msg.column = FS_SPEC.window_x_col;
		FS_REPORT(msg);
		msg.is_pos_x = false;
		msg.row = FS_SPEC.window_y_row;
		msg.column = FS_SPEC.window_y_col;
		FS_REPORT(msg);
	}

	{
		DCL_ZMSG(update_led_brightness) msg;
		msg.id = ledId_t::CMOS_X;
		msg.brightness = static_cast<double>(FS_DEV.m_x_exposure->read()) / FS_HWINFO.x_cmos_exposure_max;
		FS_REPORT(msg);
		msg.id = ledId_t::CMOS_Y;
		msg.brightness = static_cast<double>(FS_DEV.m_y_exposure->read()) / FS_HWINFO.y_cmos_exposure_max;
		FS_REPORT(msg);
	}

	DCL_ZMSG(discharge_count) msg;
	msg.discharge_count = FS_DEV.m_hvb.get_count();
	if (msg.discharge_count > 0) {
		FS_REPORT(msg);
	}

	FS_DEV.stop();
	FS_DEV.disable();
	FS_AUX.stop();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::motor_start> & evt)
{
	switch (evt.id)  {
	case motorId_t::LZ:
		if (FS_DEV.m_motorLZ) {
			FS_DEV.m_motorLZ->enable();
			if ((!FS_DEV.m_peLZ.get_state()
			|| evt.m_forward_dir != motor::go_backward)
			&& FS_DEV.m_motorLZ->is_stopped()) {
				FS_DEV.m_motorLZ->start_by_var_speed(evt.m_forward_dir);
			}
		}
		log_debug("rcved motor lz %s",
				 (evt.m_forward_dir == motor::go_backward ? "back" : "forward"));
		break;
	case motorId_t::RZ:
		if (FS_DEV.m_motorRZ) {
			FS_DEV.m_motorRZ->enable();
			if ((!FS_DEV.m_peRZ.get_state()
			|| evt.m_forward_dir != motor::go_backward)
			&& FS_DEV.m_motorRZ->is_stopped()) {
				FS_DEV.m_motorRZ->start_by_var_speed(evt.m_forward_dir);
			}
		}
		log_debug("rcved motor rz %s",
				 (evt.m_forward_dir == motor::go_backward ? "back" : "forward"));
		break;
	case motorId_t::X:
		if (FS_DEV.m_motorX) {
			FS_DEV.m_motorX->start_by_var_speed(evt.m_forward_dir);
		}
		log_debug("rcved motor x %s",
				 (evt.m_forward_dir == motor::go_backward ? "back" : "forward"));
		break;
	case motorId_t::Y:
		if (FS_DEV.m_motorY) {
			FS_DEV.m_motorY->start_by_var_speed(evt.m_forward_dir);
		}
		log_debug("rcved motor y %s",
				 (evt.m_forward_dir == motor::go_backward ? "back" : "forward"));
		break;
	default:
		break;
	}
	return discard_event();
}

sc::result stRunning::react(const evMotorBackEnd<motorId_t::LZ> &)
{
	/// \note maybe we can check if motor is running and backwarding first,
	/// or need not stop it.
	/// \note the driver should use oneshot mode for interrupt
	FS_DEV.m_motorLZ->stop();
	return discard_event();
}

sc::result stRunning::react(const evMotorBackEnd<motorId_t::RZ> &)
{
	FS_DEV.m_motorRZ->stop();
	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::motor_stop> & evt)
{
	switch (evt.id) {
	case motorId_t::LZ:
		if (FS_DEV.m_motorLZ) {
			FS_DEV.m_motorLZ->stop();
			FS_DEV.m_motorLZ->disable();
		}
		break;
	case motorId_t::RZ:
		if (FS_DEV.m_motorRZ) {
			FS_DEV.m_motorRZ->stop();
			FS_DEV.m_motorRZ->disable();
		}
		break;
	case motorId_t::X:
		if (FS_DEV.m_motorX) {
			FS_DEV.m_motorX->stop();
		}
		break;
	case motorId_t::Y:
		if (FS_DEV.m_motorY) {
			FS_DEV.m_motorY->stop();
		}
		break;
	default:
		break;
	}
	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::image_move> & evt)
{
	if (evt.is_pos_x)
	{
		FS_DEV.m_camera.move_window_pos_x(evt.column, evt.row);
	}
	else
	{
		FS_DEV.m_camera.move_window_pos_y(evt.column, evt.row);
	}
	return discard_event();

}

sc::result stRunning::react(const evMsg<zmsg::mid_t::discharge> & evt)
{
	/// \note stop it first
	FS_DEV.m_hvb.stop();
	FS_AUX.m_discharge_tmr.stop();

	FS_DEV.m_hvb.set_magnitude(evt.magnitude);
	/// \note limit arc time to 10s
	FS_AUX.m_discharge_tmr.start({
		{0, 0},
		exemodel::ms_to_timespec(10000)});
	FS_DEV.m_hvb.start();
	return discard_event();

}

sc::result stRunning::react(const evMsg<zmsg::mid_t::stop_discharge> &)
{
	FS_DEV.m_hvb.stop();
	FS_AUX.m_discharge_tmr.stop();
	return discard_event();
}

sc::result stRunning::react(const evDischargeTimeout & )
{
	FS_DEV.m_hvb.stop();
		
	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::set_led> & evt)
{
	switch (evt.id) {
	case ledId_t::CMOS_X:
		FS_DEV.m_x_exposure->write(static_cast<long>(evt.brightness * FS_HWINFO.x_cmos_exposure_max));
		break;
	case ledId_t::CMOS_Y:
		FS_DEV.m_y_exposure->write(static_cast<long>(evt.brightness * FS_HWINFO.y_cmos_exposure_max));
		break;
	default:
		break;
	}

	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::query_wave_form> &)
{
	FS_SM.active_waiter<evWaveFormInfo>(cmosId_t::X);
	FS_SM.active_waiter<evWaveFormInfo>(cmosId_t::Y);
	return discard_event();
}

sc::result stRunning::react(const evWaveFormInfo & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			{
				m_wave_img.xz_img = evt.img.xz_img;
				m_wave_x_ready = true;
			}
			break;
		case cmosId_t::Y:
			{
				m_wave_img.yz_img = evt.img.yz_img;
				m_wave_y_ready = true;
			}
			break;
		default:
			break;
	}

	if (m_wave_x_ready && m_wave_y_ready) {
		static const char * const xwave_path = "/tmp/x_wave.svg";
		static const char * const ywave_path = "/tmp/y_wave.svg";
		std::vector<uint8_t> x_wave;
		std::vector<uint8_t> y_wave;
		img_process::to_wave_form(m_wave_img.xz_img, x_wave, img_process::img_dir::x_img);
		img_process::to_wave_form(m_wave_img.yz_img, y_wave, img_process::img_dir::y_img);
		img_process::save_wave(xwave_path, x_wave, m_wave_img.xz_img.width, img_process::img_dir::x_img);
		img_process::save_wave(ywave_path, y_wave, m_wave_img.yz_img.width, img_process::img_dir::y_img);

		DCL_ZMSG(report_wave_form) ack_msg;
		ack_msg.x_wave = xwave_path;
		ack_msg.y_wave = ywave_path;
		FS_REPORT(ack_msg);

		//\for next time
		m_wave_y_ready = false;
		m_wave_x_ready = false;
	}

	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::manual_focal_distance> & evt)
{
	double focal_distance = evt.focal_distance;
	log_debug("manual focal: %f", focal_distance);

	if (focal_distance < 0.0) {
		focal_distance = 0.0;
	}

	if (focal_distance > 1.0) {
		focal_distance = 1.0;
	}

	if (evt.is_pos_x)
	{
		if (FS_DEV.m_x_focus) {
			FS_DEV.m_x_focus->write(focal_distance);
		}
	}
	else
	{
		if (FS_DEV.m_y_focus) {
			FS_DEV.m_y_focus->write(focal_distance);
		}
	}

	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::get_fiber_defect_info> &)
{
	FS_SM.active_waiter<evFiberDefectInfo>(cmosId_t::X);
	FS_SM.active_waiter<evFiberDefectInfo>(cmosId_t::Y);
	return discard_event();
}

sc::result stRunning::react(const evFiberDefectInfo & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			{
				m_dt_img.xz_img = evt.img.xz_img;
				m_dt_x_ready = true;
			}
			break;
		case cmosId_t::Y:
			{
				m_dt_img.yz_img = evt.img.yz_img;
				m_dt_y_ready = true;
			}
			break;
		default:
			break;
	}

	if (m_dt_x_ready && m_dt_y_ready) {
		DCL_ZMSG(defect_detect_result) msg;
		img_process::detect_defect(
			m_dt_img, FS_IPP,
			{
				std::abs(FS_SPEC.dbg_vangle_limit),
				std::abs(FS_SPEC.dbg_hangle_limit),
				false,
			},
			msg.data);
		FS_REPORT(msg);

		//\for next time
		m_dt_x_ready = false;
		m_dt_y_ready = false;
	}

	return discard_event();
}

sc::result stRunning::react(const evImgProcessError &)
{
	FS_DAT.code = fs_err_t::img_process_error;

	auto & re = FS_DAT;
	FS_REPORT(re);

	return discard_event();
}

sc::result stRunning::react(const evSystemError &)
{
	FS_DAT.code = fs_err_t::system_error;

	auto & re = FS_DAT;
	FS_REPORT(re);

	return discard_event();
}

} /* namespace smRegularTest */

} /* namespace svcFS */
