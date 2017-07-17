#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {


stCalibrating::stCalibrating(my_context ctx)
: my_base(ctx)
, m_x_dist(std::numeric_limits<int32_t>::max())
, m_y_dist(std::numeric_limits<int32_t>::max())
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_calibrating;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stCalibrating::~stCalibrating()
{
	FS_AUX.m_mt_tmr.stop();

	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stCalibrating::react(const __evEntryAct &)
{
	FS_AUX.m_mt_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
			15000) });
	FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
	FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
	return discard_event();
}

sc::result stCalibrating::react(const __evImgReady & evt)
{
	switch (evt.id) {
		case cmosId_t::X:
			{
				const double xz_delta = img_process::xy_dist(evt.img.xz_img, FS_IPP);
				log_debug("x dist: %3.3f", xz_delta);
				int32_t const x_dist = (FS_SPEC.is_xy_dist_ok_for_calibrating(xz_delta)
					? 0 : FS_SPEC.pixel_to_xy_step(xz_delta));

				if (FS_DEV.m_motorX->is_stopped()) {
				if (x_dist != 0) {
					FS_DEV.m_motorX->start_by_speed(
						FS_SPEC.xy_speed_from_pixel(xz_delta),
						x_dist > 0 ? motor::go_forward : motor::go_backward);
					}
				}
				else {
					if ((x_dist == 0)		/// already in postion
					|| (x_dist > 0 && m_x_dist < 0)
					|| (x_dist < 0 && m_x_dist > 0)) {		/// different direction
						FS_DEV.m_motorX->stop();
					}
					else {
						FS_DEV.m_motorX->set_speed(FS_SPEC.xy_speed_from_pixel(xz_delta));
					}
				}
				m_x_dist = x_dist;

				if(m_x_dist != 0)
					FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
			}
			break;
		case cmosId_t::Y:
			{
				const double yz_delta = img_process::xy_dist(evt.img.yz_img, FS_IPP);
				log_debug("y dist: %3.3f", yz_delta);
				int32_t const y_dist = (FS_SPEC.is_xy_dist_ok_for_calibrating(yz_delta)
					? 0 : FS_SPEC.pixel_to_xy_step(yz_delta));

				if (FS_DEV.m_motorY->is_stopped()) {
					if (y_dist != 0) {
						FS_DEV.m_motorY->start_by_speed(
							FS_SPEC.xy_speed_from_pixel(yz_delta),
							y_dist > 0 ? motor::go_forward : motor::go_backward);
					}
				}
				else {
					if ((y_dist == 0)
					|| (y_dist > 0 && m_y_dist < 0)
					|| (y_dist < 0 && m_y_dist > 0)) {
						FS_DEV.m_motorY->stop();
					}
					else {
						FS_DEV.m_motorY->set_speed(FS_SPEC.xy_speed_from_pixel(yz_delta));
					}
				}
				m_y_dist = y_dist;

				if(m_y_dist != 0)
					FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
			}
			break;
		default :
			break;
	}
	if (m_x_dist == 0 && m_y_dist == 0) {
		return transit<stClearing>();
	}

	return discard_event();
}

sc::result stCalibrating::react(const evMotorTestTimeout &)
{
	log_debug("%s...Calibrating timeout", FS_STATE_NAME);
	++FS_DAT.calibrate;
	FS_DAT.code = fs_err_t::calibrate_timeout;

	///\todo judge whether the machine need to be rebooted
	return transit<stWaitReset>();
}

} /* namespace smMotorTest */

}
