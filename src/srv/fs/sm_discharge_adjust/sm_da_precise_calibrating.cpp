#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_discharge_adjust.hpp"

namespace svcFS {

namespace smDischargeAdjust {

stPreciseCalibrating::stPreciseCalibrating(my_context ctx)
: my_base(ctx)
, m_x_dist(0)
, m_y_dist(0)
, m_x_done(false)
, m_y_done(false)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_precise_calibrating;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stPreciseCalibrating::~stPreciseCalibrating()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stPreciseCalibrating::react(const __evEntryAct &)
{
	FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
	FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
	return discard_event();
}

sc::result stPreciseCalibrating::react(const __evImgReady & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			{		/// \note indeed, we have updated x/y spliter in defect detecting already.
				if (FS_SPEC.x_spliter == 0) {
					FS_SPEC.x_spliter = img_process::get_spliter(evt.img.xz_img, FS_IPP);
				}
				const int xz_base = static_cast<int>(evt.img.xz_img.width / 2
					+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
				const int xz_off = static_cast<int>(FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
					+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
				const double xz_delta = img_process::xy_precise_dist_core(evt.img.xz_img, FS_IPP, xz_base, xz_off, FS_SPEC.x_spliter);
				log_debug("x precise dist: %3.3f", xz_delta);
				/*
				* \note don't use FS_SPEC.is_xy_dist_ok_for_precise_calibrating,
				* for fast calibrating in discharge adjust
				*/
				m_x_done = (std::abs(xz_delta) < 0.5);

				/// further check
				if (!m_x_done)
					FS_SM.active_waiter<__evImgReady>(cmosId_t::X);

				if (FS_DEV.m_motorX->is_stopped()) {
					if (!m_x_done) {
						FS_DEV.m_motorX->start_by_speed(
							FS_SPEC.xy_speed_from_pixel(xz_delta),
							xz_delta > 0 ? motor::go_forward : motor::go_backward);
					}
				}
				else {
					if (m_x_done		/// already in postion
						|| (xz_delta > 0 && m_x_dist < 0)
						|| (xz_delta < 0 && m_x_dist > 0)) {		/// different direction
						FS_DEV.m_motorX->stop();
					}
					else {
						FS_DEV.m_motorX->set_speed(FS_SPEC.xy_speed_from_pixel(xz_delta));
					}
				}
				m_x_dist = xz_delta;
				break;
			}
		case cmosId_t::Y:
			{
				if (FS_SPEC.y_spliter == 0) {
					FS_SPEC.y_spliter = img_process::get_spliter(evt.img.yz_img, FS_IPP);
				}
				const int yz_base = static_cast<int>(evt.img.yz_img.width / 2
					+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
				const int yz_off = static_cast<int>(FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
					+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
				const double yz_delta = img_process::xy_precise_dist_core(evt.img.yz_img, FS_IPP, yz_base, yz_off, FS_SPEC.y_spliter);
				log_debug("y precise dist: %3.3f", yz_delta);
				m_y_done = (std::abs(yz_delta) < 0.5);

				/// further check
				if (!m_y_done)
					FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
				
				if (FS_DEV.m_motorY->is_stopped()) {
					if (!m_y_done) {
						FS_DEV.m_motorY->start_by_speed(
							FS_SPEC.xy_speed_from_pixel(yz_delta),
							yz_delta > 0 ? motor::go_forward : motor::go_backward);
					}
				}
				else {
					if (m_y_done
						|| (yz_delta > 0 && m_y_dist < 0)
						|| (yz_delta < 0 && m_y_dist > 0)) {
						FS_DEV.m_motorY->stop();
					}
					else {
						FS_DEV.m_motorY->set_speed(FS_SPEC.xy_speed_from_pixel(yz_delta));
					}
				}
				m_y_dist = yz_delta;
				break;
			}
		default:
			break;
	}

	if (m_x_done && m_y_done) {
		return transit<stDischarge1>();
	}

	return discard_event();
}

}

}
