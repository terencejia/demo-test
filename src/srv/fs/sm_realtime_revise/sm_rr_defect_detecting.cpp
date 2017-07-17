#include <fs_log.hpp>

#include <zmsg/zmsg_defect_detect.hpp>

#include "img_process.hpp"
#include "fs_spec.hpp"

#include "sm_realtime_revise.hpp"

#include <cstdlib>

namespace svcFS {

namespace smRealtimeRevise {

stDefectDetecting::stDefectDetecting(my_context ctx)
: my_base(ctx)
, m_x_ready(false)
, m_y_ready(false)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_defect_detecting;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	FS_SM.active_delay_waiter<evHvbReady>(cmosId_t::X);
	FS_SM.active_delay_waiter<evHvbReady>(cmosId_t::Y);
}

stDefectDetecting::~stDefectDetecting()
{
}

sc::result stDefectDetecting::react(const evHvbReady & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			{
				m_img.xz_img = evt.img.xz_img;
				m_x_ready = true;
			}
			break;
		case cmosId_t::Y:
			{
				m_img.yz_img = evt.img.yz_img;
				m_y_ready = true;
			}
			break;
		default:
			break;
	}
	if ( !(m_x_ready && m_y_ready) ) {
		return discard_event();
	}

	try {
		img_process::detect_defect(
			m_img, FS_IPP,
			{
				FS_CFG.CutAngleLimit,
				FS_CFG.FiberAngleLimit,
				true,
			},
			FS_DAT.defect_data);
		/// \todo delete me, only for test
		img_process::print_defects(FS_DAT.defect_data);

		DCL_ZMSG(defect_detect_result) dd_msg = { FS_DAT.defect_data, };
		FS_REPORT(dd_msg);

		/// \note omit some defects
		ifd_t ifd_msk = ifd_all;
		if (FS_CFG.FiberCoreAngle) {
			ifd_clr(ifd_msk, ifd_horizontal_angle);
		}
		if (FS_CFG.Cut) {
			ifd_clr(ifd_msk, ifd_vertical_angle);
		}

		if (FS_DAT.defect_data.check(ifd_msk)) {
			log_warning("%s: the fiber has defect!", FS_STATE_NAME);
			FS_DAT.code = fs_err_t::fiber_defect;
			return discard_event();
		}
		else {
			/// update 'nm_per_pixel'
			FS_SPEC.update_yz_nm_per_pixel(
				FS_DAT.rec_info.wrap_diameter / FS_DAT.defect_data.yzl.wrap_diameter,
				FS_DAT.rec_info.wrap_diameter / FS_DAT.defect_data.yzr.wrap_diameter);
			FS_SPEC.update_xz_nm_per_pixel(
				FS_DAT.rec_info.wrap_diameter / FS_DAT.defect_data.xzl.wrap_diameter,
				FS_DAT.rec_info.wrap_diameter / FS_DAT.defect_data.xzr.wrap_diameter);

			/**
			 *  adjust cmos window  horizontal position
			 *  Note: adjusted result data will not be reported app
			 **/
			int xr_off = img_process::right_hcenter(m_img.xz_img, FS_IPP) - m_img.xz_img.height / 2;
			int yr_off = img_process::left_hcenter(m_img.yz_img, FS_IPP) - m_img.yz_img.height / 2;

			log_debug("xr_off: %d, yr_off: %d", xr_off, yr_off);

			if(std::abs(xr_off) <= 20 && std::abs(yr_off) <= 20) {
				if(std::abs(xr_off) > 1) {
					FS_DEV.m_camera.move_window_pos_x(0, xr_off);
				}
				if(std::abs(yr_off) > 1) {
					FS_DEV.m_camera.move_window_pos_y(0, yr_off);
				}
			}
			else {
				FS_DAT.code = fs_err_t::fiber_off_center;
				log_debug("fiber offes vertical center position too much");
				return transit<stWaitReset>();
			}

			///adjust cmos x window coordinate  by vertex pos difference between  x and y img,  x img  follow y img
			{
				const int x_lf_vertex_pos = img_process::left_vertex_pos(m_img.xz_img, {0}, true);
				const int y_lf_vertex_pos = img_process::left_vertex_pos(m_img.yz_img, {0}, true);
				const int vertex_diff = x_lf_vertex_pos - y_lf_vertex_pos;

				log_debug("vertex_diff ( %d %d %d)", x_lf_vertex_pos, y_lf_vertex_pos, vertex_diff);

				/// adjust x window
				if (vertex_diff != 0) {
					FS_DEV.m_camera.move_window_pos_x(vertex_diff, 0);
					FS_SPEC.window_x_col += vertex_diff;

					DCL_ZMSG(update_window_position) msg;
					msg.is_pos_x = true;
					msg.row = FS_SPEC.window_x_row;
					msg.column = FS_SPEC.window_x_col;
					FS_REPORT(msg);
				}

				if (std::abs(vertex_diff) > 20) {
					log_warning("difference between x/y fiber vertex position too much!");
					//\TODO report error code and transit stWaitReset if needed
				}
			}

			return transit<stPush2>();
		}
	}
	catch (const img_process::img_process_error & e) {
		log_warning("img process exception: %s", e.what());

		FS_DAT.code = fs_err_t::fiber_defect;
		return transit<stReset>();
	}
}

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
