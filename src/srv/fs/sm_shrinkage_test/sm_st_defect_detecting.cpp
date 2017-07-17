#include <fs_log.hpp>

#include <zmsg/zmsg_defect_detect.hpp>

#include "img_process.hpp"
#include "fs_spec.hpp"

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {

stDefectDetecting::stDefectDetecting(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_defect_detecting;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	FS_SM.active_delay_waiter<evHvbReady>();
}

stDefectDetecting::~stDefectDetecting()
{
}

sc::result stDefectDetecting::react(const evHvbReady & evt)
{
	try {
		img_process::detect_defect(
			evt.img, FS_IPP,
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
			int xr_off = img_process::right_hcenter(evt.img.xz_img, FS_IPP) - evt.img.xz_img.height / 2;
			int yr_off = img_process::left_hcenter(evt.img.yz_img, FS_IPP) - evt.img.yz_img.height / 2;

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
				return transit<stReset>();
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

}

}
