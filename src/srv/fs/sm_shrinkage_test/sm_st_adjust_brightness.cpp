#include <fs_log.hpp>

#include "img_process.hpp"

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {

static bool __handle_img(const gray_img & gi,
			const double ref_br,
			const double diff_br,
			double & min_brightness,
			double & max_brightness,
			led & dev)
{
	double cur_br = img_process::img_gray_scale(gi);
	if (std::fabs(cur_br - ref_br) < diff_br) {
		return true;
	}

	if (cur_br < ref_br) {
		min_brightness = dev.get_brightness();
	}
	else {
		max_brightness = dev.get_brightness();
	}

	if (max_brightness <= min_brightness
		|| (max_brightness - min_brightness) <= 0.01) {
		throw std::runtime_error("adjust brightness through led!");
	}

	dev.set_brightness((min_brightness + max_brightness) / 2);

	return false;
}

stAdjustBrightness::stAdjustBrightness(my_context ctx)
: my_base(ctx)
, m_ref_br(FS_SPEC.background_brightness())
, m_diff_br(0.02)
, m_ori_x(FS_DEV.m_ledX.get_brightness())
, m_ori_y(FS_DEV.m_ledY.get_brightness())
, m_min_x(0.0)
, m_max_x(1.0)
, m_min_y(0.0)
, m_max_y(1.0)
{
	post_event(__evEntryAct());
}

stAdjustBrightness::~stAdjustBrightness()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stAdjustBrightness::react(const __evEntryAct &)
{
	FS_SM.active_delay_waiter<__evImgReady>();
	return discard_event();
}

sc::result stAdjustBrightness::react(const __evImgReady & evt)
{
	try {
		bool x_ret = __handle_img(evt.img.xz_img, m_ref_br, m_diff_br, m_min_x, m_max_x, FS_DEV.m_ledX);
		bool y_ret = __handle_img(evt.img.yz_img, m_ref_br, m_diff_br, m_min_y, m_max_y, FS_DEV.m_ledY);
		if (x_ret && y_ret) {
			if (FS_DEV.m_ledX.get_brightness() != m_ori_x) {
				DCL_ZMSG(update_led_brightness) msg;
				msg.id = ledId_t::CMOS_X;
				msg.brightness = FS_DEV.m_ledX.get_brightness();
				FS_REPORT(msg);
			}
			if (FS_DEV.m_ledY.get_brightness() != m_ori_y) {
				DCL_ZMSG(update_led_brightness) msg;
				msg.id = ledId_t::CMOS_Y;
				msg.brightness = FS_DEV.m_ledY.get_brightness();
				FS_REPORT(msg);
			}
			return transit<stPush1>();
		}
	}
	catch (...) {
		img_process::save_pgm("errorx.pgm", evt.img.xz_img);
		img_process::save_pgm("errory.pgm", evt.img.yz_img);
		log_err("%s adjust brightness error", FS_STATE_NAME);
		FS_DEV.m_ledX.set_brightness(m_ori_x);
		FS_DEV.m_ledY.set_brightness(m_ori_y);
		FS_DAT.code = fs_err_t::img_brightness;

		return transit<stReset>();
	}

	FS_SM.active_delay_waiter<__evImgReady>();
	return discard_event();
}

} /* smDischargeAdjust */

} /* namespace svcFS */
