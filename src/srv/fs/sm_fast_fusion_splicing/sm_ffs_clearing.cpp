#include <fs_log.hpp>

#include "img_process.hpp"

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stClearing::stClearing(my_context ctx)
: my_base(ctx)
, m_has_arc(false)
, m_clr_times(0)
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));

	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_clring;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stClearing::~stClearing()
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stClearing::react(const __evEntryAct &)
{
	if(FS_CFG.CleanDischargeTime == 0) {
		log_warning("clean arc time is 0!!!");
		FS_DAT.code = fs_err_t::arc_time_zero; // arc time is 0
		return transit< stDefectDetecting >();
	}
	else {
		FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.CleanDischargeTime)});
		FS_DEV.m_hvb.start(FS_CFG.CleanDischargeStrength);
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}
}

sc::result stClearing::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();

	if (!m_has_arc) {
		log_err("%s abnormal arc", FS_STATE_NAME);
		FS_DAT.code = fs_err_t::abnormal_arc;

		return transit<stReset> ();
	}
#if 0
	/// fiber identify
	{
		for (std::size_t i = 0; i < s_y.size(); ++i) {
			const gray_img & tmp = s_y[i];
			img_process::save_pgm("yzidentify_" + std::to_string(i) + ".pgm", tmp);
			img_process::identify_left_fiber_model(tmp, FS_DAT.y_left_region);
			img_process::identify_right_fiber_model(tmp, FS_DAT.y_right_region);
		}
		for (std::size_t i = 0; i < s_x.size(); ++i) {
			const gray_img & tmp = s_x[i];
			img_process::save_pgm("xzidentify_" + std::to_string(i) + ".pgm", tmp);
			img_process::identify_left_fiber_model(tmp, FS_DAT.x_left_region);
			img_process::identify_right_fiber_model(tmp, FS_DAT.x_right_region);
		}
	}
#endif
	if (FS_CFG.CleanDischargeTwice && ++m_clr_times<2) {
		usleep(400000);
		post_event(__evEntryAct());
		return discard_event();
	}

	return transit<stDefectDetecting>();
}

sc::result stClearing::react(const __evImgReady & evt)
{
	m_has_arc = img_process::has_electric_arc(evt.img.yz_img, FS_IPP);
	if (!m_has_arc) {
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
	}

	return discard_event();
}

}

}
