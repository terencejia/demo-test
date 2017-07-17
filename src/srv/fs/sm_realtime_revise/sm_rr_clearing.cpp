#include <fs_log.hpp>

#include "img_process.hpp"

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise {

stClearing::stClearing(my_context ctx)
: my_base(ctx)
, m_has_arc(false)
, m_clr_times(0)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_clring;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stClearing::~stClearing()
{
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

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
