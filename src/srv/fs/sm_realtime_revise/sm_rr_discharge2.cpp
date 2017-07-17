#include <fs_log.hpp>

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise {

stDischarge2::stDischarge2(my_context ctx)
: my_base(ctx)
{
	if (FS_CFG.Discharge2LastTime > 0) {
		DCL_ZMSG(fs_state) msg;
		msg.sstate = svc_fs_state_t::fs_discharge2;
		FS_REPORT(msg);
		log_debug("%s...", FS_STATE_NAME);
	}

	post_event(__evEntryAct());
}

stDischarge2::~stDischarge2()
{
}

sc::result stDischarge2::react(const __evEntryAct &)
{
	if (FS_CFG.Discharge2LastTime > 0) {
		log_debug("fs: discharge2...");
		FS_AUX.m_discharge_tmr.start({
			{0, 0},
			exemodel::ms_to_timespec(FS_CFG.Discharge2LastTime * 1000)});
		FS_DEV.m_hvb.set_magnitude(FS_CFG.Discharge2Strength);
		FS_AUX.m_hvb_pwm.start(
			exemodel::ms_to_timespec(FS_CFG.Discharge2StartTime),
			exemodel::ms_to_timespec(FS_CFG.Discharge2StopTime));
		return discard_event();
	}
	else {
		return transit<stRR>();
	}
}

sc::result stDischarge2::react(const evDischargeTimeout &)
{
	FS_AUX.m_hvb_pwm.stop();
	return transit<stRR>();
}

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
