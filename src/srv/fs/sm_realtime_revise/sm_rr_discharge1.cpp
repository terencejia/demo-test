#include <fs_log.hpp>

#include "fs_spec.hpp"

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise{

stDischarge1::stDischarge1(my_context ctx)
: my_base(ctx)
, m_stair_flag(true)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_discharge1;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stDischarge1::~stDischarge1()
{
	FS_AUX.m_stair_tmr.stop();
}

static constexpr double stair_last_time = 1000;	/// ms
static constexpr uint32_t stair_interval = 10;	/// ms

sc::result stDischarge1::react(const __evEntryAct &)
{
	FS_DEV.m_motorLZ->start_ss_for_fs(
		FS_CFG.LeftFSSpeed,
		context<stNormalDischarge>().m_lz_step,
		motor::go_forward);
	FS_DEV.m_motorRZ->start_ss_for_fs(
		FS_CFG.RightFSSpeed,
		context<stNormalDischarge>().m_rz_step,
		motor::go_forward);

	if (FS_CFG.Discharge1Time > 0) {
		FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.Discharge1Time)});
		if(m_stair_flag && (FS_CFG.Discharge1Time > stair_last_time)) {
			double init = FS_CFG.FiberPreFSStrength;
			double target = FS_CFG.Discharge1Strength;
			exemodel::timespec_t begin_time = exemodel::monotonic_timeree::info::get_time();
			FS_AUX.m_stair_tmr.connect([this,init,target,begin_time](exemodel::poller&, uint64_t) {
				///\brief calculate the strength by the timer
				double arc_diff = target - init;
				auto cur = exemodel::monotonic_timeree::info::get_time();
				auto diff = exemodel::timespec_sub(cur, begin_time);
				double time = diff.tv_sec * 1000L + diff.tv_nsec / 1000000L;
				double strength;
				if(time < stair_last_time) {
					strength = (time / stair_last_time) * arc_diff + init;
				}
				else {
					strength = target;
					FS_AUX.m_stair_tmr.stop();
				}

				FS_DEV.m_hvb.set_magnitude(strength);
			});
			FS_DEV.m_hvb.set_magnitude(FS_CFG.FiberPreFSStrength);
			FS_AUX.m_stair_tmr.start({exemodel::ms_to_timespec(stair_interval), exemodel::ms_to_timespec(stair_interval)});
		}
		else {
			FS_DEV.m_hvb.set_magnitude(FS_CFG.Discharge1Strength);
		}

		return discard_event();
	}
	else {
		return transit<stDischarge2>();
	}
}

sc::result stDischarge1::react(const evDischargeTimeout &)
{
	FS_DEV.m_motorLZ->stop();
	FS_DEV.m_motorRZ->stop();
	FS_DEV.m_hvb.stop();
	return transit<stDischarge2>();
}

} /* namespace smRealtimeRevise */

} /* svcFS */
