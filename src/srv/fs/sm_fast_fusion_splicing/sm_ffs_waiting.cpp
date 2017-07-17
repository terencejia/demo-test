#include <fs_log.hpp>

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stWaiting::stWaiting(my_context ctx)
: my_base(ctx)
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
	log_debug("%s...", FS_STATE_NAME);
	post_event(__evEntryAct());
}

stWaiting::~stWaiting()
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
}

sc::result stWaiting::react(const __evEntryAct &)
{
	if(!FS_CFG.AutoStart && FS_CFG.FiberAutoFeed){
		DCL_ZMSG(fs_state) msg;
		msg.sstate = svc_fs_state_t::fs_waiting;
		FS_REPORT(msg);
		return discard_event();
	}
	else{
		return transit<stClearing>();
	}
}

}/* namespace smFastFusionSplicing */

}/* namespace svc_FS */
