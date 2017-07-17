
#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {
/**
 * stFS_stage1
 */
stFS_stage1::stFS_stage1(my_context ctx)
: my_base(ctx)
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));

	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_push1;
	FS_REPORT(msg);

	log_debug("%s...", FS_STATE_NAME);
}

stFS_stage1::~stFS_stage1()
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
}

sc::result stFS_stage1::react(const evAdjustBrightnessDone &)
{
	return __check_all_state_done();
}

sc::result stFS_stage1::react(const evPush1Done &)
{
	return __check_all_state_done();
}

sc::result stFS_stage1::react(const evCalibratingDone &)
{
	return __check_all_state_done();
}

sc::result stFS_stage1::react(const evAdjustFocusDone &)
{
	return __check_all_state_done();
}

sc::result stFS_stage1::__check_all_state_done(void)
{
	if(state_downcast< const stAdjustBrightnessDone *>() != 0
	  && state_downcast< const stPush1Done *>() != 0
	  && state_downcast< const stCalibratingDone *>() != 0
	  && state_downcast< const stAdjustFocusDone *>() != 0) {
		 return transit<stWaiting>();
	}
	else {
		return discard_event();
	}
}
  
} /* smFastFusionSplicing */

} /* namespace svcFS */
