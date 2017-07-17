
#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_discharge_adjust.hpp"

namespace svcFS {

namespace smDischargeAdjust {
/**
 * stFS_stage1
 */
stFS_stage1::stFS_stage1(my_context ctx)
: my_base(ctx)
{
	log_debug("%s...", FS_STATE_NAME);
}

stFS_stage1::~stFS_stage1()
{
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

sc::result stFS_stage1::__check_all_state_done(void)
{
	if(state_downcast< const stAdjustBrightnessDone *>() != 0
	  && state_downcast< const stPush1Done *>() != 0
	  && state_downcast< const stCalibratingDone *>() != 0) {
		 return transit<stClearing>();
	}
	else {
		return discard_event();
	}
}
  
} /* smDischargeAdjust */

} /* namespace svcFS */
