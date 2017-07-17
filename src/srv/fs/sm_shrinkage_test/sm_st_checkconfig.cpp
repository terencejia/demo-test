#include <fs_log.hpp>

#include "fs_spec.hpp"

#include "sm_shrinkage_test.hpp"

namespace svcFS{

namespace smShrinkageTest{

stCheckConfig::stCheckConfig(my_context ctx)
:my_base(ctx)
{
	post_event(__evEntryAct());
}

stCheckConfig::~stCheckConfig()
{

}

sc::result stCheckConfig::react(const __evEntryAct &)
{
	if(FS_CFG.FiberPreFSTime == 0 || FS_CFG.Discharge1Time == 0) {
		log_warning("dischge time can't be 0!!!");
		FS_DAT.code = fs_err_t::arc_time_zero;

		return transit<stIdle>(); ///don't need reset
	}
	else {
		return transit<stAdjustBrightness>();
	}

}

}

}
