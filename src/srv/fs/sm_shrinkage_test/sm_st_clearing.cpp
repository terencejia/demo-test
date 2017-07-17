#include <fs_log.hpp>

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {

stClearing::stClearing(my_context ctx)
: my_base(ctx)
{
	log_debug("fs: clearing...");
	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.CleanDischargeTime)});
	/// \todo modify the clear strength
	FS_DEV.m_hvb.start(FS_SPEC.clr_discharge_strength);
}

stClearing::~stClearing()
{
}

sc::result stClearing::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();
	return transit< stDefectDetecting >();
}

}

}
