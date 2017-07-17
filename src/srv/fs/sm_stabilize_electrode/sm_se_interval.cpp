#include <fs_log.hpp>

#include "sm_stabilize_electrode.hpp"

namespace svcFS {

namespace smStabilizeElectrode {

stInterval::stInterval(my_context ctx)
: my_base(ctx)
{
	log_debug("se: interval...");
	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.interval ? FS_CFG.interval : 3000)});
}

stInterval::~stInterval()
{
}

sc::result stInterval::react(const evDischargeTimeout &)
{
	return transit<stDischarge>();
}

}

}
