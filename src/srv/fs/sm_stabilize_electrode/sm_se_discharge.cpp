#include <fs_log.hpp>

#include "sm_stabilize_electrode.hpp"

namespace svcFS {

namespace smStabilizeElectrode {

stDischarge::stDischarge(my_context ctx)
: my_base(ctx)
{
	log_debug("se: discharge...");
	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.time)});
	FS_DEV.m_hvb.start(FS_CFG.magnitude);
	++FS_DAT.number;
}

stDischarge::~stDischarge()
{
	FS_DEV.m_hvb.stop();
}

sc::result stDischarge::react(const evDischargeTimeout &)
{
	if (FS_DAT.number < FS_CFG.number) {
		DCL_ZMSG(stabilize_electrode_result) & msg = FS_DAT;
		msg.code = fs_err_t::ignore;
		FS_REPORT(msg);

		return transit<stInterval>();
	}
	else {
		DCL_ZMSG(stabilize_electrode_result) & msg = FS_DAT;
		msg.code = fs_err_t::success;
		FS_REPORT(msg);

		return transit<stIdle>();
	}
}

}

}
