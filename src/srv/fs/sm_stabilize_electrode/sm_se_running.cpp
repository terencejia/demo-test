#include <zmsg/zmsg_discharge.hpp>

#include "sm_stabilize_electrode.hpp"

namespace svcFS {

namespace smStabilizeElectrode {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.se.cfg)
, m_data(context<fs_sm>().m_ctx.se.data)
{
	FS_DEV.enable();
}

stRunning::~stRunning()
{
	DCL_ZMSG(discharge_count) msg;
	msg.discharge_count = FS_DEV.m_hvb.get_count();
	if (msg.discharge_count > 0) {
		FS_REPORT(msg);
	}

	FS_DEV.stop();
	FS_DEV.disable();
	FS_AUX.stop();
}

sc::result stRunning::react(const evCoverOpen &)
{
	DCL_ZMSG(stabilize_electrode_result) & msg = FS_DAT;
	msg.code = fs_err_t::cover_openned;
	FS_REPORT(msg);

	return transit<stReset>();
}

} /* smStabilizeElectrode */

} /* svcFS */
