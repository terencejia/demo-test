#include <fs_log.hpp>

#include "fs_spec.hpp"

#include "sm_discharge_adjust.hpp"

namespace svcFS {

namespace smDischargeAdjust {

stSuccess::stSuccess(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}

stSuccess::~stSuccess()
{
}

sc::result stSuccess::react(const __evEntryAct &)
{
	/// update temperature
	FS_DAT.revise.temp = FS_DEV.m_env_temp.read();
	FS_DAT.revise.pressure = FS_DEV.m_env_pressure.read();

	/// update spec
	if (FS_SPEC.discharge_base.empty()) {
		FS_SPEC.discharge_base = FS_DAT.revise;
	}
	FS_SPEC.discharge_revise = FS_DAT.revise;

	/// report
	FS_DAT.code = fs_err_t::success;
	FS_DAT.base = FS_SPEC.discharge_base;

	log_debug("da: success... %f: %f,  %f, %f",
		FS_DAT.revise.p[0].x, FS_DAT.revise.p[0].y,
		FS_DAT.revise.p[1].x, FS_DAT.revise.p[1].y);

	return transit<stWaitReset>();
}

} /* namespace smDischargeAdjust */

} /* namespace svcFS */
