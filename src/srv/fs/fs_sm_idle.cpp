#include <fs_log.hpp>

#include "fs_sm.hpp"
#include "fs_ctx.hpp"

#include "sm_fast_fusion_splicing/sm_fast_fusion_splice.hpp"
#include "sm_regular_test/sm_regular_test.hpp"
#include "sm_discharge_adjust/sm_discharge_adjust.hpp"
#include "sm_motor_test/sm_motor_test.hpp"
#include "sm_dust_check/sm_dust_check.hpp"
#include "sm_full_dust_check/sm_full_dust_check.hpp"
#include "sm_stabilize_electrode/sm_stabilize_electrode.hpp"
#include "sm_shrinkage_test/sm_shrinkage_test.hpp"
#include "sm_realtime_revise/sm_realtime_revise.hpp"

namespace svcFS {

stIdle::stIdle(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_idle;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);
}

stIdle::~stIdle()
{
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::fusion_splice_start> & evt)
{
	return __transit< smFastFusionSplicing::stRunning >(
		evt, context<fs_sm>().m_ctx.fs, "fast fusion splice");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::discharge_adjust_start> & evt)
{
	return __transit< smDischargeAdjust::stRunning >(
		evt, context<fs_sm>().m_ctx.da, "discharge adjust");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::regular_test_start> & evt)
{
	return __transit_force< smRegularTest::stRunning >(evt, context<fs_sm>().m_ctx.rt, "regular test");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::motor_test_start> & evt)
{
	return __transit< smMotorTest::stRunning >(evt, context<fs_sm>().m_ctx.mt, "motor test");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::dust_check_start> & evt)
{
	return __transit< smDustCheck::stRunning >(evt, context<fs_sm>().m_ctx.dc, "dust check");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::full_dust_check_start> & evt)
{
	return __transit_force< smFullDustCheck::stRunning >(evt, context<fs_sm>().m_ctx.fc, "full dust check");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::stabilize_electrode_start> & evt)
{
	return __transit< smStabilizeElectrode::stRunning >(
		evt,
		context<fs_sm>().m_ctx.se, "stabilize electrode");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::shrinkage_test_start> & evt)
{
	return __transit< smShrinkageTest::stRunning >(
		evt,
		context<fs_sm>().m_ctx.st, "shrinkage test");
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::realtime_revise_start> & evt)
{
	return __transit< smRealtimeRevise::stRunning >(
		evt, context<fs_sm>().m_ctx.rr, "realtime revise");
}


sc::result stIdle::react(const evCoverClose &)
{
	zmsg::zmsg<zmsg::mid_t::fs_state> msg = { svc_fs_state_t::fs_ready, };
	log_debug("report fs_ready");
	FS_REPORT(msg);

	return discard_event();
}

} /* namespace svcFS */
