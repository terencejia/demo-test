#include <fs_log.hpp>

#include "fs_sm.hpp"
#include "fs_ctx.hpp"

namespace svcFS {

stReset::stReset(my_context ctx)
: my_base(ctx)
, m_deferral_processor(nullptr)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_reseting;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	FS_DEV.enable();
}

stReset::~stReset()
{
	FS_DEV.stop();
	FS_DEV.disable();

	if (m_deferral_processor) {
		m_deferral_processor();
	}
}

sc::result stReset::react(const evMsg<zmsg::mid_t::fusion_splice_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}
sc::result stReset::react(const evMsg<zmsg::mid_t::discharge_adjust_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}
sc::result stReset::react(const evMsg<zmsg::mid_t::motor_test_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}
sc::result stReset::react(const evMsg<zmsg::mid_t::dust_check_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}

sc::result stReset::react(const evMsg<zmsg::mid_t::full_dust_check_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}

sc::result stReset::react(const evMsg<zmsg::mid_t::stabilize_electrode_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}
sc::result stReset::react(const evMsg<zmsg::mid_t::regular_test_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}
sc::result stReset::react(const evMsg<zmsg::mid_t::realtime_revise_start> & evt)
{
	this->set_deferral(evt);
	return discard_event();
}

sc::result stReset::react(const evMsg<zmsg::mid_t::stop> &)
{
	this->clear_deferral();
	return discard_event();
}
sc::result stReset::react(const evMsg<zmsg::mid_t::fusion_splice_reset> &)
{
	this->clear_deferral();
	return discard_event();
}

sc::result stReset::react(const evCoverClose &)
{
	zmsg::zmsg<zmsg::mid_t::fs_state> msg = { svc_fs_state_t::fs_ready, };
	log_debug("stReset report fs_ready");
	FS_REPORT(msg);

	return discard_event();
}

} /* svcFS */

