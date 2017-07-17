#include <zmsg/zmsg_cover.hpp>

#include "fs_sm.hpp"

namespace svcFS {
stWaitReset::stWaitReset(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_wait_reset;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	const bool cover_openned = FS_DEV.m_cover.get_state();
	if (cover_openned) {
		DCL_ZMSG(fs_cover_openned) msg;
		FS_REPORT(msg);
	}
}

stWaitReset::~stWaitReset()
{
}

sc::result stWaitReset::react(const evMsg<zmsg::mid_t::fusion_splice_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}
sc::result stWaitReset::react(const evMsg<zmsg::mid_t::discharge_adjust_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}
sc::result stWaitReset::react(const evMsg<zmsg::mid_t::motor_test_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}
sc::result stWaitReset::react(const evMsg<zmsg::mid_t::dust_check_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}
sc::result stWaitReset::react(const evMsg<zmsg::mid_t::stabilize_electrode_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}
sc::result stWaitReset::react(const evMsg<zmsg::mid_t::regular_test_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}
sc::result stWaitReset::react(const evMsg<zmsg::mid_t::realtime_revise_start> & evt)
{
	post_event(evt);
	return transit<stReset>();
}

/// for app countdown reset operation
sc::result stWaitReset::react(const evCoverOpen &)
{
	DCL_ZMSG(fs_cover_openned) msg;
	FS_REPORT(msg);

	return discard_event();
}

} /* svcFS */

