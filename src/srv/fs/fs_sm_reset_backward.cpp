#include <fs_log.hpp>

#include "fs_sm.hpp"
#include "fs_ctx.hpp"

namespace svcFS {

struct stResetForward;
stResetBackward::stResetBackward(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}

stResetBackward::~stResetBackward()
{
}

bool stResetBackward::__handle_lz()
{
#if 0 /// \todo DELETEME
	log_debug("handle lz pe: %d", (int)FS_DEV.m_peLZ.get_state());
	log_debug("handle lz motor stopped: %d", (int)FS_DEV.m_motorLZ->is_stopped());
	return true;
#else
	if (!FS_DEV.m_peLZ.get_state()) {
		FS_DEV.m_motorLZ->start_by_max_speed(motor::go_backward);
	}
	else if (!FS_DEV.m_motorLZ->is_stopped()) {
		FS_DEV.m_motorLZ->stop();
	}
	else {
		return true;
	}

	return false;
#endif
}

bool stResetBackward::__handle_rz()
{
#if 0 /// \todo DELETEME
	log_debug("handle rz pe: %d", (int)FS_DEV.m_peRZ.get_state());
	log_debug("handle rz motor stopped: %d", (int)FS_DEV.m_motorRZ->is_stopped());
	return true;
#else
	if (!FS_DEV.m_peRZ.get_state()) {
		FS_DEV.m_motorRZ->start_by_max_speed(motor::go_backward);
	}
	else if (!FS_DEV.m_motorRZ->is_stopped()) {
		FS_DEV.m_motorRZ->stop();
	}
	else {
		return true;
	}

	return false;
#endif
}

sc::result stResetBackward::react(const __evEntryAct &)
{
	bool l = __handle_lz();
	bool r = __handle_rz();
	if (l && r) {
		return __transit();
	}

	return discard_event();
}

sc::result stResetBackward::react(const evMotorStop<motorId_t::LZ> &)
{
	if (!FS_DEV.m_peLZ.get_state()) {
		FS_DEV.m_motorLZ->start_by_max_speed(motor::go_backward);
		return discard_event();
	}

	if (__handle_rz()) {
		return __transit();
	}

	return discard_event();
}

sc::result stResetBackward::react(const evMotorStop<motorId_t::RZ> &)
{
	if (!FS_DEV.m_peRZ.get_state()) {
		FS_DEV.m_motorRZ->start_by_max_speed(motor::go_backward);
		return discard_event();
	}

	if (__handle_lz()) {
		return __transit();
	}

	return discard_event();
}

sc::result stResetBackward::react(const evMotorBackEnd<motorId_t::LZ> &)
{
	if (!FS_DEV.m_motorLZ->is_stopped()) {
		FS_DEV.m_motorLZ->stop();
		return discard_event();
	}

	if (__handle_rz()) {
		return __transit();
	}

	return discard_event();
}

sc::result stResetBackward::react(const evMotorBackEnd<motorId_t::RZ> &)
{
	if (!FS_DEV.m_motorRZ->is_stopped()) {
		FS_DEV.m_motorRZ->stop();
		return discard_event();
	}

	if (__handle_lz()) {
		return __transit();
	}

	return discard_event();
}

sc::result stResetBackward::__transit()
{
	return transit<stResetForward>();
}

/**
 * state: reset forward
 */
struct stResetForward : sc::state< stResetForward, stReset > {
	struct __evEntryAct : sc::event< __evEntryAct > {};

	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::LZ> >,
		sc::custom_reaction< evMotorStop<motorId_t::RZ> >
	> reactions;

	stResetForward(my_context ctx)
	: my_base(ctx)
	, m_lz_done(false)
	, m_rz_done(false)
	{
		post_event(__evEntryAct());
	}
	~stResetForward()
	{
	}

	sc::result react(const __evEntryAct &)
	{
		FS_DEV.m_motorLZ->start_by_step(FS_SPEC.zmotor_reset_forward_step());
		FS_DEV.m_motorRZ->start_by_step(FS_SPEC.zmotor_reset_forward_step());
		return discard_event();
	}
	sc::result react(const evMotorStop<motorId_t::LZ> &)
	{
		log_debug("%s: lz done", FS_STATE_NAME);
		if (m_rz_done) {
			return __transit();
		}
		m_lz_done = true;
		return discard_event();
	}
	sc::result react(const evMotorStop<motorId_t::RZ> &)
	{
		log_debug("%s: rz done", FS_STATE_NAME);
		if (m_lz_done) {
			return __transit();
		}
		m_rz_done = true;
		return discard_event();
	}
private:
	sc::result __transit(void)
	{
		const bool lstopped = FS_DEV.m_motorLZ->is_stopped();
		const bool rstopped = FS_DEV.m_motorRZ->is_stopped();

		if (lstopped && rstopped) {
			return transit<stIdle>();
		}

		log_err("%s: lz %s - rz %s", FS_STATE_NAME,
			lstopped ? "stopped" : "running",
			rstopped ? "stopped" : "running");

		return discard_event();
	}
private:
	bool m_lz_done;
	bool m_rz_done;
};

} /* svcFS */

