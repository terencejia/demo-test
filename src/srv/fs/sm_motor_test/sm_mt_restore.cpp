#include <fs_log.hpp>

#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

stRestore::stRestore(my_context ctx)
: my_base(ctx)
{
	log_debug("motor test: restore ...");
	post_event(__evEntryAct());
}

stRestore::~stRestore()
{
	FS_AUX.m_mt_tmr.stop();
}

bool stRestore::__handle_lz()
{
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
}

bool stRestore::__handle_rz()
{
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
}

sc::result stRestore::react(const __evEntryAct &)
{
	FS_AUX.m_mt_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
				10000) });

	bool l = __handle_lz();
	bool r = __handle_rz();
	if (l && r) {
		return __transit();
	}

	return discard_event();
}

sc::result stRestore::react(const evMotorStop<motorId_t::LZ> &)
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

sc::result stRestore::react(const evMotorStop<motorId_t::RZ> &)
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

sc::result stRestore::react(const evMotorBackEnd<motorId_t::LZ> &)
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

sc::result stRestore::react(const evMotorBackEnd<motorId_t::RZ> &)
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

sc::result stRestore::react(const evMotorTestTimeout &)
{
	log_debug("%s... reset timeout", FS_STATE_NAME);
	++FS_DAT.reset;
	FS_DAT.code = fs_err_t::reset_timeout;

	return transit<stReset>();
}

sc::result stRestore::__transit()
{
	DCL_ZMSG(motor_test_result) & msg = FS_DAT;

	if (FS_DAT.ele_arc_tested_times < FS_CFG.ElectricArcTestTimes) {
		msg.code = fs_err_t::ignore;
		FS_REPORT(msg);
		return transit<stEleArcTest>();
	}

	return transit<stChecker>();
}

} /* namespace smMotorTest */

} /* namespace svcFS */

