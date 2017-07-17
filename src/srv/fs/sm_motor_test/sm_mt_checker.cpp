#include <fs_log.hpp>

#include "img_process.hpp"

#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

stChecker::stChecker(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}

stChecker::~stChecker()
{
}

sc::result stChecker::react(const __evEntryAct &)
{
	///\note  \param code = \param fs_err_t::ignore just for reporting the motor test status, but not the real motor test result.
	DCL_ZMSG(motor_test_result) & msg = FS_DAT;

	if (FS_DAT.motor_tested_times < FS_CFG.MotorTestTimes) {
		msg.code = fs_err_t::ignore;
		FS_REPORT(msg);
		return transit<stPush1>();
	}
	else if (FS_DAT.ele_arc_tested_times < FS_CFG.ElectricArcTestTimes) {
		msg.code = fs_err_t::ignore;
		FS_REPORT(msg);
		return transit<stEleArcTest>();
	}

	msg.code = fs_err_t::success;
	return transit<stReset>();
}

}

}
