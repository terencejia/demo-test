#include <fs_log.hpp>

#include "img_process.hpp"
#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

stEleArcTest::stEleArcTest(my_context ctx)
: my_base(ctx)
{
	log_debug("mt: ele arc test...");
	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.Discharge1Time)});
	FS_DEV.m_hvb.start(FS_CFG.Discharge1Strength);
	FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
}

stEleArcTest::~stEleArcTest()
{
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stEleArcTest::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();

	if (!m_has_arc) {
		log_err("%s abnormal arc", FS_STATE_NAME);
		FS_DAT.code = fs_err_t::abnormal_arc;

		return transit<stWaitReset>();
	}

	///\note record the completed arc test times
	FS_DAT.ele_arc_tested_times++;
	return transit<stChecker>();
}

sc::result stEleArcTest::react(const __evImgReady & evt)
{
	m_has_arc = img_process::has_electric_arc(evt.img.yz_img, FS_IPP);

	if (!m_has_arc) {
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
	}

	return discard_event();
}

} /* namespace smMotorTest */

} /* namespace svcFS */

