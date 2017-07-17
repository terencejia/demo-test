#include <fs_log.hpp>

#include "img_process.hpp"
#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

stClearing::stClearing(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}

stClearing::~stClearing()
{
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stClearing::react(const __evEntryAct &)
{
	if(!__is_clearing_needed()) {
		return transit< stDefectDetecting >();
	}
	else {
		FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.CleanDischargeTime)});
		FS_DEV.m_hvb.start(FS_CFG.CleanDischargeStrength);

		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}
}

sc::result stClearing::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();

	if (!m_has_arc) {
		log_err("%s abnormal arc", FS_STATE_NAME);
		FS_DAT.code = fs_err_t::abnormal_arc;

		return transit<stWaitReset>();
	}


	if ((FS_CFG.XImageFocus || FS_CFG.YImageFocus)) {
		if (FS_DEV.m_x_focus || FS_DEV.m_y_focus) {
			return transit<stAdjustFocus>();
		}
	}

	return transit< stDefectDetecting >();
}

sc::result stClearing::react(const __evImgReady & evt)
{
	m_has_arc = img_process::has_electric_arc(evt.img.yz_img, FS_IPP);

	if (!m_has_arc) {
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
	}

	return discard_event();
}

bool stClearing::__is_clearing_needed()
{
	if(FS_CFG.CleanDischargeTime == 0) {
		return false;
	}
	else if(FS_DAT.motor_tested_times == 0) {
		return true;
	}
	else if(FS_CFG.CleanArcRate == 0) {
		return false;
	}
	else if((FS_DAT.motor_tested_times+1) % FS_CFG.CleanArcRate == 0) {
		return true;
	}
	else {
		return false;
	}
}

}

}
