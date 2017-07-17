#include <fs_log.hpp>

#include "sm_discharge_adjust.hpp"

namespace svcFS {

namespace smDischargeAdjust {

stClearing::stClearing(my_context ctx)
: my_base(ctx)
{
	log_debug("fs: clearing...");
	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.CleanDischargeTime)});
	FS_DEV.m_hvb.start(FS_CFG.CleanDischargeStrength);
	FS_SM.active_waiter<__evImgReady>();
}

stClearing::~stClearing()
{
}

sc::result stClearing::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();
	if (!m_has_arc) {
		log_err("%s abnormal arc", FS_STATE_NAME);
		FS_DAT.code = fs_err_t::abnormal_arc;

		return transit<stReset> ();
	}
	return transit< stDefectDetecting >();
}

sc::result stClearing::react(const __evImgReady & evt)
{
	m_has_arc = img_process::has_electric_arc(evt.img.yz_img, FS_IPP);
	if (!m_has_arc) {
		FS_SM.active_waiter<__evImgReady>();
	}

	return discard_event();
}

}

}
