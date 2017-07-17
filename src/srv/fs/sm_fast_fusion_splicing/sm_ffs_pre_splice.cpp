#include <cmath>

#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "sm_fast_fusion_splice.hpp"
#include "math_ext.hpp"
#include "img_process.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stPreSplice::stPreSplice(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_pre_splice;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);


	post_event(__evEntryAct());
}

stPreSplice::~stPreSplice()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stPreSplice::react(const __evEntryAct &)
{
	FS_SM.active_delay_waiter<__evImgReady>();

	return discard_event();
}

sc::result stPreSplice::react(const __evImgReady & evt)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_pre_splice;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);
	log_debug("now pre splicer");

	post_event(evStartFs());

	/// start
	double const lv_cur = img_process::left_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	double const rv_cur = img_process::right_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	int32_t const interval_nm = FS_SPEC.yz_pixel_to_nm(rv_cur - lv_cur);

	int32_t lz_step_base = FS_SPEC.nm_to_real_lz_step(interval_nm + FS_CFG.FiberOverlapSetup * 1000) / 2;
	int32_t rz_step_base = FS_SPEC.nm_to_real_rz_step(interval_nm + FS_CFG.FiberOverlapSetup * 1000) / 2;

	/// \note maybe we need check if the angle is valid.
	int32_t left_step = lz_step_base + FS_SPEC.nm_to_real_lz_step(static_cast<int32_t>(
		FS_DAT.rec_info.wrap_diameter
		* std::tan(deg2rad(FS_DAT.defect_data.left_cut_angle()))));
	int32_t right_step = rz_step_base + FS_SPEC.nm_to_real_rz_step(static_cast<int32_t>(
		FS_DAT.rec_info.wrap_diameter
		* std::tan(deg2rad(FS_DAT.defect_data.right_cut_angle()))));

	if (left_step <= 0 || 0xFFFF <= left_step
	    || right_step <= 0 || 0xFFFF <= right_step) {
		log_err("%s - the step number overflow/underflow: left(%d) right(%d)",
				 FS_STATE_NAME, left_step, right_step);
		throw ;
	}

	if (FS_CFG.FiberPreFSTime) {
		FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.FiberPreFSTime)});
	}
	else {
		post_event(evDischargeTimeout());
	}
	FS_DEV.m_hvb.start(FS_CFG.FiberPreFSStrength);

	///\note: motor foward in stDischarge1
	context<stFS_Discharge>().m_lz_step = static_cast<uint32_t>(left_step);
	context<stFS_Discharge>().m_rz_step = static_cast<uint32_t>(right_step);

	/// \brief store the fiber abstract info for img analysis when led off and cmos exposure change,
	if (FS_CFG.RealTimeRevise && !FS_SPEC.rt_revise_data[FS_CFG.FiberType].empty()) {
		FS_DEV.m_y_exposure->write(FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_y_exposure);
	}

	/// record abstract used for adjusting window for arc
	{
		const int sample_width = evt.img.yz_img.width/10;
		context<stFS_Discharge>().m_y_left_abs = img_process::get_abstract(evt.img.yz_img, FS_IPP, 0, sample_width);
		context<stFS_Discharge>().m_y_right_abs = img_process::get_abstract(evt.img.yz_img, FS_IPP, evt.img.yz_img.width - sample_width, sample_width);
	}
	FS_DEV.m_ledY.disable();

	return discard_event();
}

sc::result stPreSplice::react(const evDischargeTimeout &)
{
	return transit<stDischarge1>();
}

}

}
