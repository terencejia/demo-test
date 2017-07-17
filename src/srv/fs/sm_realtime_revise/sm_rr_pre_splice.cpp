#include <cmath>

#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "sm_realtime_revise.hpp"
#include "math_ext.hpp"
#include "img_process.hpp"

namespace svcFS {

namespace smRealtimeRevise {

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
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stPreSplice::react(const __evEntryAct &)
{
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);

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

	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.FiberPreFSTime)});
	FS_DEV.m_hvb.start(FS_CFG.FiberPreFSStrength);

	///\note: motor foward in stDischarge1
	context<stNormalDischarge>().m_lz_step = static_cast<uint32_t>(left_step);
	context<stNormalDischarge>().m_rz_step = static_cast<uint32_t>(right_step);

	return discard_event();
}

sc::result stPreSplice::react(const evDischargeTimeout &)
{
	return transit<stDischarge1>();
}

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
