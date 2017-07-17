#include <zmsg/zmsg_discharge.hpp>

#include "sm_discharge_adjust.hpp"

namespace svcFS {

namespace smDischargeAdjust {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.da.cfg)
, m_data(context<fs_sm>().m_ctx.da.data)
, m_glb_data(context<fs_sm>().m_ctx.da.glb_data)
{
	/// \note give suggest value for next loop at the very beginning of this service
	FS_DAT.suggest1 = FS_CFG.FiberPreFSStrength;
	FS_DAT.suggest2 = FS_CFG.Discharge1Strength;
	++m_glb_data.cnt;
	FS_DEV.enable();
}

stRunning::~stRunning()
{
	DCL_ZMSG(discharge_count) discharge_count_msg;
	discharge_count_msg.discharge_count = FS_DEV.m_hvb.get_count();
	if (discharge_count_msg.discharge_count > 0) {
		FS_REPORT(discharge_count_msg);
	}

	/// \note restore window position to avoid accumulate errors (auto adjust window).
	FS_DEV.m_camera.set_window_pos_x(FS_SPEC.window_x_col, FS_SPEC.window_x_row);
	FS_DEV.m_camera.set_window_pos_y(FS_SPEC.window_y_col, FS_SPEC.window_y_row);

	FS_DEV.stop();
	FS_DEV.disable();

	FS_AUX.stop();

	/// \note record success cnt
	switch (FS_DAT.code) {
	case fs_err_t::success:
		++FS_GLB_DAT.continuous_success_cnt;
		break;
	case fs_err_t::revise1_mag:
	case fs_err_t::revise2_mag:
		FS_GLB_DAT.continuous_success_cnt = 0;
		break;
	default:
		break;
	}

	log_info("discharge adjust continuous success cnt: %d", FS_GLB_DAT.continuous_success_cnt);

	/// \note report result
	DCL_ZMSG(discharge_adjust_result) & res = FS_DAT;
	log_info("discharge adjust result: %d", (int)(res.code));
	FS_REPORT(res);
}

sc::result stRunning::react(const evCoverOpen &)
{
	FS_DAT.code = fs_err_t::cover_openned;

	return transit<stReset>();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::stop> &)
{
	FS_DAT.code = fs_err_t::quit_midway;
	return transit<stReset>();

}

sc::result stRunning::react(const evImgProcessError &)
{
	FS_DAT.code = fs_err_t::img_process_error;

	return transit<stWaitReset>();
}

sc::result stRunning::react(const evSystemError &)
{
	FS_DAT.code = fs_err_t::system_error;

	return transit<stWaitReset>();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::fusion_splice_reset> &)
{
	FS_DEV.stop();
	FS_DAT.code = fs_err_t::quit_midway;

	return transit<stReset>();
}

} /* smDischargeAdjust */

} /* svcFS */
