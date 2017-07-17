#include <zmsg/zmsg_discharge.hpp>

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.st.cfg)
, m_data(context<fs_sm>().m_ctx.st.data)
{
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

	/// \note report result
	DCL_ZMSG(shrinkage_test_result) & res = FS_DAT;
	log_info("shrinkage test result: %d", (int)(res.code));
	FS_REPORT(res);
}

sc::result stRunning::react(const evCoverOpen &)
{
	FS_DAT.code = fs_err_t::cover_openned;

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

} /* smShrinkageTest */

} /* svcFS */
