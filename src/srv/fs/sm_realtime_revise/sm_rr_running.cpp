#include <zmsg/zmsg_discharge.hpp>

#include "gray_video.hpp"

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.rr.cfg)
, m_last_data(context<fs_sm>().m_ctx.rr.last_data)
, m_data(context<fs_sm>().m_ctx.rr.data)
, m_is_fsing(false)
#ifdef CFG_RECORD_VIDEO
, m_video(new gray_video("/tmp/v.mkv", FS_HWINFO.cmos_win_width, FS_HWINFO.cmos_win_height))
#endif
{
	FS_DEV.enable();

#ifdef CFG_RECORD_VIDEO
	/// \todo enable RecordVideo
	// FS_SM.active_waiter<evRecordVideoReady>();
#endif
}

stRunning::~stRunning()
{
#ifdef CFG_RECORD_VIDEO
	FS_SM.deactive_waiter_video<evRecordVideoFrame>();
#endif

	/// \note restore window position to avoid accumulate errors (auto adjust window).
	FS_DEV.m_camera.set_window_pos_x(FS_SPEC.window_x_col, FS_SPEC.window_x_row);
	FS_DEV.m_camera.set_window_pos_y(FS_SPEC.window_y_col, FS_SPEC.window_y_row);

	DCL_ZMSG(discharge_count) msg;
	msg.discharge_count = FS_DEV.m_hvb.get_count();
	if (msg.discharge_count > 0) {
		FS_REPORT(msg);
	}

	DCL_ZMSG(realtime_revise_result) & res = FS_DAT;
	log_info("realtime revise result: %d", (int)(res.code));
	FS_REPORT(res);

	FS_SPEC.rt_revise_data[zmsg::to_val(FS_CFG.FiberType)] = res.RealtimeReviseData;
	FS_SPEC.rt_revise_data[zmsg::to_val(FS_CFG.FiberType)].rt_offset_auto = 0;
	FS_SPEC.rt_revise_data[zmsg::to_val(FS_CFG.FiberType)].rt_offset_cal = 0;
	DCL_ZMSG(realtime_revise_update) rmg;
	std::memcpy(rmg.rt_revise_data, FS_SPEC.rt_revise_data, sizeof(FS_SPEC.rt_revise_data));
	FS_REPORT(rmg);

	FS_DEV.stop();
	FS_DEV.disable();

	FS_AUX.stop();
}

sc::result stRunning::react(const evRecordVideoFrame & evt)
{
#ifdef CFG_RECORD_VIDEO
	m_video->write(evt.img);
#else
	(void)evt;
#endif
	return discard_event();
}

sc::result stRunning::react(const evStartFs &)
{
	m_is_fsing = true;
	return discard_event();
}

sc::result stRunning::react(const evCoverOpen &)
{
	if (FS_DAT.code != fs_err_t::success && FS_DAT.code != fs_err_t::cmos_exposure) {
		FS_DAT.code = fs_err_t::quit_midway;
	}

	if (m_is_fsing) {
		return transit<stWaitReset>();
	}
	else {
		return transit<stReset>();
	}
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::stop> &)
{
	FS_DEV.stop();

	DCL_ZMSG(realtime_revise_result) & msg = FS_DAT;
	if ((msg.code != fs_err_t::success) && (msg.code != fs_err_t::cmos_exposure)) {
		msg.code = fs_err_t::quit_midway;
		log_debug("fs quit midway");
	}

	if (m_is_fsing) {
		return transit<stWaitReset>();
	}
	else {
		return transit<stReset>();
	}
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

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
