#include <cstring>

#include <zmsg/zmsg_discharge.hpp>

#include "gray_video.hpp"

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.fs.cfg)
, m_last_data(context<fs_sm>().m_ctx.fs.last_data)
, m_data(context<fs_sm>().m_ctx.fs.data)
, m_is_fsing(false)
#ifdef CFG_RECORD_VIDEO
, m_video(new gray_video("/tmp/v.mkv", FS_HWINFO.cmos_win_width, FS_HWINFO.cmos_win_height))
#endif
, m_delta_arc(0.0)
{
	m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));

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

	DCL_ZMSG(fusion_splice_result) & res = FS_DAT;
	log_info("fusion splice result: %d", (int)(res.code));
	FS_REPORT(res);

	if (FS_CFG.RealTimeRevise) {
		if (FS_DAT.code == fs_err_t::success) {
			double & arc_delta = context<stRunning>().m_delta_arc;
			switch(FS_CFG.FSPattern) {
				case fs_pattern_t::automatic:
					FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_offset_auto += arc_delta;
					break;
				case fs_pattern_t::calibrate:
					if (FS_CFG.FiberType == zmsg::to_val(fiber_t::sm)) {
						for (uint i =  zmsg::to_val(fiber_t::sm); i <  zmsg::to_val(fiber_t::max); i++) {
							FS_SPEC.rt_revise_data[i].rt_offset_cal += arc_delta;
							FS_SPEC.rt_revise_data[i].rt_offset_auto = FS_SPEC.rt_revise_data[i].rt_offset_cal;
						}
					} else {
						FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_offset_cal += arc_delta;
					}
					break;
				default:
					break;
			}
		}
		else if (FS_DAT.code == fs_err_t::fiber_broken) {
			switch(FS_CFG.FSPattern) {
				case fs_pattern_t::automatic:
					FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_offset_auto = 0;
					break;
				case fs_pattern_t::calibrate:
					if (FS_CFG.FiberType == zmsg::to_val(fiber_t::sm)) {
						for (uint i = zmsg::to_val(fiber_t::sm); i <  zmsg::to_val(fiber_t::max); i++) {
							FS_SPEC.rt_revise_data[i].rt_offset_cal = 0;
							FS_SPEC.rt_revise_data[i].rt_offset_auto = FS_SPEC.rt_revise_data[i].rt_offset_cal;
						}
					} else {
						FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_offset_cal = 0;
					}
					break;
				default:
					break;
			}
		}

		DCL_ZMSG(realtime_revise_update) msg;
		std::memcpy(msg.rt_revise_data, FS_SPEC.rt_revise_data, sizeof(FS_SPEC.rt_revise_data));
		FS_REPORT(msg);
	}

	FS_DEV.stop();
	FS_DEV.disable();

	FS_AUX.stop();
	FS_DEV.m_display.zoom_init();
	if (g_fs_state_time) {
		uint i=1;
		while ((i+1)<m_stamps.size()) {
			auto ctor_time = std::get<1>(m_stamps.at(i));
			auto dtor_time = std::get<1>(m_stamps.at(i+1));
			auto time = exemodel::timespec_sub(dtor_time, ctor_time);
			log_warning("%s : %4ld", std::get<0>(m_stamps.at(i)).c_str(), time.tv_sec*1000L+time.tv_nsec/1000000L);
			i=i+2;
		}
	}
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
	if (FS_DAT.code != fs_err_t::success && FS_DAT.code != fs_err_t::fiber_broken
		&& FS_DAT.code != fs_err_t::loss_estimate
		&& FS_DAT.code != fs_err_t::tense_test_fail) {
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

	DCL_ZMSG(fusion_splice_result) & msg = FS_DAT;
	if ((msg.code != fs_err_t::success) && (msg.code != fs_err_t::tense_test_fail)
	   && (FS_DAT.code != fs_err_t::loss_estimate)
	   && (msg.code != fs_err_t::fiber_broken)) {
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

sc::result stRunning::react(const evMsg<zmsg::mid_t::fusion_splice_reset> &)
{
	FS_DEV.stop();

	DCL_ZMSG(fusion_splice_result) & msg = FS_DAT;
	if ((msg.code != fs_err_t::success) && (msg.code != fs_err_t::tense_test_fail)
	   && (FS_DAT.code != fs_err_t::loss_estimate)
	   && (msg.code != fs_err_t::fiber_broken)) {
		msg.code = fs_err_t::quit_midway;
		log_debug("fs quit midway");
	}

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

} /* smFastFusionSplicing */

} /* svcFS */
