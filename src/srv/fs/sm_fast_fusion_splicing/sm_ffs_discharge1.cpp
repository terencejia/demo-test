#include <fs_log.hpp>

#include "fs_spec.hpp"

#include "sm_fast_fusion_splice.hpp"

#include <vector>

namespace svcFS {

namespace smFastFusionSplicing {

static double __get_delta_arc(const std::vector<double> & coefficients, const double & x, const double & y1)
 {
	const std::size_t coe_num = coefficients.size();

	double slope = 0.0;
	for (std::size_t i = 1; i < coe_num; ++i) {
		slope += (double)i * coefficients[i] * std::pow(x, i-1);
	}

	double y = 0.0;
	for (std::size_t i = 0; i < coe_num; ++i) {
		y += coefficients[i] * std::pow(x, i);
	}

	double delta_y = y - y1;
	double delta_x = delta_y / slope;

	return delta_x;
}

stDischarge1::stDischarge1(my_context ctx)
: my_base(ctx)
, m_stair_flag(true)
, m_start_stamp({0, 0})
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_discharge1;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stDischarge1::~stDischarge1()
{
	FS_AUX.m_stair_tmr.stop();
}

static constexpr double stair_last_time = 1000;	/// ms
static constexpr uint32_t stair_interval = 10;	/// ms
static constexpr int rr_sample_time = 1000;	/// ms

bool stDischarge1::__is_stair_enabled() const
{
	return (m_stair_flag && (FS_CFG.Discharge1Time > stair_last_time));
}

sc::result stDischarge1::react(const __evEntryAct &)
{
	FS_DEV.m_motorLZ->start_ss_for_fs(
		FS_CFG.LeftFSSpeed,
		context<stFS_Discharge>().m_lz_step,
		motor::go_forward);
	FS_DEV.m_motorRZ->start_ss_for_fs(
		FS_CFG.RightFSSpeed,
		context<stFS_Discharge>().m_rz_step,
		motor::go_forward);

	if (FS_CFG.Discharge1Time > 0) {
		FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.Discharge1Time)});
		m_start_stamp = exemodel::monotonic_clock_info::get_time();
		if(this->__is_stair_enabled()) {
			double init = FS_CFG.FiberPreFSStrength;
			double target = FS_CFG.Discharge1Strength;
			exemodel::timespec_t begin_time = exemodel::monotonic_timeree::info::get_time();
			FS_AUX.m_stair_tmr.connect([this,init,target,begin_time](exemodel::poller&, uint64_t) {
				///\brief calculate the strength by the timer
				double arc_diff = target - init;
				auto cur = exemodel::monotonic_timeree::info::get_time();
				auto diff = exemodel::timespec_sub(cur, begin_time);
				double time = (double)diff.tv_sec * 1000L + (double)diff.tv_nsec / 1000000L;
				double strength;
				if(time < stair_last_time) {
					strength = (time / stair_last_time) * arc_diff + init;
				}
				else {
					strength = target;
					FS_AUX.m_stair_tmr.stop();
					if (FS_CFG.RealTimeRevise) {
						/// \note we can get image only after reach full discharge 1 strength.
						/// we don't need worry about short Discharge1Time.
						FS_SM.active_waiter<evRealtimeRevise>(cmosId_t::Y, rr_sample_time);
					}
				}

				FS_DEV.m_hvb.set_magnitude(strength);
			});
			FS_DEV.m_hvb.set_magnitude(FS_CFG.FiberPreFSStrength);
			FS_AUX.m_stair_tmr.start({exemodel::ms_to_timespec(stair_interval), exemodel::ms_to_timespec(stair_interval)});
		}
		else {
			FS_DEV.m_hvb.set_magnitude(FS_CFG.Discharge1Strength);

			/// \note we don't need worry about short Discharge1Time.
			if (FS_CFG.RealTimeRevise) {
				FS_SM.active_waiter<evRealtimeRevise>(cmosId_t::Y, rr_sample_time);
			}
		}

		FS_SM.active_waiter<__evImgArcSteady>(cmosId_t::Y, rr_sample_time);

		if (FS_CFG.ConeFS) {
			FS_AUX.m_cone_waiter.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.ConeFSWaitTime)});
		}

		DCL_ZMSG(arc_revise) msg;
		msg.revise = FS_CFG.Discharge1Strength - FS_ORI_CFG.Discharge1Strength;
		FS_REPORT(msg);

		return discard_event();
	}
	else {
		return transit<stDischarge2>();
	}
}

sc::result stDischarge1::react(const evStartCone &)
{
	///\question  	we unuse the \param FS_CFG.ConeFSStretchLength to control the fiber's stretchlength,
	///		motors go backward by \param FS_CFG.ConeFSSpeed until the discharg1 time over;
	///\todo 	cone splicer improvement
	FS_DEV.m_motorLZ->start_by_speed(FS_CFG.ConeFSSpeed, motor::go_backward);
	FS_DEV.m_motorRZ->start_by_speed(FS_CFG.ConeFSSpeed, motor::go_backward);

	return discard_event();
}

sc::result stDischarge1::react(const evDischargeTimeout &)
{
	FS_DEV.m_motorLZ->stop();
	FS_DEV.m_motorRZ->stop();
	FS_DEV.m_hvb.stop();
	return transit<stDischarge2>();
}

sc::result stDischarge1::react(const evRealtimeRevise & evt )
{
	if (FS_SPEC.rt_revise_data[FS_CFG.FiberType].empty())
		return discard_event();

	std::vector<double> coefficients = {
		FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_revise_a0,
		FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_revise_a1,
		FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_revise_a2,
		FS_SPEC.rt_revise_data[FS_CFG.FiberType].rt_revise_a3,
	};

	double real_br = img_process::get_br_during_arc(evt.img.yz_img, context<stFS_Discharge>().m_y_left_abs);
	double arc_delta = __get_delta_arc(coefficients, FS_ORI_CFG.Discharge1Strength, real_br);
	log_debug("arc delta = %f", arc_delta);

	context<stRunning>().m_delta_arc = arc_delta;

	/// \note only calibrate mode will adjust arc time of this time
	if (FS_CFG.FSPattern != fs_pattern_t::calibrate) {
		return discard_event();
	}

	///\brief adjust the Discharge1Time
	double revise_time;
	if (this->__is_stair_enabled()) {
		const double T2 = FS_ORI_CFG.Discharge1Time;
		const double V2 = FS_ORI_CFG.Discharge1Strength;
		const double T1 = FS_ORI_CFG.FiberPreFSTime;
		const double V1 = FS_ORI_CFG.FiberPreFSStrength;

		// const auto t2 = ?;
		const double v2 = V2 - arc_delta;
		const double t1 = T1;
		const double v1 = V1 * v2 / V2;		/// \note just approximation

		const double total_energy = V1 * T1 + V2 * T2 - (V2 - V1) / 2 * stair_last_time;

		revise_time = (total_energy - v1 * t1 + (v2 - v1) / 2 * stair_last_time) / v2;
	}
	else {
		const double T2 = FS_ORI_CFG.Discharge1Time;
		const double V2 = FS_ORI_CFG.Discharge1Strength;
		const double T1 = FS_ORI_CFG.FiberPreFSTime;
		const double V1 = FS_ORI_CFG.FiberPreFSStrength;

		// const auto t2 = ?;
		const double v2 = V2 - arc_delta;
		const double t1 = T1;
		const double v1 = V1 * v2 / V2;		/// \note just approximation

		const double total_energy = std::pow(V1,2) * T1 + std::pow(V2,2) * T2;

		revise_time = (total_energy - std::pow(v1,2) * t1) / std::pow(v2,2);
	}

	const auto target = exemodel::timespec_add(m_start_stamp, exemodel::ms_to_timespec(static_cast<int>(revise_time)));
	FS_AUX.m_discharge_tmr.start_abs({{0, 0}, target});
	log_debug("revise Discharge1Time = %f", revise_time);

	return discard_event();
}

sc::result stDischarge1::react(const __evImgArcSteady & evt)
{
	if (!FS_DEV.m_hvb.is_running()) {
		return discard_event();
	}

	if (FS_DAT.defect_data.check(ifd_all)) {
		log_warning("adjust window based on arc position stopped in the result of defect");
		return discard_event();
	}

	img_process::save_pgm("arc_steady.pgm", evt.img.yz_img);

	const int middle_of_arc_y = img_process::get_arc_horizental_center(
					evt.img.yz_img,
					context<stFS_Discharge>().m_y_left_abs,
					context<stFS_Discharge>().m_y_right_abs);
	int col_off = middle_of_arc_y - evt.img.yz_img.width / 2;
	if (col_off == 0) {
		return discard_event();
	}

	const int col_off_limit = (int)FS_SPEC.yz_nm_to_pixel(125000 / 5);
	log_warning("middle_of_arc_y %d col_off %d with limit %d", middle_of_arc_y, col_off, col_off_limit);

	/// adjust cmos window horizontal coordinate by arc center, x follow y
	/// check if the center of the two fiber-ends is at the right position
	if (col_off > col_off_limit) {
		col_off = col_off_limit;
	}
	else if (col_off < -col_off_limit) {
		col_off = -col_off_limit;
	}

	///\ Y window
	{
		///\WARNNING move_window_pos_* maybe throw exception
		FS_DEV.m_camera.move_window_pos_y(col_off, 0);
		FS_SPEC.window_y_col += col_off;

		DCL_ZMSG(update_window_position) msg;
		msg.is_pos_x = false;
		msg.row = FS_SPEC.window_y_row;
		msg.column = FS_SPEC.window_y_col;
		FS_REPORT(msg);
	}

	///\ x window
	{
		FS_DEV.m_camera.move_window_pos_x(col_off, 0);
		FS_SPEC.window_x_col += col_off;

		DCL_ZMSG(update_window_position) msg;
		msg.is_pos_x = true;
		msg.row = FS_SPEC.window_x_row;
		msg.column = FS_SPEC.window_x_col;
		FS_REPORT(msg);
	}

	return discard_event();
}

}

}
