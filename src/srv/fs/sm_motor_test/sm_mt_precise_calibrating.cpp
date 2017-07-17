#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "fs_debug.hpp"
#include "img_process.hpp"

#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

/**
 * state: prepare calibrating
 */
struct stTestCalibratingPrecision
: sc::state< stTestCalibratingPrecision, stRunning >
, helper< stTestCalibratingPrecision > {
private:
	struct __evImgReady final : sc::event< __evImgReady > {
		__evImgReady(const fs_imgs & i, const cmosId_t cmosid)
		: img(i)
		, id(cmosid)
		{
		}

		const fs_imgs & img;
		const cmosId_t id;
	};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stTestCalibratingPrecision(my_context ctx)
	: my_base(ctx)
	, m_x_min(1000)
	, m_x_max(-1000)
	, m_y_min(1000)
	, m_y_max(-1000)
	{
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
	}
	~stTestCalibratingPrecision()
	{
	}

	sc::result react(const __evImgReady & evt)
	{
		switch(evt.id)
		{
			case cmosId_t::X:
				{
					const int xz_base = static_cast<int>(evt.img.xz_img.width / 2
						+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
					const int xz_off = static_cast<int>(FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
						+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
					const double xz_delta = img_process::xy_precise_dist_core(evt.img.xz_img, FS_IPP, xz_base, xz_off, FS_SPEC.x_spliter);

					if (xz_delta < m_x_min) {
						m_x_min = xz_delta;
						img_process::save_pgm("xz_min.pgm", evt.img.xz_img);
					}
					if (xz_delta > m_x_max) {
						m_x_max = xz_delta;
						img_process::save_pgm("xz_max.pgm", evt.img.xz_img);
					}

					log_debug("stTestCalibratingPrecision: %+1.3f      xmin(%+1.2f) xmax(%+1.2f)", xz_delta,m_x_min, m_x_max);
				}
				break;
			case cmosId_t::Y:
				{
					const int yz_base = static_cast<int>(evt.img.yz_img.width / 2
						+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
					const int yz_off = static_cast<int>(FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
						+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
					const double yz_delta = img_process::xy_precise_dist_core(evt.img.yz_img, FS_IPP, yz_base, yz_off, FS_SPEC.y_spliter);

					if (yz_delta < m_y_min) {
						m_y_min = yz_delta;
						img_process::save_pgm("yz_min.pgm", evt.img.xz_img);
					}
					if (yz_delta > m_y_max) {
						m_y_max = yz_delta;
						img_process::save_pgm("yz_max.pgm", evt.img.xz_img);
					}

					log_debug("stTestCalibratingPrecision: %+1.3f     ymin(%+1.2f) ymax(%+1.2f)",  yz_delta, m_y_min, m_y_max);
				}
				break;
			default:
				break;
		}

		FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}
private:
	double m_x_min;
	double m_x_max;
	double m_y_min;
	double m_y_max;
};

////////////////////////////////////////////////////////////////////////////////
stPreciseCalibrating::stPreciseCalibrating(my_context ctx)
: my_base(ctx)
, m_x_dist(0)
, m_y_dist(0)
, m_x_done(false)
, m_y_done(false)
, m_x_retracing(false)
, m_y_retracing(false)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_precise_calibrating;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}

stPreciseCalibrating::~stPreciseCalibrating()
{
	FS_AUX.m_mt_tmr.stop();

	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::X);

	log_debug("x pixel  diff  steps  direction");
	for (const auto & i : m_x_motor_data) {
		log_debug("%3.4f %3.4f %d %s", std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<3>(i).c_str());
	}

	log_debug("y pixel  diff  steps  direction");
	for (const auto & i : m_y_motor_data) {
		log_debug("%3.4f %3.4f %d %s", std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<3>(i).c_str());
	}
}
/*
	sc::result react(const evMotorXReady &);
	sc::result react(const evMotorYReady &);
	sc::result react(const evMotorTestTimeout &);*/

sc::result stPreciseCalibrating::react(const __evEntryAct &)
{
	FS_AUX.m_mt_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
			15000) });
	FS_SM.active_waiter<evMotorXReady>();
	FS_SM.active_waiter<evMotorYReady>();
	return discard_event();
}
sc::result stPreciseCalibrating::react(const evMotorStop<motorId_t::X> &)
{
	FS_SM.active_delay_waiter<evMotorXReady>();
	return discard_event();
}
sc::result stPreciseCalibrating::react(const evMotorStop<motorId_t::Y> &)
{
	FS_SM.active_delay_waiter<evMotorYReady>();
	return discard_event();
}

sc::result stPreciseCalibrating::react(const evMotorXReady & evt)
{
	/// \note indeed, we have updated x/y spliter in defect detecting already.
	if (FS_SPEC.x_spliter == 0) {
		FS_SPEC.x_spliter = img_process::get_spliter(evt.img.xz_img, FS_IPP);
	}

	const int xz_base = static_cast<int>(evt.img.xz_img.width / 2
		+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
	const int xz_off = static_cast<int>(FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
		+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
	const double xz_delta = img_process::xy_precise_dist_core(evt.img.xz_img, FS_IPP, xz_base, xz_off, FS_SPEC.x_spliter);

	m_x_done = FS_SPEC.is_xy_dist_ok_for_precise_calibrating(xz_delta);
	const double fiber_moved_pixel = m_x_dist-xz_delta;

	if (!m_x_done) {
		m_x_dist = xz_delta;
		int32_t x_steps = FS_SPEC.pixel_to_xy_step(xz_delta);

		if (m_x_retracing && FS_SPEC.is_fiber_moved(fiber_moved_pixel)) {
			m_x_retracing = false;
			m_x_motor_retrace_data.push_back(std::make_tuple(xz_delta, x_steps));
			FS_SPEC.update_motor_retrace_steps(true, m_x_motor_retrace_data);
			m_x_motor_retrace_data.resize(0);
		}

		if (FS_DEV.m_motorX->will_dir_change(x_steps)) {
			m_x_retracing = true;
			x_steps = (x_steps > 0 ? FS_SPEC.x_motor_retrace_steps : -FS_SPEC.x_motor_retrace_steps);

			m_x_motor_retrace_data.push_back(std::make_tuple(xz_delta, x_steps));
			m_x_motor_data.push_back(std::make_tuple(xz_delta, fiber_moved_pixel, x_steps, "reverse dir"));
		} else {
			if (m_x_retracing)
				m_x_motor_retrace_data.push_back(std::make_tuple(xz_delta, x_steps));

			m_x_motor_data.push_back(std::make_tuple(xz_delta, fiber_moved_pixel, x_steps, (xz_delta > 0 ? "forward" : "backward")));
		}

		FS_DEV.m_motorX->start_by_ss(FS_SPEC.motor_xy_precise_calibrate_speed * 2, x_steps);
	} else {
		m_x_motor_data.push_back(std::make_tuple(xz_delta, fiber_moved_pixel, 0, "in target pos"));
	}

	return __transit();
}

sc::result stPreciseCalibrating::react(const evMotorYReady & evt)
{
	if (FS_SPEC.y_spliter == 0) {
		FS_SPEC.y_spliter = img_process::get_spliter(evt.img.yz_img, FS_IPP);
	}

	const int yz_base = static_cast<int>(evt.img.yz_img.width / 2
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
	const int yz_off = static_cast<int>(FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
	const double yz_delta = img_process::xy_precise_dist_core(evt.img.yz_img, FS_IPP, yz_base, yz_off, FS_SPEC.y_spliter);
	m_y_done = FS_SPEC.is_xy_dist_ok_for_precise_calibrating(yz_delta);
	const double fiber_moved_pixel = m_y_dist -yz_delta;

	if (!m_y_done) {
		m_y_dist = yz_delta;
		int32_t y_steps = FS_SPEC.pixel_to_xy_step(yz_delta);

		if (m_y_retracing && FS_SPEC.is_fiber_moved(fiber_moved_pixel)) {
			m_y_retracing = false;

			m_y_motor_retrace_data.push_back(std::make_tuple(yz_delta, y_steps));
			FS_SPEC.update_motor_retrace_steps(false, m_y_motor_retrace_data);
			m_y_motor_retrace_data.resize(0);
		}

		if (FS_DEV.m_motorY->will_dir_change(y_steps)) {
			m_y_retracing = true;
			y_steps = (y_steps > 0 ? FS_SPEC.y_motor_retrace_steps : -FS_SPEC.y_motor_retrace_steps);

			m_y_motor_data.push_back(std::make_tuple(yz_delta, fiber_moved_pixel, y_steps, "reverse dir"));
			m_y_motor_retrace_data.push_back(std::make_tuple(yz_delta, FS_SPEC.y_motor_retrace_steps));
		} else {
			if (m_y_retracing)
				m_y_motor_retrace_data.push_back(std::make_tuple(yz_delta, y_steps));

			m_y_motor_data.push_back(std::make_tuple(yz_delta, fiber_moved_pixel, y_steps, (yz_delta > 0 ? "forward" : "backward") ));
		}

		FS_DEV.m_motorY->start_by_ss(FS_SPEC.motor_xy_precise_calibrate_speed * 2, y_steps);
	} else {
		m_y_motor_data.push_back( std::make_tuple(yz_delta, fiber_moved_pixel, 0, "in target pos") );
	}

	return __transit();
}

/*
sc::result stPreciseCalibrating::react(const __evImgReady & evt)
{
	switch(evt.id)
	{
		case cmosId_t::X:
			{
				/// \note indeed, we have updated x/y spliter in defect detecting already.
				if (FS_SPEC.x_spliter == 0) {
					FS_SPEC.x_spliter = img_process::get_spliter(evt.img.xz_img, FS_IPP);
				}

				const int xz_base = static_cast<int>(evt.img.xz_img.width / 2
					+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
				const int xz_off = static_cast<int>(FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
					+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
				const double xz_delta = img_process::xy_precise_dist_core(evt.img.xz_img, FS_IPP, xz_base, xz_off, FS_SPEC.x_spliter);
				m_x_done = FS_SPEC.is_xy_dist_ok_for_precise_calibrating_relax(xz_delta);
				if (!m_x_done)
					FS_SM.active_waiter<__evImgReady>(cmosId_t::X);

				if (FS_DEV.m_motorX->is_stopped()) {
					if (!m_x_done) {
						FS_DEV.m_motorX->start_by_speed(
							FS_SPEC.xy_speed_from_pixel(xz_delta),
							xz_delta > 0 ? motor::go_forward : motor::go_backward);
					}
				}
				else {
					if (m_x_done		/// already in postion
					|| (xz_delta > 0 && m_x_dist < 0)
					|| (xz_delta < 0 && m_x_dist > 0)) {		/// different direction
						FS_DEV.m_motorX->stop();
					}
					else {
						FS_DEV.m_motorX->set_speed(FS_SPEC.xy_speed_from_pixel(xz_delta));
					}
				}
				m_x_dist = xz_delta;
			}
			break;
		case cmosId_t::Y:
			{
				if (FS_SPEC.y_spliter == 0) {
					FS_SPEC.y_spliter = img_process::get_spliter(evt.img.yz_img, FS_IPP);
				}

				const int yz_base = static_cast<int>(evt.img.yz_img.width / 2
					+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
				const int yz_off = static_cast<int>(FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
					+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));
				const double yz_delta = img_process::xy_precise_dist_core(evt.img.yz_img, FS_IPP, yz_base, yz_off, FS_SPEC.y_spliter);
				m_y_done = FS_SPEC.is_xy_dist_ok_for_precise_calibrating_relax(yz_delta);
				if (!m_y_done)
					FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);

				if (FS_DEV.m_motorY->is_stopped()) {
					if (!m_y_done) {
						FS_DEV.m_motorY->start_by_speed(
							FS_SPEC.xy_speed_from_pixel(yz_delta),
							yz_delta > 0 ? motor::go_forward : motor::go_backward);
					}
				}
				else {
					if (m_y_done
					|| (yz_delta > 0 && m_y_dist < 0)
					|| (yz_delta < 0 && m_y_dist > 0)) {
						FS_DEV.m_motorY->stop();
					}
					else {
						FS_DEV.m_motorY->set_speed(FS_SPEC.xy_speed_from_pixel(yz_delta));
					}
				}
				m_y_dist = yz_delta;
			}
			break;
		default:
			break;
	}

	log_debug("xy precise dist: %3.3f  %3.3f", m_x_dist, m_y_dist);

	if (m_x_done && m_y_done) {
		++FS_DAT.motor_tested_times;
		///\note record the completed motor test times

		if (debug_precise_calibrating) {
			return transit<stTestCalibratingPrecision>();
		}
		else {
			return transit<stRestore>();
		}
	}

	return discard_event();
}
*/
sc::result stPreciseCalibrating::react(const evMotorTestTimeout &)
{
	log_debug("%s...preciseCalibrating timeout", FS_STATE_NAME);
	++FS_DAT.calibrate;
	FS_DAT.code = fs_err_t::calibrate_timeout;

	///\todo judge whether the machine need to be rebooted
	return transit<stWaitReset>();
}

sc::result stPreciseCalibrating::__transit()
{
	if (m_x_done && m_y_done) {
		++FS_DAT.motor_tested_times;
		///\note record the completed motor test times

		if (debug_precise_calibrating) {
			return transit<stTestCalibratingPrecision>();
		}
		else {
			return transit<stRestore>();
		}
	}

	return discard_event();
}

}

}
