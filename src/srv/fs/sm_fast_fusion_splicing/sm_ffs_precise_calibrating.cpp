#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

/**
 * \brief stPreciseCalibrating
 */
stPreciseCalibrating::stPreciseCalibrating(my_context ctx)
: my_base(ctx)
, m_x_dest_dist(0)
, m_y_dest_dist(0)
, xz_base(0)
, xz_off(0)
, yz_base(0)
, yz_off(0)
{
}
stPreciseCalibrating::~stPreciseCalibrating()
{
}

/**
 * \brief stPreparePreciseCalibrating
 */
struct stPreciseCalibratingStage1;
stPreparePreciseCalibrating::stPreparePreciseCalibrating(my_context ctx)
: my_base(ctx)
, m_x_done(false)
, m_y_done(false)
{
	post_event(__evEntryAct());
}
stPreparePreciseCalibrating::~stPreparePreciseCalibrating()
{
	FS_SM.deactive_waiter<__evImgReady>();
}
sc::result stPreparePreciseCalibrating::react(const __evEntryAct &)
{
	FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
	FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
	return discard_event();
}
sc::result stPreparePreciseCalibrating::react(const __evImgReady & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
				if (FS_SPEC.x_spliter == 0) {
					FS_SPEC.x_spliter = img_process::get_spliter(evt.img.xz_img, FS_IPP);
				}

				context<stPreciseCalibrating>().xz_base = static_cast<int>(evt.img.xz_img.width / 2
					+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
				context<stPreciseCalibrating>().xz_off = static_cast<int>(FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
					+ FS_SPEC.xz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));

				if (FS_CFG.FiberShift) {
					/// record x core shift value
					const double xc_off = img_process::xy_precise_dist_core(
						evt.img.xz_img, FS_IPP,
						context<stPreciseCalibrating>().xz_base,
						context<stPreciseCalibrating>().xz_off,
						FS_SPEC.x_spliter);
					const double xs_off = img_process::xy_precise_dist_cladding(
						evt.img.xz_img, FS_IPP,
						context<stPreciseCalibrating>().xz_base,
						context<stPreciseCalibrating>().xz_off,
						FS_SPEC.x_spliter);
					context<stPreciseCalibrating>().m_x_dest_dist = (xs_off - xc_off) * 0.25;

					log_debug("%s: x(%f)", FS_STATE_NAME, context<stPreciseCalibrating>().m_x_dest_dist);
				}

				m_x_done = true;
			break;
		case cmosId_t::Y:
			/// \note indeed, we have updated x/y spliter in defect detecting already.
			if (FS_SPEC.y_spliter == 0) {
				FS_SPEC.y_spliter = img_process::get_spliter(evt.img.yz_img, FS_IPP);
			}

			/// \note optimize base/off, calc only once
			context<stPreciseCalibrating>().yz_base = static_cast<int>(evt.img.yz_img.width / 2
				+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
			context<stPreciseCalibrating>().yz_off = static_cast<int>(FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
				+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));

			if (FS_CFG.FiberShift) {
				/// record y core shift value
				const double yc_off = img_process::xy_precise_dist_core(
					evt.img.yz_img, FS_IPP,
					context<stPreciseCalibrating>().yz_base,
					context<stPreciseCalibrating>().yz_off,
					FS_SPEC.y_spliter);
				const double ys_off = img_process::xy_precise_dist_cladding(
					evt.img.yz_img, FS_IPP,
					context<stPreciseCalibrating>().yz_base,
					context<stPreciseCalibrating>().yz_off,
					FS_SPEC.y_spliter);
				context<stPreciseCalibrating>().m_y_dest_dist = (ys_off - yc_off) * 0.25;

				log_debug("%s: y(%f)", FS_STATE_NAME, context<stPreciseCalibrating>().m_y_dest_dist);
			}

			m_y_done = true;
			break;
		default:
			break;
	}

	if (m_x_done && m_y_done) {
		return transit<stPreciseCalibratingStage1>();
	} else {
		return discard_event();
	}
}

/**
 * \brief stPreciseCalibratingStage1
 */
struct stPreciseCalibratingStage1
: sc::state< stPreciseCalibratingStage1, stPreciseCalibrating > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
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
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::X> >,
		sc::custom_reaction< evMotorStop<motorId_t::Y> >,
		sc::custom_reaction< evMotorXReady >,
		sc::custom_reaction< evMotorYReady >
	> reactions;
public:

	stPreciseCalibratingStage1(my_context ctx)
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
		context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));

		post_event(__evEntryAct());
	}

	~stPreciseCalibratingStage1()
	{
		context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));

		log_debug("x pixel  diff  steps  direction");
		for (const auto & i : m_x_motor_data) {
			log_debug("%3.4f %3.4f %d %s", std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<3>(i).c_str());
		}

		log_debug("y pixel  diff  steps  direction");
		for (const auto & i : m_y_motor_data) {
			log_debug("%3.4f %3.4f %d %s", std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<3>(i).c_str());
		}
	}

	sc::result react(const __evEntryAct &)
	{
		FS_SM.active_waiter<evMotorXReady>();
		FS_SM.active_waiter<evMotorYReady>();
		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::X>)
	{
		FS_SM.active_delay_waiter<evMotorXReady>();
		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::Y>)
	{
		FS_SM.active_delay_waiter<evMotorYReady>();
		return discard_event();
	}

	sc::result react(const evMotorXReady & evt)
	{
		double xz_delta = 0;
		switch (FS_CFG.FiberAlignment){
		case align_method_t::clad:
			xz_delta = img_process::xy_precise_dist_cladding(
				evt.img.xz_img, FS_IPP,
				context<stPreciseCalibrating>().xz_base,
				context<stPreciseCalibrating>().xz_off,
				FS_SPEC.x_spliter) - context<stPreciseCalibrating>().m_x_dest_dist;
			break;
		default:
			xz_delta = img_process::xy_precise_dist_core(
				evt.img.xz_img, FS_IPP,
				context<stPreciseCalibrating>().xz_base,
				context<stPreciseCalibrating>().xz_off,
				FS_SPEC.x_spliter) - context<stPreciseCalibrating>().m_x_dest_dist;
			break;
		}


		m_x_done = FS_SPEC.is_xy_dist_ok_for_precise_calibrating(xz_delta, FS_CFG.FiberAlignment);
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

			FS_DEV.m_motorX->start_by_ss(FS_SPEC.motor_xy_precise_calibrate_speed, x_steps);
		} else {
			m_x_motor_data.push_back(std::make_tuple(xz_delta, fiber_moved_pixel, 0, "in target pos"));
		}

		return __transit();
	}

	sc::result react(const evMotorYReady & evt)
	{
		double yz_delta = 0;
		switch (FS_CFG.FiberAlignment){
		case align_method_t::clad:
			yz_delta = img_process::xy_precise_dist_cladding(
				evt.img.yz_img, FS_IPP,
				context<stPreciseCalibrating>().yz_base,
				context<stPreciseCalibrating>().yz_off,
				FS_SPEC.y_spliter) - context<stPreciseCalibrating>().m_y_dest_dist;
			break;
		default:
			yz_delta = img_process::xy_precise_dist_core(
				evt.img.yz_img, FS_IPP,
				context<stPreciseCalibrating>().yz_base,
				context<stPreciseCalibrating>().yz_off,
				FS_SPEC.y_spliter) - context<stPreciseCalibrating>().m_y_dest_dist;
			break;
		}

		m_y_done = FS_SPEC.is_xy_dist_ok_for_precise_calibrating(yz_delta, FS_CFG.FiberAlignment);
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

			FS_DEV.m_motorY->start_by_ss(FS_SPEC.motor_xy_precise_calibrate_speed, y_steps);
		} else {
			m_y_motor_data.push_back( std::make_tuple(yz_delta, fiber_moved_pixel, 0, "in target pos") );
		}

		return __transit();
	}
private:
	sc::result __transit()
	{
		if (m_x_done && m_y_done) {
			if (FS_CFG.Stop2) {
				return transit<stPause2>();
			} else {
				return transit<stRecordOffset>();
			}
		}

		return discard_event();
	}
private:
	double m_x_dist;
	double m_y_dist;

	bool m_x_done;
	bool m_y_done;

	bool m_x_retracing;
	bool m_y_retracing;

	///\brief record the motor movement locus for debug
	std::vector<std::tuple<double, double, int32_t, std::string>> m_x_motor_data;
	std::vector<std::tuple<double, double, int32_t, std::string>> m_y_motor_data;

	///\brief to caculate motor retrace steps
	std::vector<std::tuple<double, int32_t>> m_x_motor_retrace_data;
	std::vector<std::tuple<double, int32_t>> m_y_motor_retrace_data;
};

} /* namespace smFastFusionSplicing */

} /* namespace svcFS */
