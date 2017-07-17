#include <sstream>

#include "fs_log.hpp"
#include "img_process.hpp"
#include "math_ext.hpp"

#include "sm_realtime_revise.hpp"

#define STD_ARC_STRENGTH  0.8
#define STD_ARC_TIME  1000
#define STD_FIBER_BR  0.55
#define WAIT_FIBER_COOL 10000

namespace svcFS {

namespace smRealtimeRevise {
/**
 * \brief internal event : wait fiber cool down
 */
struct evWaitFiberCool final : sc::event< evWaitFiberCool > {
};

/**
 * \brief real time revise
 */
stRR::stRR(my_context ctx)
: my_base(ctx)
, m_ximg_abs({0, 0, 0, 0})
, m_yimg_abs({0, 0, 0, 0})
, m_x_ori_exposure(FS_DEV.m_x_exposure->read())
, m_y_ori_exposure(FS_DEV.m_y_exposure->read())
{
	log_debug("%s...", FS_STATE_NAME);

	FS_AUX.m_user_tmr.connect([this](exemodel::poller&, uint64_t) {
			FS_SM.process_event(evWaitFiberCool());
	});

	post_event(__evEntryAct());
}

stRR::~stRR()
{
	bool cover_openned = FS_DEV.m_cover.get_state();
	if(!cover_openned) {
		FS_DEV.m_ledX.enable();
		FS_DEV.m_ledY.enable();
	}
	FS_DEV.m_x_exposure->write(m_x_ori_exposure);
	FS_DEV.m_y_exposure->write(m_y_ori_exposure);
	log_debug("%s...exit", FS_STATE_NAME);
}

sc::result stRR::react(const __evEntryAct &)
{
	return discard_event();
}

/**
 * \brief init state
 */
struct stRR_AdjustExposure;
stRR_init::stRR_init(my_context ctx)
: my_base(ctx)
, m_x_img_abs_ready(false)
, m_y_img_abs_ready(false)
{
	log_debug("%s...in", FS_STATE_NAME);
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
}

stRR_init::~stRR_init()
{
	log_debug("%s...exit", FS_STATE_NAME);
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stRR_init::react(const __evImgReady & evt)
{
	/**
	*\brief can't find fiber's fine row coordinate arter the led brightness and cmos exposure changed,
	*  so store the abstract information for img analysis before they change.
	*/
	switch (evt.id)
	{
		case cmosId_t::X:
		{
			context<stRR>().m_ximg_abs = img_process::get_abstract(evt.img.xz_img, FS_IPP, 0, evt.img.xz_img.width/10);
			m_x_img_abs_ready = true;
			break;
		}
		case cmosId_t::Y:
		{
			context<stRR>().m_yimg_abs = img_process::get_abstract(evt.img.yz_img, FS_IPP, 0, evt.img.yz_img.width/10);
			m_y_img_abs_ready = true;
			break;
		}
		default:
			break;
	}

	if (m_x_img_abs_ready && m_y_img_abs_ready) {
		return transit<stRR_AdjustExposure>();
	} else {
		return discard_event();
	}
}


/**
 * \brief  Adjust cmos exposure
 */
static bool __set_br(const gray_img & img, const img_process::img_abstract_t abs, const double target_br,
			int& min, int& max, double& min_br, double& max_br, std::unique_ptr< exemodel::dev_attr_rw<long> >& exposure_ctl)
{
	double br = img_process::get_br_during_arc(img, abs);
	log_debug("current frame cmos exposure %d : %d : %ld", min, max, exposure_ctl->read());

	if (br < target_br) {
		min = exposure_ctl->read();
		min_br = br;
	}
	else {
		max = exposure_ctl->read();
		max_br = br;
	}

	if (max <= min+1) {
		if (std::abs(min_br - target_br) <= std::abs(max_br - target_br)) {
			exposure_ctl->write(min);
		}
		else {
			exposure_ctl->write(max);
		}
		return true;
	}

	exposure_ctl->write((min + max) / 2);

	return false;
}

struct stRR_FitCurve;
struct stRR_AdjustExposure
: sc::state< stRR_AdjustExposure, stRR > {
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
		sc::custom_reaction< __evImgReady >,
		sc::custom_reaction< evWaitFiberCool >
	> reactions;
public:
	stRR_AdjustExposure(my_context ctx)
	: my_base(ctx)
	, m_x_ok(false)
	, m_y_ok(false)
	, m_target_br(STD_FIBER_BR)
	, m_min_x(FS_HWINFO.cmos_exposure_min)
	, m_max_x(FS_HWINFO.cmos_exposure_max)
	, m_min_y(FS_HWINFO.cmos_exposure_min)
	, m_max_y(FS_HWINFO.cmos_exposure_max)
	, m_min_br_x(0)
	, m_max_br_x(255)
	, m_min_br_y(0)
	, m_max_br_y(255)
	{
		log_debug("%s...", FS_STATE_NAME);
		post_event(__evEntryAct());
	}

	~stRR_AdjustExposure()
	{
		FS_DEV.m_hvb.stop();
		FS_AUX.m_user_tmr.stop();
		FS_SM.deactive_waiter<__evImgReady>();
	}

	sc::result react(const __evEntryAct &)
	{
		FS_DEV.m_ledX.disable();
		FS_DEV.m_ledY.disable();

		FS_AUX.m_user_tmr.start({exemodel::ms_to_timespec(WAIT_FIBER_COOL), exemodel::ms_to_timespec(WAIT_FIBER_COOL)});
		double offset = delta_discharge_magnitude(FS_SPEC.discharge_base, FS_SPEC.discharge_revise,
						FS_SPEC.hvb_pressure_compensate_coefficent1,
						FS_SPEC.hvb_pressure_compensate_coefficent2,
						FS_CFG.Temperature, FS_DEV.m_env_temp.read(),
						FS_CFG.AirPressure, FS_DEV.m_env_pressure.read(),
						STD_ARC_STRENGTH);
		FS_DEV.m_hvb.set_magnitude(STD_ARC_STRENGTH + offset);
		return discard_event();
	}

	sc::result react(const __evImgReady &evt)
	{
		///\brief ensure capture img before the hvb be closed
		FS_DEV.m_hvb.stop();

		if (g_log_debug) {
			std::stringstream x_img_name, y_img_name;
			switch (evt.id)
			{
				case cmosId_t::X:
				{
					double x_br  = img_process::get_br_during_arc(evt.img.xz_img, context<stRR>().m_ximg_abs);
					x_img_name << "x_ae_" << FS_DEV.m_x_exposure->read() << "_" << x_br << ".pgm";
					img_process::save_pgm(x_img_name.str(), evt.img.xz_img);
					break;
				}
				case cmosId_t::Y:
				{
					double y_br  = img_process::get_br_during_arc(evt.img.yz_img, context<stRR>().m_yimg_abs);
					y_img_name << "y_ae_" << FS_DEV.m_y_exposure->read() << "_" << y_br << ".pgm";
					img_process::save_pgm(y_img_name.str(), evt.img.yz_img);
					break;
				}
				default:
					break;
			}
		}

		try {
			switch (evt.id)
			{
				case cmosId_t::X:
				{
					if (!m_x_ok)
						m_x_ok = __set_br(evt.img.xz_img, context<stRR>().m_ximg_abs, m_target_br, m_min_x, m_max_x,
								m_min_br_x, m_max_br_x, FS_DEV.m_x_exposure);
					break;
				}
				case cmosId_t::Y:
				{
					if (!m_y_ok)
						m_y_ok = __set_br(evt.img.yz_img, context<stRR>().m_yimg_abs, m_target_br, m_min_y, m_max_y,
								m_min_br_y, m_max_br_y, FS_DEV.m_y_exposure);
					break;
				}
				default:
					break;
			}

			if (m_x_ok && m_y_ok) {
				log_debug("cmos x exposure : %ld", FS_DEV.m_x_exposure->read());
				log_debug("cmos y exposure : %ld", FS_DEV.m_y_exposure->read());

				FS_DAT.RealtimeReviseData.rt_x_exposure = FS_DEV.m_x_exposure->read();
				FS_DAT.RealtimeReviseData.rt_y_exposure = FS_DEV.m_y_exposure->read();
				return transit<stRR_FitCurve>();
			}
		}
		catch (...) {
			log_err("%s adjust cmos exposure error", FS_STATE_NAME);
			FS_DAT.code = fs_err_t::cmos_exposure;
			return transit<stReset>();
		}

		return discard_event();
	}

	sc::result react(const evWaitFiberCool &)
	{
		FS_SM.active_waiter<__evImgReady>(cmosId_t::X, STD_ARC_TIME);
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y, STD_ARC_TIME);
		FS_DEV.m_hvb.start();

		return discard_event();
	}
private:
	private:
	bool m_x_ok;
	bool m_y_ok;

	double m_target_br;

	int m_min_x;
	int m_max_x;

	int m_min_y;
	int m_max_y;

	double m_min_br_x;
	double m_max_br_x;

	double m_min_br_y;
	double m_max_br_y;
};

struct stRR_FitCurve
: sc::state< stRR_FitCurve, stRR > {
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
		sc::custom_reaction< __evImgReady >,
		sc::custom_reaction< evWaitFiberCool >
	> reactions;
public:
	stRR_FitCurve(my_context ctx)
	: my_base(ctx)
	, m_datay(0)
	, m_arc_min(0.4)
	, m_arc_max(1.2)
	, m_arc_step(0.05)
	, m_cur_arc(m_arc_min)
	{
		FS_AUX.m_user_tmr.start({exemodel::ms_to_timespec(WAIT_FIBER_COOL), exemodel::ms_to_timespec(WAIT_FIBER_COOL)});
		log_debug("%s...", FS_STATE_NAME);
	}

	~stRR_FitCurve()
	{
		FS_DEV.m_hvb.stop();
		FS_AUX.m_user_tmr.stop();
		FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
	}

	bool is_sample_finish(void) const
	{
		///\qustion (m_cur_arc < m_arc_max+m_arc_step && m_cur_arc >= m_arc_min) 无法识别相等的情况等于最大值的情况
		if (m_cur_arc < m_arc_max+m_arc_step && m_cur_arc >= m_arc_min) {
			return false;
		}
		return true;
	}

	sc::result react(const __evImgReady & evt)
	{
		///\brief ensure capture img before the hvb be closed
		FS_DEV.m_hvb.stop();

		double y_br = img_process::get_br_during_arc(evt.img.yz_img, context<stRR>().m_yimg_abs);

		///\todo add offset
		m_datay.push_back({m_cur_arc, y_br});
		m_cur_arc += m_arc_step;

		if (g_log_debug) {
			std::stringstream  y_img_name;
			y_img_name << "y_fc_" << m_cur_arc << "_" << y_br << ".pgm";
			img_process::save_pgm(y_img_name.str(), evt.img.yz_img);
		}

		return discard_event();
	}

	sc::result react(const evWaitFiberCool &)
	{
		if (!is_sample_finish()) {
			double offset = delta_discharge_magnitude(FS_SPEC.discharge_base, FS_SPEC.discharge_revise,
					FS_SPEC.hvb_pressure_compensate_coefficent1,
					FS_SPEC.hvb_pressure_compensate_coefficent2,
					FS_CFG.Temperature, FS_DEV.m_env_temp.read(),
					FS_CFG.AirPressure, FS_DEV.m_env_pressure.read(),
					m_cur_arc);
			FS_DEV.m_hvb.set_magnitude(m_cur_arc + offset);
			FS_SM.active_waiter<__evImgReady>(cmosId_t::Y, STD_ARC_TIME);

			FS_DEV.m_hvb.start();

			return discard_event();
		}
		else {
			const int degree = 3;

			log_debug(".........y...............");
			for (auto i : m_datay) {
				log_debug("%f %f", i.x, i.y);
			}
			m_coefficient.clear();
			mathext::multifit_robust(m_datay, degree, m_coefficient);
			for (auto i : m_coefficient) {
				log_debug("%f", i);
			}

			///\note use the y coefficient
			FS_DAT.RealtimeReviseData.rt_revise_a3 = m_coefficient[3];
			FS_DAT.RealtimeReviseData.rt_revise_a2 = m_coefficient[2];
			FS_DAT.RealtimeReviseData.rt_revise_a1 = m_coefficient[1];
			FS_DAT.RealtimeReviseData.rt_revise_a0 = m_coefficient[0];

			FS_DAT.code = fs_err_t::success;
			return transit<stWaitReset>();
		}
	}

private:
	std::vector<mathext::point_t> m_datay;
	std::vector<double> m_coefficient;
private:
	const double m_arc_min;
	const double m_arc_max;
	const double m_arc_step;
	double m_cur_arc;
};


}/// end of smRealtimeRevise

}/// end of svcFS
