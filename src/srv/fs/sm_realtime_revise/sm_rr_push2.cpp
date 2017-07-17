#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_realtime_revise.hpp"

#include "fs_debug.hpp"

namespace svcFS {

namespace smRealtimeRevise {

/**
 * stPush2
 */
stPush2::stPush2(my_context ctx)
: my_base(ctx)
, m_lv_cur(0)
, m_rv_cur(0)
, m_lv_dest(0)
, m_rv_dest(0)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_push2;
	FS_REPORT(msg);
}

stPush2::~stPush2()
{
}

/**
 * stPush2_initial
 */
struct stPush2_stage1;
stPush2_initial::stPush2_initial(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}
stPush2_initial::~stPush2_initial()
{
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stPush2_initial::react(const __evEntryAct &)
{
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
	return discard_event();
}

sc::result stPush2_initial::react(const __evImgReady & evt)
{
	const gray_img & img = evt.img.yz_img;

	const double lv_dest = (img.width
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000) * 2
		- FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000)
	) / 2;

	const double rv_dest = (img.width
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000) * 2
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000)
	) / 2;

	context<stPush2>().init_dest(lv_dest, rv_dest);

	context<stPush2>().m_lv_cur = img_process::left_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
	context<stPush2>().m_rv_cur = img_process::right_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);

	return transit<stPush2_stage1>();
}

/**
 * push2 stage1
 */
struct stPush2_stage2;
struct stPush2_stage1
: sc::state< stPush2_stage1, stPush2 >
, helper< stPush2_stage1 > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::LZ> >,
		sc::custom_reaction< evMotorStop<motorId_t::RZ> >,
		sc::custom_reaction< evMotorLZReady >,
		sc::custom_reaction< evMotorRZReady >,
		sc::custom_reaction< evMotorPushTimeout >
	> reactions;
public:
	stPush2_stage1(my_context ctx)
	: my_base(ctx)
	, m_lm_steps(0)
	, m_rm_steps(0)
	{
		FS_AUX.m_push_timeout_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
						FS_SPEC.moter_push_timeout_time()) });
		post_event(__evEntryAct());
	}
	~stPush2_stage1()
	{
		FS_DEV.m_motorLZ->stop();
		FS_DEV.m_motorRZ->stop();
		FS_AUX.m_push_timeout_tmr.stop();
	}

	sc::result react(const __evEntryAct &)
	{
		constexpr double push2_stage1_ratio = 0.6;
		/// \note the error must be less than 5%
		constexpr double push2_stage1_min_pixels = (1 / 0.05);

		const double lf_pixels = context<stPush2>().lv_dist();
		const double rf_pixels = context<stPush2>().rv_dist();

		double lf_push_pixels = lf_pixels * push2_stage1_ratio;
		if (lf_push_pixels < push2_stage1_min_pixels) {
			lf_push_pixels = std::min(lf_pixels, push2_stage1_min_pixels);
		}
		double rf_push_pixels = rf_pixels * push2_stage1_ratio;
		if (rf_push_pixels < push2_stage1_min_pixels) {
			rf_push_pixels = std::min(rf_pixels, push2_stage1_min_pixels);
		}

		if (lf_push_pixels < push2_stage1_min_pixels
		    && rf_push_pixels < push2_stage1_min_pixels) {
			return __transit();
		}

		if (lf_push_pixels >= push2_stage1_min_pixels) {
			m_lm_steps = FS_SPEC.yz_pixel_to_lz_step(lf_push_pixels);
			FS_DEV.m_motorLZ->start_by_ss(FS_SPEC.push1_speed, m_lm_steps);
		}
		if (rf_push_pixels >= push2_stage1_min_pixels) {
			m_rm_steps = FS_SPEC.yz_pixel_to_rz_step(rf_push_pixels);
			FS_DEV.m_motorRZ->start_by_ss(FS_SPEC.push1_speed, m_rm_steps);
		}

		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::LZ> &)
	{
		FS_SM.active_delay_waiter<evMotorLZReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::RZ> &)
	{
		FS_SM.active_delay_waiter<evMotorRZReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const evMotorLZReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		const double lv_cur = img_process::left_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
		const double lz_nm_per_step = FS_SPEC.yz_pixel_to_nm(lv_cur - context<stPush2>().m_lv_cur)/ static_cast<double>(m_lm_steps);
		const double error = std::abs((lz_nm_per_step - FS_SPEC.lz_nm_per_step) / FS_SPEC.lz_nm_per_step);
		if (error < 0.30) {
			log_info("push2 stage1 lz_nm_per_step %f update to %f", FS_SPEC.lz_nm_per_step, lz_nm_per_step);
			FS_SPEC.lz_nm_per_step = lz_nm_per_step;
		}
		else {
			log_err("push2 stage1 lz_nm_per_step %f is bad compare to %f", lz_nm_per_step, FS_SPEC.lz_nm_per_step);
		}

		/// save lv for next state
		context<stPush2>().m_lv_cur = lv_cur;
		m_lm_steps = 0;

		if (m_rm_steps == 0) {
			return __transit();
		}

		return discard_event();
	}
	sc::result react(const evMotorRZReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		const double rv_cur = img_process::right_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
		const double rz_nm_per_step = FS_SPEC.yz_pixel_to_nm(context<stPush2>().m_rv_cur - rv_cur)/ static_cast<double>(m_rm_steps);
		const double error = std::abs((rz_nm_per_step - FS_SPEC.rz_nm_per_step) / FS_SPEC.rz_nm_per_step);
		if (error < 0.30) {
			log_info("push2 stage1 rz_nm_per_step %f update to %f", FS_SPEC.rz_nm_per_step, rz_nm_per_step);
			FS_SPEC.rz_nm_per_step = rz_nm_per_step;
		}
		else {
			log_err("push2 stage1 rz_nm_per_step %f is bad compare to %f", rz_nm_per_step, FS_SPEC.rz_nm_per_step);
		}

		/// save rv for next state
		context<stPush2>().m_rv_cur = rv_cur;
		m_rm_steps = 0;

		if (m_lm_steps == 0) {
			return __transit();
		}

		return discard_event();
	}

	sc::result react(const evMotorPushTimeout &)
	{
		log_debug("%s...stPush1_stage1 timeout", FS_STATE_NAME);
		FS_DAT.code = fs_err_t::push_timeout;

		return transit<stWaitReset>();
	}

private:
	sc::result __transit(void)
	{
		return transit<stPush2_stage2>();
	}
private:
	/// \note 0 means corresponding motor handling done or need not handling.
	/// we should not use motor state, because even motor stopped, maybe we have not handle it.
	int m_lm_steps;
	int m_rm_steps;
};

/**
 * push2 stage2
 */
struct stPush2_stage2
: sc::state< stPush2_stage2, stPush2 >
, helper< stPush2_stage2 > {
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
		  sc::custom_reaction< __evEntryAct >
		, sc::custom_reaction< evMotorStop<motorId_t::LZ> >
		, sc::custom_reaction< evMotorStop<motorId_t::RZ> >
		, sc::custom_reaction< evMotorLZReady >
		, sc::custom_reaction< evMotorRZReady >
		, sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stPush2_stage2(my_context ctx)
	: my_base(ctx)
	, m_lf_push_pixels(0)
	, m_rf_push_pixels(0)
	, m_lf_done(false)
	, m_rf_done(false)
	{
		post_event(__evEntryAct());
	}
	~stPush2_stage2()
	{
		FS_DEV.m_motorLZ->stop();
		FS_DEV.m_motorRZ->stop();
		FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
	}

	sc::result react(const __evEntryAct &)
	{
		m_lf_push_pixels = context<stPush2>().lv_dist();
		m_rf_push_pixels = context<stPush2>().rv_dist();

		if (FS_SPEC.is_z_dist_ok(m_lf_push_pixels)) {
			m_lf_done = true;
		}
		else if (m_lf_push_pixels < 0) {
			log_debug("%s: entry lf cross over", FS_STATE_NAME);
			FS_DAT.code = fs_err_t::fiber_cross_over;

			return transit<stWaitReset>();
		}

		if (FS_SPEC.is_z_dist_ok(m_rf_push_pixels)) {
			m_rf_done = true;
		}
		else if (m_rf_push_pixels < 0) {
			log_debug("%s: entry rf cross over", FS_STATE_NAME);
			FS_DAT.code = fs_err_t::fiber_cross_over;

			return transit<stWaitReset>();
		}

		if (m_lf_done && m_rf_done) {
			return __transit();
		}

		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);

		if (!m_lf_done) {
			const uint32_t lm_steps = FS_SPEC.yz_pixel_to_lz_step(m_lf_push_pixels);
			FS_DEV.m_motorLZ->start_by_ss(FS_SPEC.z_speed_from_pixel(m_lf_push_pixels), lm_steps * FS_SPEC.push2_stage2_dist_coe, motor::go_forward);
		}

		if (!m_rf_done) {
			const uint32_t rm_steps = FS_SPEC.yz_pixel_to_rz_step(m_rf_push_pixels);
			FS_DEV.m_motorRZ->start_by_ss(FS_SPEC.z_speed_from_pixel(m_rf_push_pixels), rm_steps * FS_SPEC.push2_stage2_dist_coe, motor::go_forward);
		}

		return discard_event();
	}

	sc::result react(const __evImgReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		if (!FS_SPEC.is_z_dist_ok(m_lf_push_pixels)) {
			context<stPush2>().m_lv_cur = img_process::left_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
			m_lf_push_pixels = context<stPush2>().lv_dist();
			if (FS_SPEC.is_z_dist_ok(m_lf_push_pixels)) {
				FS_DEV.m_motorLZ->stop();
			}
			else if (m_lf_push_pixels < 0) {
				FS_DEV.m_motorLZ->stop();
				log_err("%s... lf cross over (%3.2f)", FS_STATE_NAME, m_lf_push_pixels);
				FS_DAT.code = fs_err_t::fiber_cross_over;
				return transit<stWaitReset>();
			}
			else {
				FS_DEV.m_motorLZ->set_speed(FS_SPEC.z_speed_from_pixel(m_lf_push_pixels));
			}
			log_debug("%s... lf %3.2f", FS_STATE_NAME, m_lf_push_pixels);
		}

		if (!FS_SPEC.is_z_dist_ok(m_rf_push_pixels)) {
			context<stPush2>().m_rv_cur = img_process::right_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
			m_rf_push_pixels = context<stPush2>().rv_dist();
			if (FS_SPEC.is_z_dist_ok(m_rf_push_pixels)) {
				FS_DEV.m_motorRZ->stop();
			}
			else if (m_rf_push_pixels < 0) {
				FS_DEV.m_motorRZ->stop();
				log_err("%s... rf cross over (%3.2f)", FS_STATE_NAME, m_rf_push_pixels);
				FS_DAT.code = fs_err_t::fiber_cross_over;
				return transit<stWaitReset>();
			}
			else {
				FS_DEV.m_motorRZ->set_speed(FS_SPEC.z_speed_from_pixel(m_rf_push_pixels));
			}
			log_debug("%s... rf %3.2f", FS_STATE_NAME, m_rf_push_pixels);
		}

		if (!FS_SPEC.is_z_dist_ok(m_lf_push_pixels)
		    || !FS_SPEC.is_z_dist_ok(m_rf_push_pixels)) {
			FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		}

		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::LZ> &)
	{
		FS_SM.active_delay_waiter<evMotorLZReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::RZ> &)
	{
		FS_SM.active_delay_waiter<evMotorRZReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const evMotorLZReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		context<stPush2>().m_lv_cur = img_process::left_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
		/// \note we use std::min to avoid image process error
		/// the motor always go foward, so the push_pixels will not increase.
		m_lf_push_pixels = std::min(m_lf_push_pixels, context<stPush2>().lv_dist());
		log_debug("%s... lf final %3.2f", FS_STATE_NAME, m_lf_push_pixels);
		if (!FS_SPEC.is_z_dist_ok(m_lf_push_pixels)) {
			const bool flag = m_lf_push_pixels > 0;
			FS_DAT.code =(flag ?  fs_err_t::no_fiber : fs_err_t::fiber_cross_over);
			log_warning("%s: left %s (%3.1f pixel)", FS_STATE_NAME,
					(flag ? "stucked" : "cross over"),  m_lf_push_pixels);
			return transit<stWaitReset>();
		}

		if (m_rf_done) {
			return __transit();
		}

		m_lf_done = true;

		return discard_event();
	}
	sc::result react(const evMotorRZReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		context<stPush2>().m_rv_cur = img_process::right_vertex_fine_pos(img, FS_IPP, FS_SPEC.y_spliter);
		/// \note we use std::min to avoid image process error
		/// the motor always go foward, so the push_pixels will not increase.
		m_rf_push_pixels = std::min(m_rf_push_pixels, context<stPush2>().rv_dist());
		log_debug("%s... rf final %3.2f", FS_STATE_NAME, m_rf_push_pixels);
		if (!FS_SPEC.is_z_dist_ok(m_rf_push_pixels)) {
			const bool flag = m_rf_push_pixels > 0;
			FS_DAT.code =(flag ?  fs_err_t::no_fiber : fs_err_t::fiber_cross_over);
			log_warning("%s: right %s (%3.1f pixel)", FS_STATE_NAME,
					(flag ? "stucked" : "cross over"),  m_rf_push_pixels);
			return transit<stWaitReset>();
		}

		if (m_lf_done) {
			return __transit();
		}

		m_rf_done = true;

		return discard_event();
	}

private:
	sc::result __transit(void)
	{
		if (FS_DEV.m_motorX && FS_DEV.m_motorY) {
			return transit<stPreciseCalibrating>();
		} else {
			return transit<stRecordOffset>();
		}
	}
private:
	double m_lf_push_pixels;
	double m_rf_push_pixels;

	bool m_lf_done;
	bool m_rf_done;
};

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
