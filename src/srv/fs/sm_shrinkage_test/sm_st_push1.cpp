#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {


/**
 * stPush1
 */
stPush1::stPush1(my_context ctx)
: my_base(ctx)
, m_lv_cur(0)
, m_rv_cur(0)
, m_lv_dest(0)
, m_rv_dest(0)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_push1;
	FS_REPORT(msg);
}

stPush1::~stPush1()
{
}

/**
 * stPush1_initial
 */
struct stPush1_stage1;
stPush1_initial::stPush1_initial(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}
stPush1_initial::~stPush1_initial()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stPush1_initial::react(const __evEntryAct &)
{
	FS_SM.active_delay_waiter<__evImgReady>();
	return discard_event();
}

sc::result stPush1_initial::react(const __evImgReady & evt)
{
	const gray_img & img = evt.img.yz_img;
	img_process::save_pgm("push1_initial.pgm", img);

	const int lv_dest = (evt.img.width() - static_cast<int>(FS_SPEC.yz_clr_discharge_pixel())) / 2;

	const int rv_dest = (evt.img.width() + static_cast<int>(FS_SPEC.yz_clr_discharge_pixel())) / 2;

	context<stPush1>().init_dest(lv_dest, rv_dest);

	context<stPush1>().m_lv_cur = img_process::left_vertex_pos(img, FS_IPP, true);
	context<stPush1>().m_rv_cur = img_process::right_vertex_pos(img, FS_IPP, true);

	return transit<stPush1_stage1>();
}

/**
 * stPush1_stage1
 */

struct stPush1_stage1
: sc::state< stPush1_stage1, stPush1 >
, helper< stPush1_stage1 > {
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
		, sc::custom_reaction< evMotorPushTimeout >
	> reactions;
public:
	stPush1_stage1(my_context ctx)
	: my_base(ctx)
	, m_lf_push_pixels(0)
	, m_rf_push_pixels(0)
	, m_lf_done(false)
	, m_rf_done(false)
	{
		FS_AUX.m_push_timeout_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
						FS_SPEC.moter_push_timeout_time()) });
		post_event(__evEntryAct());
	}
	~stPush1_stage1()
	{
		FS_DEV.m_motorLZ->stop();
		FS_DEV.m_motorRZ->stop();
		FS_AUX.m_push_timeout_tmr.stop();
		FS_SM.deactive_waiter<__evImgReady>();
	}

	sc::result react(const __evEntryAct &)
	{
		m_lf_push_pixels = context<stPush1>().lv_dist();
		m_rf_push_pixels = context<stPush1>().rv_dist();

		if (FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)) {
			m_lf_done = true;
		}
		else if (m_lf_push_pixels < 0) {
			log_debug("%s: entry lf cross over", FS_STATE_NAME);
			FS_DAT.code = fs_err_t::fiber_cross_over;

			return transit<stWaitReset>();
		}

		if (FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
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

		FS_SM.active_waiter<__evImgReady>();

		if (!m_lf_done) {
			FS_DEV.m_motorLZ->start_by_ss(
				FS_SPEC.z_speed_from_pixel(m_lf_push_pixels),
				FS_SPEC.zmotor_stroke_step() - FS_SPEC.zmotor_reset_forward_step());
		}

		if (!m_rf_done) {
			FS_DEV.m_motorRZ->start_by_ss(
				FS_SPEC.z_speed_from_pixel(m_rf_push_pixels),
				FS_SPEC.zmotor_stroke_step() - FS_SPEC.zmotor_reset_forward_step());
		}

		return discard_event();
	}

	sc::result react(const __evImgReady &evt)
	{
		const gray_img & img = evt.img.yz_img;

		if (!FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)) {
			context<stPush1>().m_lv_cur = img_process::left_vertex_pos(img, FS_IPP, true);
			m_lf_push_pixels = context<stPush1>().lv_dist();
			if (FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)) {
				FS_DEV.m_motorLZ->stop();
			}
			else if (m_lf_push_pixels < 0) {
				log_debug("%s... lf cross over %d", FS_STATE_NAME, m_lf_push_pixels);
				FS_DAT.code = fs_err_t::fiber_cross_over;
				return transit<stWaitReset>();
			}
			else {
				FS_DEV.m_motorLZ->set_speed(FS_SPEC.z_speed_from_pixel(m_lf_push_pixels));
			}
			log_debug("%s... lf %d", FS_STATE_NAME, m_lf_push_pixels);
		}

		if (!FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
			context<stPush1>().m_rv_cur = img_process::right_vertex_pos(img, FS_IPP, true);
			if (context<stPush1>().m_rv_cur < 160) {
				img_process::save_pgm("right_error.pgm", img);
			}
			m_rf_push_pixels = context<stPush1>().rv_dist();
			if (FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
				FS_DEV.m_motorRZ->stop();
			}
			else if (m_rf_push_pixels < 0) {
				log_debug("%s... rf cross over %d", FS_STATE_NAME, m_rf_push_pixels);
				FS_DAT.code = fs_err_t::fiber_cross_over;
				return transit<stWaitReset>();
			}
			else {
				FS_DEV.m_motorRZ->set_speed(FS_SPEC.z_speed_from_pixel(m_rf_push_pixels));
			}
			log_debug("%s... rf %d", FS_STATE_NAME, m_rf_push_pixels);
		}

		if (!FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)
			|| !FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
			FS_SM.active_waiter<__evImgReady>();
		}

		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::LZ> &)
	{
		FS_SM.active_delay_waiter<evMotorLZReady>();
		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::RZ> &)
	{
		FS_SM.active_delay_waiter<evMotorRZReady>();
		return discard_event();
	}

	sc::result react(const evMotorLZReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		context<stPush1>().m_lv_cur = img_process::left_vertex_pos(img, FS_IPP, true);
		/// \note we use std::min to avoid image process error
		/// the motor always go foward, so the push_pixels will not increase.
		m_lf_push_pixels = std::min(m_lf_push_pixels, context<stPush1>().lv_dist());
		log_debug("%s... lf final %d", FS_STATE_NAME, m_lf_push_pixels);
		if (!FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)) {
			log_warning("%s: left cross over or not reach dest (%d pixel)", FS_STATE_NAME, m_lf_push_pixels);
			FS_DAT.code = (m_lf_push_pixels < 0 ? fs_err_t::fiber_cross_over : fs_err_t::no_fiber);
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

		context<stPush1>().m_rv_cur = img_process::right_vertex_pos(img, FS_IPP, true);
		/// \note we use std::min to avoid image process error
		/// the motor always go foward, so the push_pixels will not increase.
		m_rf_push_pixels = std::min(m_rf_push_pixels, context<stPush1>().rv_dist());
		log_debug("%s... rf final %d", FS_STATE_NAME, m_rf_push_pixels);
		if (!FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
			log_warning("%s: right cross over or not reach dest (%d pixel)", FS_STATE_NAME, m_rf_push_pixels);
			FS_DAT.code = (m_rf_push_pixels < 0 ? fs_err_t::fiber_cross_over : fs_err_t::no_fiber);
			return transit<stWaitReset>();
		}

		if (m_lf_done) {
			return __transit();
		}

		m_rf_done = true;

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
		return transit<stCalibrating>();
	}
private:
	int m_lf_push_pixels;
	int m_rf_push_pixels;

	bool m_lf_done;
	bool m_rf_done;
};

}

}
