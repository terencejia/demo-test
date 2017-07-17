#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise {
/**
 * stPush1_starter
 */
struct stPush1;
stPush1_starter::stPush1_starter(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
	log_debug("%s...", FS_STATE_NAME);
}

stPush1_starter::~stPush1_starter()
{
}

sc::result stPush1_starter::react(const __evEntryAct &)
{
	return transit<stPush1>();
}
/**
 * stPush1
 */
struct stPush1_initial;
struct stPush1
: sc::state< stPush1, stFS_stage1::orthogonal< 1 >, stPush1_initial > {
public:
	stPush1(my_context ctx)
	: my_base(ctx)
	, m_lv_cur(0)
	, m_rv_cur(0)
	, m_lv_dest(0)
	, m_rv_dest(0)
	{
		DCL_ZMSG(fs_state) msg;
		msg.sstate = svc_fs_state_t::fs_push1;
		FS_REPORT(msg);
		log_debug("%s...", FS_STATE_NAME);
	}
	~stPush1()
	{
	}
public:
	int lv_dist(void) const
	{
		return m_lv_dest - m_lv_cur;
	}
	int rv_dist(void) const
	{
		return m_rv_cur - m_rv_dest;
	}
	void init_dest(int lv_dest, int rv_dest)
	{
		m_lv_dest = lv_dest;
		m_rv_dest = rv_dest;
	}
public:
	int m_lv_cur;
	int m_rv_cur;
private:
	/// \note indeed, the m_l/rv_dest should be const.
	int m_lv_dest;
	int m_rv_dest;
};

/**
 * stPush1_initial
 */
struct stPush1_stage1;
struct stPush1_initial
: sc::state< stPush1_initial, stPush1 > {
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

	struct __evCheckFbrIn final : sc::event< __evCheckFbrIn > {
		__evCheckFbrIn(const fs_imgs & i, const cmosId_t cmosid)
		: img(i)
		, id(cmosid)
		{
		}

		const fs_imgs & img;
		const cmosId_t id;
	};

	struct __evFbrAllIn final : sc::event< __evFbrAllIn > {
		__evFbrAllIn(const fs_imgs & i, const cmosId_t cmosid)
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
		sc::custom_reaction< __evCheckFbrIn >,
		sc::custom_reaction< __evFbrAllIn >,
		sc::custom_reaction< evMotorStop<motorId_t::LZ> >,
		sc::custom_reaction< evMotorStop<motorId_t::RZ> >
	> reactions;
public:
	stPush1_initial(my_context ctx)
	: my_base(ctx)
	, m_lf_in(false)
	, m_rf_in(false)
	{
		post_event(__evEntryAct());
		log_debug("%s...", FS_STATE_NAME);
	}
	~stPush1_initial()
	{
		FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
		FS_SM.deactive_waiter<__evCheckFbrIn>(cmosId_t::Y);
		FS_SM.deactive_waiter<__evFbrAllIn>(cmosId_t::Y);
	}

	sc::result react(const __evEntryAct &)
	{
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}
	sc::result react(const __evImgReady & evt)
	{
		const gray_img & img = evt.img.yz_img;
		m_rf_in = img_process::is_rf_in(img, FS_IPP);
		m_lf_in = img_process::is_lf_in(img, FS_IPP);
		log_debug("lf %s in , rf %s in ", (m_lf_in ? "is" : "is not"), (m_rf_in ? "is" : "is not"));

		if (!m_rf_in) {
			FS_DEV.m_motorRZ->start_by_ss(
				FS_SPEC.entering_speed,
				FS_SPEC.zmotor_stroke_step() - FS_SPEC.zmotor_reset_forward_step());
		}

		if (!m_lf_in) {
			FS_DEV.m_motorLZ->start_by_ss(
				FS_SPEC.entering_speed,
				FS_SPEC.zmotor_stroke_step() - FS_SPEC.zmotor_reset_forward_step());
		}

		if (m_lf_in && m_rf_in) {
			return transit<stPush1_stage1>();
		} else {
			FS_SM.active_waiter<__evCheckFbrIn>(cmosId_t::Y);
			return discard_event();
		}
	}

	sc::result react(const __evCheckFbrIn & evt)
	{
		const gray_img & img = evt.img.yz_img;
		m_rf_in = img_process::is_rf_in(img, FS_IPP);
		m_lf_in = img_process::is_lf_in(img, FS_IPP);
		log_debug("lf %s in , rf %s in ", (m_lf_in ? "is" : "is not"), (m_rf_in ? "is" : "is not"));

		if (m_rf_in) {
			FS_DEV.m_motorRZ->stop();
		}

		if (m_lf_in) {
			FS_DEV.m_motorLZ->stop();
		}

		if (m_lf_in && m_rf_in) {
			FS_SM.active_delay_waiter<__evFbrAllIn>(cmosId_t::Y);
		} else {
			FS_SM.active_waiter<__evCheckFbrIn>(cmosId_t::Y);
			return discard_event();
		}

		return discard_event();
	}

	sc::result react(const __evFbrAllIn &)
	{
		return transit<stPush1_stage1>();
	}

	sc::result react(const evMotorStop<motorId_t::LZ> &)
	{
		if (!m_lf_in) {
			FS_DAT.code = fs_err_t::no_fiber;
			return transit<stWaitReset>();
		}

		return discard_event();
	}

	sc::result react(const evMotorStop<motorId_t::RZ> &)
	{
		if (!m_rf_in) {
			FS_DAT.code = fs_err_t::no_fiber;
			return transit<stWaitReset>();
		}

		return discard_event();
	}
private:
	bool m_lf_in;
	bool m_rf_in;
};

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
		log_debug("%s...", FS_STATE_NAME);
	}
	~stPush1_stage1()
	{
		FS_DEV.m_motorLZ->stop();
		FS_DEV.m_motorRZ->stop();
		FS_AUX.m_push_timeout_tmr.stop();
		FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
		FS_SM.deactive_waiter<evMotorLZReady>(cmosId_t::Y);
		FS_SM.deactive_waiter<evMotorRZReady>(cmosId_t::Y);
	}

	sc::result react(const __evEntryAct &)
	{
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const __evImgReady &evt)
	{
		const gray_img & img = evt.img.yz_img;

		context<stPush1>().m_lv_cur = img_process::left_vertex_pos(img, {0}, true);
		context<stPush1>().m_rv_cur = img_process::right_vertex_pos(img, {0}, true);
		const int lv_dest = (evt.img.width() - static_cast<int>(FS_SPEC.yz_clr_discharge_pixel())) / 2;
		const int rv_dest = (evt.img.width() + static_cast<int>(FS_SPEC.yz_clr_discharge_pixel())) / 2;
		context<stPush1>().init_dest(lv_dest, rv_dest);
		log_debug("lf cur %d dest %d , rfcur %d dest %d", context<stPush1>().m_lv_cur, lv_dest, context<stPush1>().m_rv_cur, rv_dest);

		/// left fiber
		{
			m_lf_push_pixels = context<stPush1>().lv_dist();

			if (FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)) {
				FS_DEV.m_motorLZ->stop();
				m_lf_done = true;
			}
			else if (m_lf_push_pixels < 0) {
				log_debug("%s... lf cross over %d", FS_STATE_NAME, m_lf_push_pixels);
				img_process::save_pgm("lf_cross_over.pgm", img);
				FS_DAT.code = fs_err_t::fiber_cross_over;
				return transit<stWaitReset>();
			}
			else {
				const int steps = FS_SPEC.yz_pixel_to_lz_step(m_lf_push_pixels);
				FS_DEV.m_motorLZ->start_by_ss(FS_SPEC.push1_speed, steps);
			}
			log_debug("%s... lf need push %d to dest", FS_STATE_NAME, m_lf_push_pixels);
		}

		/// right fiber
		{
			m_rf_push_pixels = context<stPush1>().rv_dist();

			if (FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
				FS_DEV.m_motorRZ->stop();
				m_rf_done = true;
			}
			else if (m_rf_push_pixels < 0) {
				log_debug("%s... rf cross over %d", FS_STATE_NAME, m_rf_push_pixels);
				img_process::save_pgm("rf_cross_over.pgm", img);
				FS_DAT.code = fs_err_t::fiber_cross_over;
				return transit<stWaitReset>();
			}
			else {
				const int steps = FS_SPEC.yz_pixel_to_rz_step(m_rf_push_pixels);
				FS_DEV.m_motorRZ->start_by_ss(FS_SPEC.push1_speed, steps);
			}
			log_debug("%s... rf need push %d to dest", FS_STATE_NAME, m_rf_push_pixels);
		}

		return __transit();
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

		context<stPush1>().m_lv_cur = img_process::left_vertex_pos(img, {0}, true);
		const int expect_push_pixel = m_lf_push_pixels;
		m_lf_push_pixels = context<stPush1>().lv_dist();
		log_debug("lf expect push %d but %d actually", expect_push_pixel, expect_push_pixel - m_lf_push_pixels);

		if (FS_SPEC.is_z_dist_clr_ok(m_lf_push_pixels)) {
			FS_DEV.m_motorLZ->stop();
			m_lf_done = true;
		}
		else if (m_lf_push_pixels < 0) {
			log_debug("%s... lf cross over %d", FS_STATE_NAME, m_lf_push_pixels);
			img_process::save_pgm("lf_cross_over.pgm", img);
			FS_DAT.code = fs_err_t::fiber_cross_over;
			return transit<stWaitReset>();
		}
		else {
			const int steps = FS_SPEC.yz_pixel_to_lz_step(m_lf_push_pixels);
			FS_DEV.m_motorLZ->start_by_ss(FS_SPEC.push1_speed, steps);
		}

		return __transit();
	}

	sc::result react(const evMotorRZReady & evt)
	{
		const gray_img & img = evt.img.yz_img;

		context<stPush1>().m_rv_cur = img_process::right_vertex_pos(img, {0}, true);
		const int expect_push_pixel = m_rf_push_pixels;
		m_rf_push_pixels = context<stPush1>().rv_dist();
		log_debug("rf expect push %d but %d actually", expect_push_pixel, expect_push_pixel - m_rf_push_pixels);

		if (FS_SPEC.is_z_dist_clr_ok(m_rf_push_pixels)) {
			FS_DEV.m_motorRZ->stop();
			m_rf_done = true;
		}
		else if (m_rf_push_pixels < 0) {
			log_debug("%s... rf cross over %d", FS_STATE_NAME, m_rf_push_pixels);
			img_process::save_pgm("rf_cross_over.pgm", img);
			FS_DAT.code = fs_err_t::fiber_cross_over;
			return transit<stWaitReset>();
		}
		else {
			const int steps = FS_SPEC.yz_pixel_to_rz_step(m_rf_push_pixels);
			FS_DEV.m_motorRZ->start_by_ss(FS_SPEC.push1_speed, steps);
		}

		return __transit();

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
		if (m_lf_done && m_rf_done) {
			return transit<stPush1Done>();
		} else {
			return discard_event();
		}
	}
private:
	int m_lf_push_pixels;
	int m_rf_push_pixels;

	bool m_lf_done;
	bool m_rf_done;
};
/**
 * stPush1Done
 */
stPush1Done::stPush1Done(my_context ctx)
: my_base(ctx)
{
	post_event(evPush1Done());
}

stPush1Done::~stPush1Done()
{
}

}

}
