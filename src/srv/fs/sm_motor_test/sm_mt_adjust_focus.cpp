#include <fs_log.hpp>

#include "img_process.hpp"

#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

/**
 * \brief stAdjustFocus
 */
struct stAF;
stAdjustFocus::stAdjustFocus(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_focusing;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	post_event(__evEntryAct());
}
stAdjustFocus::~stAdjustFocus()
{
}

sc::result stAdjustFocus::react(const __evEntryAct &)
{
	return transit<stAF>();
}

/**
 * \brief stAF
 */
template < cmosId_t cmosid >
struct evFocusDone final : sc::event< evFocusDone<cmosid> > {
};
template < cmosId_t cmosid >
struct evFocusFail final : sc::event< evFocusFail<cmosid> > {
};

struct af_data_t {
	const std::function<void(int i)> set_f;
	const std::function<const gray_img & (const fs_imgs & img)> sel_img;
	const std::function<void (void)> save_result;
	const std::function<void (void)> active_delay_waiter;
	const bool is_needed;
	const bool is_left;
	const int low_limit;
	const int high_limit;
	const int big_step;

	const int init;

	typedef struct {
		int f;	/// focus setting
		int v;	/// image estimate value
	} fe_t;
	std::vector<fe_t> search_list;

	int ll;
	int rr;
public:
	int clamp_f(int i) {
		if (i < low_limit) {
			return low_limit;
		}
		else if (i > high_limit) {
			return high_limit;
		}
		else {
			return i;
		}
	}

	const fe_t & add(int f, const fs_imgs & img)
	{
		int est = img_process::definition_estimate(sel_img(img), is_left);
		search_list.push_back({ f, est, });

		return *search_list.rbegin();
	}
};

template< cmosId_t cmosid >
struct stAF_check;
template< cmosId_t cmosid >
struct stAF_stage1;
template< cmosId_t cmosid >
struct stAF_stage2_type1;
template< cmosId_t cmosid >
struct stAF_stage2_type2;
template< cmosId_t cmosid >
struct stAF_stage3;
template< cmosId_t cmosid >
struct stAF_done;
template< cmosId_t cmosid >
struct stAF_fail;

/// \note the parent and child state must all be state or all be simple_state,
/// or the program will coruppt.
struct stAF
: sc::state< stAF, stRunning,
	boost::mpl::list< stAF_check<cmosId_t::X>, stAF_check<cmosId_t::Y> > >
, helper< stAF > {
	typedef boost::mpl::list<
		    sc::custom_reaction< evFocusDone<cmosId_t::X> >
		  , sc::custom_reaction< evFocusDone<cmosId_t::Y> >
		  , sc::custom_reaction< evFocusFail<cmosId_t::X> >
		  , sc::custom_reaction< evFocusFail<cmosId_t::Y> >
	> reactions;
public:
	stAF(my_context ctx)
	: my_base(ctx)
	, data{ {
			[this](int i) -> void { FS_DEV.m_x_focus->write_raw(i); },
			[] (const fs_imgs & img) -> const gray_img & { return img.xz_img; },
			[this](void) -> void { FS_SPEC.x_focal_distance = FS_DEV.m_x_focus->read(); },
			[this](void) -> void { FS_SM.active_delay_waiter< evFocusImg<cmosId_t::X> >(cmosId_t::X); },
			FS_CFG.XImageFocus,
			true,
			0,
			FS_HWINFO.x_liquid_lens_max_val,
			16,
			static_cast<int>(FS_HWINFO.x_liquid_lens_max_val * FS_DEV.m_x_focus->read()),
			decltype(af_data_t::search_list)(),
			0,
			FS_HWINFO.x_liquid_lens_max_val,
		}, {
			[this](int i) -> void { FS_DEV.m_y_focus->write_raw(i); },
			[] (const fs_imgs & img) -> const gray_img & { return img.yz_img; },
			[this](void) -> void { FS_SPEC.y_focal_distance = FS_DEV.m_y_focus->read(); },
			[this](void) -> void { FS_SM.active_delay_waiter< evFocusImg<cmosId_t::Y> >(cmosId_t::Y); },
			FS_CFG.YImageFocus,
			false,
			0,
			FS_HWINFO.y_liquid_lens_max_val,
			16,
			static_cast<int>(FS_HWINFO.y_liquid_lens_max_val * FS_DEV.m_y_focus->read()),
			decltype(af_data_t::search_list)(),
			0,
			FS_HWINFO.y_liquid_lens_max_val,
		}, }
	{
		log_debug("%s...", FS_STATE_NAME);
	}
	~stAF()
	{
	}

	sc::result react(const evFocusDone<cmosId_t::X> &)
	{
		return __check_all_done();
	}
	sc::result react(const evFocusDone<cmosId_t::Y> &)
	{
		return __check_all_done();
	}

	sc::result react(const evFocusFail<cmosId_t::X> &)
	{
		FS_DAT.code = fs_err_t::focus_x;
		log_err("x focus search list error:");
		for (auto & i : data[0].search_list) {
			log_err("      f(%d) v(%d)", i.f, i.v);
		}
		return transit<stWaitReset>();
	}
	sc::result react(const evFocusFail<cmosId_t::Y> &)
	{
		FS_DAT.code = fs_err_t::focus_y;
		log_err("y focus search list error:");
		for (auto & i : data[1].search_list) {
			log_err("      f(%d) v(%d)", i.f, i.v);
		}
		return transit<stWaitReset>();
	}
private:
	sc::result __check_all_done(void)
	{
		if (state_downcast< const stAF_done<cmosId_t::X> * >() != 0
		    && state_downcast< const stAF_done<cmosId_t::Y> * >() != 0)
		{
			log_debug("x focus search list debug:");
			for (auto & i : data[0].search_list) {
				log_debug("      f(%d) v(%d)", i.f, i.v);
			}
			log_debug("y focus search list debug:");
			for (auto & i : data[1].search_list) {
				log_debug("      f(%d) v(%d)", i.f, i.v);
			}
			/// \todo maybe we need re adjust brightness
			return transit<stDefectDetecting>();
		}
		else {
			return discard_event();
		}
	}
public:
	af_data_t data[2];
};


template< typename _T >
static constexpr auto v_of_e(_T e) -> typename std::underlying_type<_T>::type
{
	return static_cast< typename std::underlying_type<_T>::type >(e);
}

/**
 * check
 */
template< cmosId_t cmosid >
struct stAF_check
: sc::state< stAF_check<cmosid>, stAF::orthogonal<v_of_e(cmosid)> >
, helper< stAF_check<cmosid> > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > { };
public:
	/// workaround for template template member
	typedef sc::state< stAF_check<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;

	typedef boost::mpl::list<
		  sc::custom_reaction< __evEntryAct >
		, sc::custom_reaction< evFocusImg<cmosid> >
	> reactions;
public:
	stAF_check(my_context ctx)
	: my_base(ctx)
	{
		this->template post_event(__evEntryAct());
	}
	~stAF_check()
	{
	}

	sc::result react(const __evEntryAct &)
	{
		if (data().is_needed) {
			data().active_delay_waiter();
			return this->template discard_event();
		}
		else {
			return this->template transit< stAF_done<cmosid> >();
		}
	}

	sc::result react(const evFocusImg<cmosid> & evt)
	{
		const auto & res = data().add(data().init, evt.img);
		if (res.v < img_process::fc_lvl_1) {
			if (res.v == img_process::fc_lvl_0) {
				return this->template transit< stAF_stage2_type1<cmosid> >();
			}
			else {
				return this->template transit< stAF_stage2_type2<cmosid> >();
			}
		}
		return this->template transit< stAF_stage1<cmosid> >();
	}
private:
	af_data_t & data(void)
	{
		return this->template context<stAF>().data[v_of_e(cmosid)];
	}
};

/**
 * stage 1
 * \brief find fc_lvl_0[_x]
 */
template< cmosId_t cmosid >
struct stAF_stage1
: sc::state< stAF_stage1<cmosid>, stAF::orthogonal<v_of_e(cmosid)> >
, helper< stAF_stage1<cmosid> > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > { };
public:
	/// workaround for template template member
	typedef sc::state< stAF_stage1<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;

	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evFocusImg<cmosid> >
	> reactions;
public:
	stAF_stage1(my_context ctx)
	: my_base(ctx)
	, cur(data().init)
	, step(cur < (data().low_limit + data().high_limit)/2 ? data().big_step : -data().big_step)
	, rev_dir(false)
	, last_v((*data().search_list.cbegin()).v)
	{
		this->template post_event(__evEntryAct());
	}
	~stAF_stage1()
	{
	}

	sc::result react(const __evEntryAct &)
	{
		do {
			const int next = data().clamp_f(cur + step);
			if (next != cur) {
				cur = next;
				break;
			}

			if (rev_dir) {
				return this->template transit< stAF_fail<cmosid> >();
			}
			__reverse_dir();
		} while (true);

		data().set_f(cur);
		data().active_delay_waiter();
		return this->template discard_event();
	}

	sc::result react(const evFocusImg<cmosid> & evt)
	{
		const auto & res = data().add(cur, evt.img);
		if (res.v < img_process::fc_lvl_1) {
			if (res.v == img_process::fc_lvl_0) {
				return this->template transit< stAF_stage2_type1<cmosid> >();
			}
			else {
				return this->template transit< stAF_stage2_type2<cmosid> >();
			}
		}

		if (res.v > last_v) {		/// if falling, reverse dir, and reinit
			if (rev_dir) {
				return this->template transit< stAF_fail<cmosid> >();
			}
			__reverse_dir();
		}
		else if (res.v < last_v) {	/// the dir is right, to avoid reverse dir, so:
			last_v = res.v;
			rev_dir = true;
		}
		/// \note if res.v == last_v, there is no need to update last_v

		return this->react(__evEntryAct());
	}
private:
	af_data_t & data(void)
	{
		return this->template context<stAF>().data[v_of_e(cmosid)];
	}

	void __reverse_dir(void)
	{
		step = -step;
		cur = data().init;
		rev_dir = true;
		last_v = (*data().search_list.cbegin()).v;
	}
private:
	int cur;
	int step;
	bool rev_dir;
	int last_v;
};

/**
 * stage 2 type 1
 */
template< cmosId_t cmosid >
struct stAF_stage2_type1
: sc::state< stAF_stage2_type1<cmosid>, stAF::orthogonal<v_of_e(cmosid)> >
, helper< stAF_stage2_type1<cmosid> > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > { };
public:
	/// workaround for template template member
	typedef sc::state< stAF_stage2_type1<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;

	typedef boost::mpl::list<
		  sc::custom_reaction< __evEntryAct >
		, sc::custom_reaction< evFocusImg<cmosid> >
	> reactions;
public:
	stAF_stage2_type1(my_context ctx)
	: my_base(ctx)
	, ll((*data().search_list.crbegin()).f)
	, rr(ll)
	, step(data().big_step/2)
	{
		this->template post_event(__evEntryAct());
	}
	~stAF_stage2_type1()
	{
		data().ll = ll;
		data().rr = rr;
	}

	sc::result react(const __evEntryAct &)
	{
		const int next = data().clamp_f(rr + step);
		if (next == rr) {
			return this->template transit< stAF_fail<cmosid> >();
		}

		rr = next;
		data().set_f(rr);
		data().active_delay_waiter();

		return this->template discard_event();
	}

	sc::result react(const evFocusImg<cmosid> & evt)
	{
		const auto & res = data().add(rr, evt.img);
		if (res.v >= img_process::fc_lvl_1) {
			/// reach or over fc_lvl_1, recalc using small step
			rr = ll;
			step = step / 2;
			/// \note here we should not compare with last focus
			return this->react(__evEntryAct());
		}
		else if (res.v == img_process::fc_lvl_0) {
			return this->react(__evEntryAct());
		}
		else {
			return this->template transit< stAF_stage3<cmosid> >();
		}
	}
private:
	af_data_t & data(void)
	{
		return this->template context<stAF>().data[v_of_e(cmosid)];
	}
private:
	int ll;
	int rr;
	int step;
};

/**
 * stage 2 type 2
 */
template< cmosId_t cmosid >
struct stAF_stage2_type2
: sc::state< stAF_stage2_type2<cmosid>, stAF::orthogonal<v_of_e(cmosid)> >
, helper< stAF_stage2_type2<cmosid> > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > { };
public:
	/// workaround for template template member
	typedef sc::state< stAF_stage2_type2<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;

	typedef boost::mpl::list<
		  sc::custom_reaction< __evEntryAct >
		, sc::custom_reaction< evFocusImg<cmosid> >
	> reactions;
public:
	stAF_stage2_type2(my_context ctx)
	: my_base(ctx)
	, ll((*data().search_list.crbegin()).f)
	, rr(ll)
	, step(-10)
	{
		this->template post_event(__evEntryAct());
	}
	~stAF_stage2_type2()
	{
		data().ll = ll;
		data().rr = rr;
	}

	sc::result react(const __evEntryAct &)
	{
		const int next = data().clamp_f(ll + step);
		if (next == ll) {
			return this->template transit< stAF_fail<cmosid> >();
		}

		ll = next;
		data().set_f(ll);
		data().active_delay_waiter();

		return this->template discard_event();
	}

	sc::result react(const evFocusImg<cmosid> & evt)
	{
		const auto & res = data().add(ll, evt.img);
		if (res.v == img_process::fc_lvl_0 || res.v >= img_process::fc_lvl_1) {
			return this->template transit< stAF_stage3<cmosid> >();
		}
		else {
			rr = ll;
		}

		return this->react(__evEntryAct());
	}
private:
	af_data_t & data(void)
	{
		return this->template context<stAF>().data[v_of_e(cmosid)];
	}
private:
	int ll;
	int rr;
	int step;
};

/**
 * stage 3
 */
template< cmosId_t cmosid >
struct stAF_stage3
: sc::state< stAF_stage3<cmosid>, stAF::orthogonal<v_of_e(cmosid)> >
, helper< stAF_stage3<cmosid> > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > { };
public:
	/// workaround for template template member
	typedef sc::state< stAF_stage3<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;

	typedef boost::mpl::list<
		  sc::custom_reaction< __evEntryAct >
		, sc::custom_reaction< evFocusImg<cmosid> >
	> reactions;
public:
	stAF_stage3(my_context ctx)
	: my_base(ctx)
	, ll(data().ll)
	, rr(data().rr)
	, cur(0)
	{
		this->template post_event(__evEntryAct());
	}
	~stAF_stage3()
	{
	}

	sc::result react(const __evEntryAct &)
	{
		if ((ll + 1) >= rr) {
			/// \note ll is the result
			if (ll != data().init) {
				data().save_result();
			}
			return this->template transit< stAF_done<cmosid> >();
		}

		cur = (ll + rr) / 2;

		data().set_f(cur);
		data().active_delay_waiter();

		return this->template discard_event();
	}

	sc::result react(const evFocusImg<cmosid> & evt)
	{
		const auto & res = data().add(cur, evt.img);
		if (res.v == img_process::fc_lvl_0 || res.v >= img_process::fc_lvl_1) {
			ll = cur;
		}
		else {
			rr = cur;
		}

		return this->react(__evEntryAct());
	}
private:
	af_data_t & data(void)
	{
		return this->template context<stAF>().data[v_of_e(cmosid)];
	}
private:
	int ll;
	int rr;
	int cur;
};

template< cmosId_t cmosid >
struct stAF_done
: sc::state< stAF_done<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > {
	/// workaround for template template member
	typedef sc::state< stAF_done<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;
public:
	stAF_done(my_context ctx)
	: my_base(ctx)
	{
		this->template post_event(evFocusDone<cmosid>());
	}
	~stAF_done()
	{
	}
};

template< cmosId_t cmosid >
struct stAF_fail
: sc::state< stAF_fail<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > {
	/// workaround for template template member
	typedef sc::state< stAF_fail<cmosid>, stAF::orthogonal<v_of_e(cmosid)> > my_base;
	using typename my_base::my_context;
public:
	stAF_fail(my_context ctx)
	: my_base(ctx)
	{
		data().set_f(data().init);	/// restore focus
		this->template post_event(evFocusFail<cmosid>());
	}
	~stAF_fail()
	{
	}
private:
	af_data_t & data(void)
	{
		return this->template context<stAF>().data[v_of_e(cmosid)];
	}
};

} /* namespace smFusionSplicing */

} /* namespace svcFS */
