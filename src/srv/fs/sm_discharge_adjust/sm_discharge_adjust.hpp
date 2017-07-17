#pragma once

#include "../fs_sm.hpp"

namespace svcFS {

namespace smDischargeAdjust {

/**
 * state: running
 */
struct stCheckConfig;
struct stRunning
: sc::state< stRunning, fs_sm, stCheckConfig > {
	typedef boost::mpl::list<
		sc::custom_reaction< evImgProcessError >,
		sc::custom_reaction< evSystemError >,
		sc::custom_reaction< evCoverOpen >,
		sc::custom_reaction< evMsg<zmsg::mid_t::stop> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::fusion_splice_reset> >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();

	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);
	sc::result react(const evCoverOpen &);
	sc::result react(const evMsg<zmsg::mid_t::stop> &);
	sc::result react(const evMsg<zmsg::mid_t::fusion_splice_reset> &);
public:
	const da_cfg_t & m_cfg;
	da_data_t & m_data;
	da_glb_data_t & m_glb_data;
};

/**
 * state: check adjust_time
 */
struct stCheckConfig
: sc::state< stCheckConfig, stRunning >{
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction<__evEntryAct>
	>reactions;
public:
	stCheckConfig(my_context);
	~stCheckConfig();

	sc::result react(const __evEntryAct &);
};

/**
 * state: stFS_stage1 (including adjust brightness, push1, and calibrating)
 */
struct evAdjustBrightnessDone final : sc::event< evAdjustBrightnessDone > {
};
struct evPush1Done final : sc::event< evPush1Done > {
};
struct evCalibratingDone final : sc::event< evCalibratingDone > {
};

struct stAdjustBrightness_starter;
struct stPush1_starter;
struct stCalibrating_starter;
struct stFS_stage1
: sc::state< stFS_stage1, stRunning,
	boost::mpl::list< stAdjustBrightness_starter, stPush1_starter, stCalibrating_starter > > {
	typedef boost::mpl::list<
		sc::custom_reaction< evAdjustBrightnessDone >,
		sc::custom_reaction< evPush1Done >,
		sc::custom_reaction< evCalibratingDone >
	> reactions;
public:
	stFS_stage1(my_context ctx);
	~stFS_stage1();
public:
	sc::result react(const evAdjustBrightnessDone &);
	sc::result react(const evPush1Done &);
	sc::result react(const evCalibratingDone &);
private:
	sc::result __check_all_state_done(void);
};

struct stAdjustBrightness_starter : sc::state< stAdjustBrightness_starter, stFS_stage1::orthogonal< 0 > > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
		>reactions;
public:
	stAdjustBrightness_starter(my_context ctx);
	~stAdjustBrightness_starter();
public:
	sc::result react(const __evEntryAct &);
};

struct stPush1_starter : sc::state< stPush1_starter, stFS_stage1::orthogonal< 1 > > {
private:
	struct __evEntryAct final : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
		>reactions;
public:
	stPush1_starter(my_context ctx);
	~stPush1_starter();
public:
	sc::result react(const __evEntryAct &);
};

struct stCalibrating_starter : sc::state< stCalibrating_starter, stFS_stage1::orthogonal< 2 > > {
private:
	struct __evEntryAct final : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
		>reactions;
public:
	stCalibrating_starter(my_context ctx);
	~stCalibrating_starter();
public:
	sc::result react(const __evEntryAct &);
};

struct stAdjustBrightnessDone : sc::state< stAdjustBrightnessDone, stFS_stage1::orthogonal< 0 > > {
public:
	stAdjustBrightnessDone(my_context ctx);
	~stAdjustBrightnessDone();
};

struct stPush1Done : sc::state< stPush1Done, stFS_stage1::orthogonal< 1 > > {
public:
	stPush1Done(my_context ctx);
	~stPush1Done();
};

struct stCalibratingDone : sc::state< stCalibratingDone, stFS_stage1::orthogonal< 2 > > {
public:
	stCalibratingDone(my_context ctx);
	~stCalibratingDone();
};

/**
 * state: clearing
 */
struct stClearing
: sc::state< stClearing, stRunning > {
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
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< __evImgReady >
	> reactions;

public:
	stClearing(my_context ctx);
	~stClearing();

	sc::result react(const evDischargeTimeout &);
	sc::result react(const __evImgReady &);
private:
	bool m_has_arc;
};

/**
 * state: defect detecting
 */
struct stPush2;
struct stDefectDetecting
: sc::state< stDefectDetecting, stRunning >
, helper< stDefectDetecting > {
	typedef boost::mpl::list<
		sc::custom_reaction< evHvbReady >,
		sc::transition< evMsg<zmsg::mid_t::go_on>, stPush2 >
	> reactions;
public:
	stDefectDetecting(my_context ctx);
	~stDefectDetecting();

	sc::result react(const evHvbReady &);
private:
	bool m_x_ready;
	bool m_y_ready;
	fs_imgs m_img;
};

/**
 * state: push2
 */
struct stPush2_initial;
struct stPush2
: sc::state< stPush2, stRunning, stPush2_initial > {
public:
	stPush2(my_context ctx);
	~stPush2();
public:
	double lv_dist(void) const
	{
		return m_lv_dest - m_lv_cur;
	}
	double rv_dist(void) const
	{
		return m_rv_cur - m_rv_dest;
	}

	void init_dest(double lv_dest, double rv_dest)
	{
		m_lv_dest = lv_dest;
		m_rv_dest = rv_dest;
	}
public:
	double m_lv_cur;
	double m_rv_cur;
private:
	/// \note indeed, the m_l/rv_dest should be const.
	double m_lv_dest;
	double m_rv_dest;
};

/**
 * state: push2 initial
 */
struct stPush2_initial
: sc::state< stPush2_initial, stPush2 > {
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
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stPush2_initial(my_context ctx);
	~stPush2_initial();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady & evt);
};

/**
 * state: precise calibrating
 */
struct stPreciseCalibrating
: sc::state< stPreciseCalibrating, stRunning > {
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
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stPreciseCalibrating(my_context ctx);
	~stPreciseCalibrating();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady &);
private:
	double m_x_dist;
	double m_y_dist;

	bool m_x_done;
	bool m_y_done;
};

/**
 * state: discharge1
 */
struct stDischarge1
: sc::state< stDischarge1, stRunning >
, helper< stDischarge1 > {
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

	struct __evImgArcSteady final : sc::event< __evImgArcSteady > {
		__evImgArcSteady(const fs_imgs & i, const cmosId_t cmosid)
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
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< evHvbReady >,
		sc::custom_reaction< __evImgArcSteady>
	> reactions;
public:
	stDischarge1(my_context ctx);
	~stDischarge1();

	sc::result react(const __evImgReady &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const evHvbReady &);
	sc::result react(const __evImgArcSteady &);
private:
	double __autoAdjust_dischargeStrength(const double base_shrink_um);

private:
	double m_left_pos;
	double m_right_pos;
private:
	///\brief adjust window by arc center
	bool m_x_ready;
	bool m_y_ready;
	gray_img m_xz_img;
	gray_img m_yz_img;
};

/**
 * state: discharge2
 */
struct stDischarge2
: sc::state< stDischarge2, stRunning >
, helper< stDischarge2 > {
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
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< evHvbReady >
	> reactions;
public:
	stDischarge2(my_context ctx);
	~stDischarge2();

	sc::result react(const __evImgReady &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const evHvbReady &);
private:
	double __autoAdjust_dischargeStrength(const double base_shrink_um);

private:
	double m_left_pos;
	double m_right_pos;
};

/**
 * state: success
 */
struct stSuccess : sc::state< stSuccess, stRunning > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
	> reactions;
public:
	stSuccess(my_context ctx);
	~stSuccess();

	sc::result react(const __evEntryAct &);
};

} /* namespace smDischargeAdjust */

} /* namespace svcFS */
