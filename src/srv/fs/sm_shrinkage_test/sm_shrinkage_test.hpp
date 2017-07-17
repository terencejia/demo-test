#pragma once

#include "../fs_sm.hpp"

namespace svcFS {

namespace smShrinkageTest {

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
		sc::transition< evMsg<zmsg::mid_t::stop>, stReset >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();

	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);
	sc::result react(const evCoverOpen &);
public:
	const st_cfg_t & m_cfg;
	st_data_t & m_data;
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
 * state: adjust brightness
 */
struct stAdjustBrightness
: sc::state< stAdjustBrightness, stRunning > {
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
	stAdjustBrightness(my_context ctx);
	~stAdjustBrightness();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady & evt);

private:
	const double m_ref_br;
	const double m_diff_br;

	/// original brightness
	const double m_ori_x;
	const double m_ori_y;

	double m_min_x;
	double m_max_x;

	double m_min_y;
	double m_max_y;
};

/**
 * state: push1
 */
struct stPush1_initial;
struct stPush1
: sc::state< stPush1, stRunning, stPush1_initial > {
public:
	stPush1(my_context ctx);
	~stPush1();
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
 * state: push1 initial
 */
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
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stPush1_initial(my_context ctx);
	~stPush1_initial();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady & evt);
};

/**
 * state: calibrating
 */
struct stCalibrating
: sc::state< stCalibrating, stRunning > {
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
	stCalibrating(my_context ctx);
	~stCalibrating();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady &);
	sc::result react(const evMotorTestTimeout &);
private:
	int32_t m_x_dist;
	int32_t m_y_dist;
};

/**
 * state: clearing
 */
struct stClearing
: sc::state< stClearing, stRunning > {
	typedef boost::mpl::list<
		sc::custom_reaction< evDischargeTimeout >
	> reactions;
public:
	stClearing(my_context ctx);
	~stClearing();

	sc::result react(const evDischargeTimeout &);
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
};

/**
 * state: stRecordOffset
 */
struct stRecordOffset
: sc::state< stRecordOffset, stRunning > {
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
	stRecordOffset(my_context ctx);
	~stRecordOffset();
public:
	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady &);
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
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evImgReady >,
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< evHvbReady >
	> reactions;
public:
	stDischarge1(my_context ctx);
	~stDischarge1();

	sc::result react(const __evImgReady &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const evHvbReady &);

private:
	double m_left_pos;
	double m_right_pos;
};

} /* namespace smDischargeAdjust */

} /* namespace svcFS */
