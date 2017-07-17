#pragma once

#include "../fs_sm.hpp"

namespace svcFS {

namespace smMotorTest {

/**
 * state: running
 */
struct stAdjustBrightness;
struct stRunning
: sc::state< stRunning, fs_sm, stAdjustBrightness > {
	typedef boost::mpl::list<
		sc::custom_reaction< evImgProcessError >,
		sc::custom_reaction< evSystemError >,
		sc::custom_reaction< evCoverOpen >,
		sc::custom_reaction< evMsg<zmsg::mid_t::stop> >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();

	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);
	sc::result react(const evCoverOpen &);
	sc::result react(const evMsg<zmsg::mid_t::stop> &);
public:
	const mt_cfg_t & m_cfg;
	mt_data_t & m_data;
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
	const long m_ori_x;
	const long m_ori_y;

	long m_min_x;
	double m_min_x_l;
	long m_max_x;
	double m_max_x_l;

	long m_min_y;
	double m_min_y_l;
	long m_max_y;
	double m_max_y_l;

	bool m_x_done;
	bool m_y_done;
};

/**
 * state: checker
 */
struct stChecker : sc::state< stChecker, stRunning > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
	> reactions;
public:
	stChecker(my_context ctx);
	~stChecker();

	sc::result react(const __evEntryAct &);
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
	stPush1_initial(my_context ctx);
	~stPush1_initial();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady & evt);
	sc::result react(const __evCheckFbrIn & evt);
	sc::result react(const __evFbrAllIn & evt);
	sc::result react(const evMotorStop<motorId_t::LZ> & evt);
	sc::result react(const evMotorStop<motorId_t::RZ> & evt);
private:
	bool m_lf_in;
	bool m_rf_in;
};

/**
 * state: prepare calibrating
 */
struct stPrepareCalibrating
: sc::state< stPrepareCalibrating, stRunning > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::X> >,
		sc::custom_reaction< evMotorStop<motorId_t::Y> >,
		sc::custom_reaction< evMotorTestTimeout >
	> reactions;
public:
	stPrepareCalibrating(my_context ctx);
	~stPrepareCalibrating();

	sc::result react(const __evEntryAct &);
	sc::result react(const evMotorStop<motorId_t::X> &);
	sc::result react(const evMotorStop<motorId_t::Y> &);
	sc::result react(const evMotorTestTimeout &);
private:
	bool m_x_stopped;
	bool m_y_stopped;

	bool m_reverse_start;
	int32_t m_steps;
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
		sc::custom_reaction< __evImgReady >,
		sc::custom_reaction< evMotorTestTimeout>
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
struct stClearing : sc::state< stClearing, stRunning > {
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
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stClearing(my_context ctx);
	~stClearing();

	sc::result react(const __evEntryAct &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const __evImgReady &);

private:
	bool __is_clearing_needed();
private:
	bool m_has_arc = false;
};

/**
 * state: adjust focus
 */
struct stAdjustFocus
: sc::state< stAdjustFocus, stRunning > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
	> reactions;
public:
	stAdjustFocus(my_context ctx);
	~stAdjustFocus();
public:
	sc::result react(const __evEntryAct &);
};

/**
 * state: defect detecting
 */
struct stDefectDetecting
: sc::state< stDefectDetecting, stRunning >
, helper< stDefectDetecting > {
	typedef boost::mpl::list<
		sc::custom_reaction< evHvbReady >
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
 * state: prepare precise calibrating
 */
struct stPreparePreciseCalibrating
: sc::state< stPreparePreciseCalibrating, stRunning >
, helper< stPreparePreciseCalibrating > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::X> >,
		sc::custom_reaction< evMotorStop<motorId_t::Y> >,
		sc::custom_reaction< evMotorTestTimeout >
	> reactions;
public:
	stPreparePreciseCalibrating(my_context ctx);
	~stPreparePreciseCalibrating();

	sc::result react(const __evEntryAct &);
	sc::result react(const evMotorStop<motorId_t::X> &);
	sc::result react(const evMotorStop<motorId_t::Y> &);
	sc::result react(const evMotorTestTimeout &);
private:
	bool m_x_stopped;
	bool m_y_stopped;

	bool m_reverse_start;
	int32_t m_steps;
};

/**
 * state: precise calibrating
 */
struct stPreciseCalibrating
: sc::state< stPreciseCalibrating, stRunning >
, helper< stPreciseCalibrating > {
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
		sc::custom_reaction< evMotorYReady >,
		sc::custom_reaction< evMotorTestTimeout>
	> reactions;
public:
	stPreciseCalibrating(my_context ctx);
	~stPreciseCalibrating();

	sc::result react(const __evEntryAct &);
	sc::result react(const evMotorStop<motorId_t::X> &);
	sc::result react(const evMotorStop<motorId_t::Y> &);
	sc::result react(const evMotorXReady &);
	sc::result react(const evMotorYReady &);
	sc::result react(const evMotorTestTimeout &);
private:
	sc::result __transit();
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

/**
 * state: restore
 */
struct stRestore : sc::state< stRestore, stRunning > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::LZ> >,
		sc::custom_reaction< evMotorStop<motorId_t::RZ> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::LZ> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::RZ> >,
		sc::custom_reaction< evMotorTestTimeout>
	> reactions;

	stRestore(my_context ctx);
	~stRestore();

	sc::result react(const __evEntryAct &);
	sc::result react(const evMotorStop<motorId_t::LZ> &);
	sc::result react(const evMotorStop<motorId_t::RZ> &);
	sc::result react(const evMotorBackEnd<motorId_t::LZ> &);
	sc::result react(const evMotorBackEnd<motorId_t::RZ> &);
	sc::result react(const evMotorTestTimeout &);
private:
	bool __handle_lz(void);
	bool __handle_rz(void);
	sc::result __transit(void);
};

/**
 * state: electric arc test
 */
struct stEleArcTest : sc::state< stEleArcTest, stRunning >
, helper< stEleArcTest > {
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

	stEleArcTest(my_context ctx);
	~stEleArcTest();

	sc::result react(const evDischargeTimeout &);
	sc::result react(const __evImgReady &);
private:
	sc::result __transit(void);
private:
	bool m_has_arc = false;
};

} /* namespace smMotorTest */

} /* namespace svcFS */
