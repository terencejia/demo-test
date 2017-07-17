#pragma once

#include <zmsg/zmsg_display_oper.hpp>
#include <zmsg/zmsg_discharge.hpp>
#include <zmsg/zmsg_motor_oper.hpp>

#include "fsconf.h"

#include "../fs_sm.hpp"

class gray_video;

namespace svcFS {

namespace smRealtimeRevise {

/**
 * internal events
 */
struct evStartFs final : sc::event< evStartFs > {
};

/**
 * state: running
 */
struct stFS_stage1;
struct stRunning
: sc::state< stRunning, fs_sm, stFS_stage1 >
, helper<stRunning> {
	typedef boost::mpl::list<
		sc::custom_reaction< evImgProcessError >,
		sc::custom_reaction< evSystemError >,
		sc::custom_reaction< evStartFs >,
		sc::custom_reaction< evCoverOpen >,
		sc::custom_reaction< evMsg<zmsg::mid_t::stop> >,
		sc::custom_reaction< evRecordVideoFrame >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();

	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);
	sc::result react(const evStartFs &);
	sc::result react(const evCoverOpen &);
	sc::result react(const evMsg<zmsg::mid_t::stop> &);
	sc::result react(const evRecordVideoFrame &);
public:
	const rr_cfg_t & m_cfg;
	rr_data_t & m_last_data;
	rr_data_t & m_data;
private:
	bool m_is_fsing;
#ifdef CFG_RECORD_VIDEO
	std::unique_ptr<gray_video> m_video;
#endif
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
		sc::custom_reaction< __evImgReady >,
		sc::custom_reaction< evDischargeTimeout >
	> reactions;
public:
	stClearing(my_context ctx);
	~stClearing();

	sc::result react(const __evEntryAct &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const __evImgReady &);
private:
	bool m_has_arc;
	uint32_t m_clr_times;
};

/**
 * state: defect detecting
 * Note:this step will do auto adjusting X&Y window vertical position
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
struct stPreparePreciseCalibrating;
struct stPreciseCalibrating
: sc::state< stPreciseCalibrating, stRunning, stPreparePreciseCalibrating > {
public:
	stPreciseCalibrating(my_context ctx);
	~stPreciseCalibrating();
public:
	double m_x_dest_dist;
	double m_y_dest_dist;

	int xz_base;
	int xz_off;
	int yz_base;
	int yz_off;
};

struct stPreparePreciseCalibrating
: sc::state< stPreparePreciseCalibrating, stPreciseCalibrating > {
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
	stPreparePreciseCalibrating(my_context ctx);
	~stPreparePreciseCalibrating();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady &);
private:
	bool m_x_done;
	bool m_y_done;
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
 * state: normal fusion splice discharge
 * three substates: stPreSplice \ stDischarge1 \ stDischarge2
 */
struct stPreSplice;
struct stNormalDischarge
: sc::state< stNormalDischarge, stRunning, stPreSplice >
{
public:
	stNormalDischarge(my_context ctx)
	:my_base(ctx)
	, m_lz_step(0)
	, m_rz_step(0)
	{
	}

	~stNormalDischarge()
	{
		FS_DEV.m_hvb.stop();

		bool cover_openned = FS_DEV.m_cover.get_state();
		if(!cover_openned) {
			FS_DEV.m_ledX.enable();
			FS_DEV.m_ledY.enable();
		}
	}
public:
	uint32_t m_lz_step;
	uint32_t m_rz_step;
};


/**
 * state: pre splice
 */
struct stPreSplice
: sc::state< stPreSplice, stNormalDischarge > {
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
		sc::custom_reaction< evDischargeTimeout >
	> reactions;
public:
	stPreSplice(my_context ctx);
	~stPreSplice();

	sc::result react(const __evEntryAct &);
	sc::result react(const __evImgReady &);
	sc::result react(const evDischargeTimeout &);
};

/**
 * state: discharge1
 */
struct stDischarge1
: sc::state< stDischarge1, stNormalDischarge > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evDischargeTimeout >
	> reactions;
public:
	stDischarge1(my_context ctx);
	~stDischarge1();

	sc::result react(const __evEntryAct &);
	sc::result react(const evDischargeTimeout &);
private:
	const bool m_stair_flag;
};

/**
 * state: discharge2
 */
struct stRR;
struct stDischarge2 : sc::state< stDischarge2, stNormalDischarge > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evDischargeTimeout >
	> reactions;
public:
	stDischarge2(my_context ctx);
	~stDischarge2();

	sc::result react(const __evEntryAct &);
	sc::result react(const evDischargeTimeout &);
};

/**
 * state : realtime arc revise
 */
struct stRR_init;
struct stRR
: sc::state< stRR, stRunning, stRR_init > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >
	> reactions;
public:
	stRR(my_context ctx);
	~stRR();

	sc::result react(const __evEntryAct &);
public:
	struct img_process::img_abstract_t m_ximg_abs;
	struct img_process::img_abstract_t m_yimg_abs;
	///store the fiber information before the led lightness and cmos exposure change
private:
	int m_x_ori_exposure;
	int m_y_ori_exposure;
};

struct stRR_init
: sc::state< stRR_init, stRR > {
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
	stRR_init(my_context ctx);
	~stRR_init();

	sc::result react(const __evImgReady &);
private:
	//\note to record fiber abs information
	bool m_x_img_abs_ready;
	bool m_y_img_abs_ready;
};

} /* namespace smRealtimeRevise */

} /* namespace svcFS */
