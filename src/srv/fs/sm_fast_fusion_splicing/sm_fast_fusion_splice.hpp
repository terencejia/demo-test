#pragma once

#include <zmsg/zmsg_display_oper.hpp>
#include <zmsg/zmsg_discharge.hpp>
#include <zmsg/zmsg_motor_oper.hpp>
#include <zmsg/zmsg_arc.hpp>

#include "fsconf.h"

#include "../fs_sm.hpp"

class gray_video;

namespace svcFS {

namespace smFastFusionSplicing {

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
		sc::custom_reaction< evRecordVideoFrame >,
		sc::custom_reaction< evMsg<zmsg::mid_t::fusion_splice_reset> >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();

	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);
	sc::result react(const evStartFs &);
	sc::result react(const evCoverOpen &);
	sc::result react(const evMsg<zmsg::mid_t::stop> &);
	sc::result react(const evMsg<zmsg::mid_t::fusion_splice_reset> &);
	sc::result react(const evRecordVideoFrame &);
public:
	const fs_cfg_t & m_cfg;
	fs_data_t & m_last_data;
	fs_data_t & m_data;
private:
	bool m_is_fsing;
#ifdef CFG_RECORD_VIDEO
	std::unique_ptr<gray_video> m_video;
#endif
public:
	double m_delta_arc;
	std::vector<std::tuple<std::string, exemodel::timespec_t>> m_stamps;
};

/**
 * state: stFS_stage1 (including adjust brightness, push1, calibrating, and adjust focus)
 */
struct evAdjustBrightnessDone final : sc::event< evAdjustBrightnessDone > {
};
struct evPush1Done final : sc::event< evPush1Done > {
};
struct evCalibratingDone final : sc::event< evCalibratingDone > {
};
struct evAdjustFocusDone final : sc::event< evAdjustFocusDone > {
};

struct stAdjustBrightness_starter;
struct stPush1_starter;
struct stCalibrating_starter;
struct stAdjustFocus;
struct stFS_stage1
: sc::state< stFS_stage1, stRunning,
	boost::mpl::list< stAdjustBrightness_starter, stPush1_starter, stCalibrating_starter, stAdjustFocus > > {
	typedef boost::mpl::list<
		sc::custom_reaction< evAdjustBrightnessDone >,
		sc::custom_reaction< evPush1Done >,
		sc::custom_reaction< evCalibratingDone >,
		sc::custom_reaction< evAdjustFocusDone >
	> reactions;
public:
	stFS_stage1(my_context ctx);
	~stFS_stage1();
public:
	sc::result react(const evAdjustBrightnessDone &);
	sc::result react(const evPush1Done &);
	sc::result react(const evCalibratingDone &);
	sc::result react(const evAdjustFocusDone &);
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

/**
 * state : waiting
 * \brief if \param AutoStart off and \param FiberAutoFeed on ;
 * step1: push fiber to the clearing position
 * step2: pause and wait for \msg go_on
 */
struct stClearing;
struct stWaiting : sc::state< stWaiting, stRunning >
, helper< stWaiting > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::transition< evMsg<zmsg::mid_t::go_on>, stClearing >
	> reactions;
public:
	stWaiting(my_context ctx);
	~stWaiting();

	sc::result react(const __evEntryAct &);
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
 * state: adjust focus
 */
struct stAdjustFocus
: sc::state< stAdjustFocus, stFS_stage1::orthogonal< 3 > > {
private:
	struct __evEntryAct final : sc::event < __evEntryAct > {};
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

struct stAdjustFocusDone : sc::state< stAdjustFocusDone, stFS_stage1::orthogonal< 3 > > {
public:
	stAdjustFocusDone(my_context ctx);
	~stAdjustFocusDone();
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
 * state: pause1
 */
struct stPreciseCalibrating;
struct stPause1
: sc::state< stPause1, stRunning > {
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
		sc::custom_reaction< evMsg<zmsg::mid_t::motor_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::motor_stop> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::LZ> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::RZ> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::go_on> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::skip> >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stPause1(my_context ctx);
	~stPause1();
public:
	sc::result react(const evMsg<zmsg::mid_t::motor_start> & evt);
	sc::result react(const evMsg<zmsg::mid_t::motor_stop> & evt);

	sc::result react(const evMotorBackEnd<motorId_t::LZ> & evt);
	sc::result react(const evMotorBackEnd<motorId_t::RZ> & evt);

	sc::result react(const evMsg<zmsg::mid_t::go_on> & evt);
	sc::result react(const evMsg<zmsg::mid_t::skip> & evt);

	sc::result react(const __evImgReady &);
private:
	double m_lzdiff;
	double m_rzdiff;

	double m_maxzdiff;
	double m_minzdiff;
private:
	evMsg<zmsg::mid_t::motor_start> m_evtms;
	bool m_ProcessImg;
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
 * state: pause2
 */
struct stRecordOffset;
struct stPause2
: sc::state< stPause2, stRunning > {
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
		sc::custom_reaction< evMsg<zmsg::mid_t::motor_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::motor_stop> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::LZ> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::RZ> >,
		sc::transition< evMsg<zmsg::mid_t::go_on >, stRecordOffset >,
		sc::transition< evMsg<zmsg::mid_t::skip >, stRecordOffset >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stPause2(my_context ctx);
	~stPause2();
public:
	sc::result react(const evMsg<zmsg::mid_t::motor_start> & evt);
	sc::result react(const evMsg<zmsg::mid_t::motor_stop> & evt);

	sc::result react(const evMotorBackEnd<motorId_t::LZ> & evt);
	sc::result react(const evMotorBackEnd<motorId_t::RZ> & evt);

	sc::result react(const __evImgReady &);
private:
	double m_lzdiff;
	double m_rzdiff;

	double m_maxzdiff;
	double m_minzdiff;
private:
	evMsg<zmsg::mid_t::motor_start> m_evtms;
	bool m_ProcessImg;
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
 * state: fusion splice discharge
 * three substates: stPreSplice \ stDischarge1 \ stDischarge2
 */
struct stPreSplice;
struct stFS_Discharge
: sc::state< stFS_Discharge, stRunning, stPreSplice >
{
public:
	stFS_Discharge(my_context ctx)
	:my_base(ctx)
	, m_y_left_abs({0, 0, 0, 0})
	, m_y_right_abs({0, 0, 0, 0})
	, m_x_cmos_ori_exposure(FS_DEV.m_x_exposure->read())
	, m_y_cmos_ori_exposure(FS_DEV.m_y_exposure->read())
	, m_lz_step(0)
	, m_rz_step(0)
	{
		context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
	}
	~stFS_Discharge()
	{
		context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));

		FS_DEV.m_hvb.stop();

		FS_DEV.m_x_exposure->write(m_x_cmos_ori_exposure);
		FS_DEV.m_y_exposure->write(m_y_cmos_ori_exposure);

		bool cover_openned = FS_DEV.m_cover.get_state();
		if(!cover_openned) {
			FS_DEV.m_ledX.enable();
			FS_DEV.m_ledY.enable();
		}
	}
public:
	struct img_process::img_abstract_t m_y_left_abs;
	struct img_process::img_abstract_t m_y_right_abs;
	///store the fiber information before the led lightness and cmos exposure change
private:
	const long m_x_cmos_ori_exposure;
	const long m_y_cmos_ori_exposure;
public:
	uint32_t m_lz_step;
	uint32_t m_rz_step;
};

/**
 * state: pre splice
 */
struct stPreSplice
: sc::state< stPreSplice, stFS_Discharge > {
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
: sc::state< stDischarge1, stFS_Discharge >
, helper<stDischarge1> {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
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
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evStartCone >,
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< evRealtimeRevise >,
		sc::custom_reaction< __evImgArcSteady >
	> reactions;
public:
	stDischarge1(my_context ctx);
	~stDischarge1();

	sc::result react(const __evEntryAct &);
	sc::result react(const evStartCone &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const evRealtimeRevise &);
	sc::result react(const __evImgArcSteady &);
private:
	bool __is_stair_enabled(void) const;
private:
	const bool m_stair_flag;
	exemodel::timespec_t m_start_stamp;
};

/**
 * state: discharge2
 */
struct stDischarge2 : sc::state< stDischarge2, stFS_Discharge > {
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
 * state: discharge manual
 * Note:this step will do loss estimating
 */
struct stTenseTest;
struct stDischargeManual
: sc::state< stDischargeManual, stRunning > {
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
		sc::custom_reaction< evMsg<zmsg::mid_t::discharge> >,
		sc::custom_reaction< evDischargeTimeout >,
		sc::custom_reaction< __evImgReady >,
		sc::transition< evMsg<zmsg::mid_t::go_on >, stTenseTest >
	> reactions;
public:
	stDischargeManual(my_context ctx);
	~stDischargeManual();

	sc::result react(const evMsg<zmsg::mid_t::discharge> &);
	sc::result react(const evDischargeTimeout &);
	sc::result react(const __evImgReady &);

private:
	uint32_t m_discharge_counts;
	bool m_is_discharge;
	bool m_is_processImg;
private:
	bool m_x_ready;
	bool m_y_ready;
	fs_imgs m_img;
};

/**
 * state: tense test
 */
struct stTenseTest
: sc::state< stTenseTest, stRunning > {
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
		sc::custom_reaction< evMotorStop<motorId_t::LZ> >,
		sc::custom_reaction< evMotorStop<motorId_t::RZ> >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stTenseTest(my_context ctx);
	~stTenseTest();

	sc::result react(const __evEntryAct &);
	sc::result react(const evMotorStop<motorId_t::LZ> &);
	sc::result react(const evMotorStop<motorId_t::RZ> &);
	sc::result react(const __evImgReady &);
private:
	sc::result __check(void);
private:
	bool m_lz_ok;
	bool m_rz_ok;
};

} /* namespace smFastFusionSplicing */

} /* namespace svcFS */
