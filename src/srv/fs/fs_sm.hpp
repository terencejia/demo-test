#pragma once

#include <functional>

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <zmsg/zmsg_packer.hpp>	/// \note only for report
#include <fs_log.hpp>

#include <zmsg/zmsg_fusion_splice.hpp>
#include <zmsg/zmsg_discharge_adjust.hpp>
#include <zmsg/zmsg_regular_test.hpp>
#include <zmsg/zmsg_motor_test.hpp>
#include <zmsg/zmsg_dust_check.hpp>
#include <zmsg/zmsg_stabilize_electrode.hpp>
#include <zmsg/zmsg_realtime_revise.hpp>

#include <zmsg/zmsg_process_control.hpp>
#include <zmsg/zmsg_fusion_splice_reset.hpp>
#include <zmsg/zmsg_shrinkage_test.hpp>

#include <zmsg/zmsg_svc_state.hpp>

#include "img_process.hpp"

#include "has_mem_func.hpp"

#include "evt_cmm.hpp"

#include "fs_cmm.hpp"
#include "fs_ctx.hpp"
#include "cfg_pre_process.hpp"

namespace svcFS {


/**
 * events
 */
template< motorId_t idx>
struct evMotorStop final : sc::event< evMotorStop<idx> > {};

template< motorId_t idx >
struct evMotorBackEnd final : sc::event< evMotorBackEnd<idx> > {};

struct evDischargeTimeout final : sc::event< evDischargeTimeout > {};

struct evMotorTestTimeout final : sc::event< evMotorTestTimeout > {};

struct evMotorPushTimeout final : sc::event< evMotorPushTimeout >{};

/**
 * \note all fs_imgs related events should be added in the helper class for
 * auto deletion
 */

/// \note the evXXXFrame is periodicity image event
/// for record video
struct evRecordVideoFrame final : sc::event< evRecordVideoFrame > {
	evRecordVideoFrame(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	///\todo fs_imgs to gray_img
	const fs_imgs & img;
	const cmosId_t id;
};

/// \note the evXXXReady is disposable image event
/// for common use, wait stable img for img analysis
struct evCheckFiberReady final : sc::event< evCheckFiberReady > {
	evCheckFiberReady(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

/// after move motor, wait stable img for img analysis
struct evMotorLZReady final : sc::event< evMotorLZReady > {
	evMotorLZReady(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

struct evMotorRZReady final : sc::event< evMotorRZReady > {
	evMotorRZReady(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

struct evMotorXReady final : sc::event< evMotorXReady > {
	evMotorXReady(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

struct evMotorYReady final : sc::event< evMotorYReady > {
	evMotorYReady(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

/// after discharge, wait stable img for img analysis
struct evHvbReady final : sc::event< evHvbReady > {
	evHvbReady(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

/// \note the evXXXInfo is disposable image event
struct evWaveFormInfo final : sc::event< evWaveFormInfo > {
	evWaveFormInfo(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

struct evFiberDefectInfo final : sc::event< evFiberDefectInfo > {
	evFiberDefectInfo(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

/// \note the evFocusImg is disposable image event
template< cmosId_t cmosid >
struct evFocusImg final : sc::event< evFocusImg<cmosid> > {
	evFocusImg(const fs_imgs & i, cmosId_t id)
	: img(i)
	, id(id)
	{
	}

	const fs_imgs & img;
	cmosId_t id;
};

///\note RealtimeRevise wait stable img for img analysis
struct evRealtimeRevise final : sc::event< evRealtimeRevise > {
	evRealtimeRevise(const fs_imgs & i, const cmosId_t cmosid)
	: img(i)
	, id(cmosid)
	{
	}

	const fs_imgs & img;
	const cmosId_t id;
};

/// cover state change
struct evCoverClose final : sc::event< evCoverClose > {
};
struct evCoverOpen final : sc::event< evCoverOpen > {
};
/// cone-fusion-splicing
struct evStartCone final : sc::event< evStartCone > {
};

/**
 * exception translator
 */
struct evImgProcessError final : sc::event< evImgProcessError > {};
struct evSystemError final : sc::event< evSystemError > {};

class fs_exception_translator
{
public:
	template< class Action, class ExceptionEventHandler >
	sc::result operator()(Action action,
			      ExceptionEventHandler eventHandler)
	{
		try {
			return action();
		}
		catch (const img_process::img_process_error & e) {
			log_err("exception : %s", e.what());
			img_process::save_pgm("ipe.pgm", e.m_img);
			return eventHandler(evImgProcessError());
		}
		catch (const std::exception & e) {
			log_err("exception : %s", e.what());
			return eventHandler(evSystemError());
		}
		catch (...) {
			log_err("exception : unkown error");
			return eventHandler(evSystemError());
		}
	}
};

/**
 * fusion splicing state machine
 */
struct stReset;	// initial state
struct fs_sm : sc::state_machine< fs_sm, stReset, std::allocator<void>, fs_exception_translator > {
public:
	fs_sm(const hw_info & hwinfo,
		fs_spec & spec,
		fs_devs & devs,
		fs_auxs & auxs,
		fs_ctx & ctx,
		zmsg::sender & packer,
		std::function<size_t(const void *, size_t)> reporter)
	: m_hwinfo(hwinfo)
	, m_spec(spec)
	, m_devs(devs)
	, m_auxs(auxs)
	, m_ctx(ctx)
	, m_packer(packer)
	, m_reporter(reporter)
	{
	}
public:
	template< zmsg::mid_t mid >
	void report(const zmsg::zmsg<mid> & msg)
	{
		try {
			m_packer.fill_to<false, true>(msg, m_reporter);
		}
		catch(...) {
			log_warning("svc fs report: %d fail!", (int)mid);
		}
	}

	template< typename _img_evt_t >
	void active_waiter_video(cmosId_t cmosid = cmosId_t::X)
	{
		m_devs.m_camera.push_waiter_video(__form_img_waiter<_img_evt_t>(cmosid));
	}

	template< typename _img_evt_t >
	void deactive_waiter_video(cmosId_t cmosid = cmosId_t::X)
	{
		m_devs.m_camera.pop_waiter_video(__form_img_waiter<_img_evt_t>(cmosid));
	}

	template< typename _img_evt_t>
	void active_waiter(cmosId_t cmosid = cmosId_t::Y, int __delay_ms_t = 0)
	{
		m_devs.m_camera.push_waiter(__form_img_waiter<_img_evt_t>(cmosid), __delay_ms_t, cmosid);
	}

	template< typename _img_evt_t>
	void active_delay_waiter(cmosId_t cmosid = cmosId_t::Y)
	{
		m_devs.m_camera.push_waiter(__form_img_waiter<_img_evt_t>(cmosid), m_spec.img_cap_delay, cmosid);
	}

	template< typename _img_evt_t>
	void deactive_waiter(cmosId_t cmosid = cmosId_t::XY)
	{
		if (cmosid == cmosId_t::XY) {
			m_devs.m_camera.pop_waiter(__form_img_waiter<_img_evt_t>(cmosId_t::X), cmosId_t::X);
			m_devs.m_camera.pop_waiter(__form_img_waiter<_img_evt_t>(cmosId_t::Y), cmosId_t::Y);
		} else {
			m_devs.m_camera.pop_waiter(__form_img_waiter<_img_evt_t>(cmosid), cmosid);
		}
	}

private:
	template< typename _img_evt_t>
	std::function<void (const fs_imgs & img)> __form_img_waiter(cmosId_t id)
	{
		return [this, id](const fs_imgs & img) {
			this->process_event(_img_evt_t(img, id));
		};
	}
public:
	const hw_info & m_hwinfo;
	fs_spec & m_spec;
	fs_devs & m_devs;
	fs_auxs & m_auxs;
	fs_ctx & m_ctx;	/// used by events.
private:
	zmsg::sender & m_packer;
	std::function<size_t(const void *, size_t)> m_reporter;
};



/**
 * state: wait reset
 * \note if we receive 'cover close' when waiting reset, we will also reset.
 */
struct stWaitReset : sc::state< stWaitReset, fs_sm > {
	typedef boost::mpl::list<
		  sc::custom_reaction< evMsg<zmsg::mid_t::fusion_splice_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::discharge_adjust_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::motor_test_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::dust_check_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::stabilize_electrode_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::regular_test_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::realtime_revise_start> >

		, sc::custom_reaction< evCoverOpen >

		, sc::transition< evMsg<zmsg::mid_t::fusion_splice_reset>, stReset >
		, sc::transition< evCoverClose, stReset >
	> reactions;

	stWaitReset(my_context ctx);
	~stWaitReset();

	sc::result react(const evMsg<zmsg::mid_t::fusion_splice_start> &);
	sc::result react(const evMsg<zmsg::mid_t::discharge_adjust_start> &);
	sc::result react(const evMsg<zmsg::mid_t::motor_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::dust_check_start> &);
	sc::result react(const evMsg<zmsg::mid_t::stabilize_electrode_start> &);
	sc::result react(const evMsg<zmsg::mid_t::regular_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::realtime_revise_start> &);

	sc::result react(const evCoverOpen &);
};


#define FS_STATE_NAME	typeid(*this).name()

#define FS_SM		context< fs_sm >()

#define FS_SPEC		context< fs_sm >().m_spec

#define FS_HWINFO	context< fs_sm >().m_hwinfo

#define FS_IPP		FS_SPEC.ipp

#define FS_DEV		context< fs_sm >().m_devs
#define FS_AUX		context< fs_sm >().m_auxs
#define FS_REPORT	context< fs_sm >().report

/// the cfg after preprocess
#define FS_CFG		context< stRunning >().m_cfg
/// the data this time
#define FS_DAT		context< stRunning >().m_data
/// the cfg before preprocess
#define FS_ORI_CFG	FS_DAT.z_cfg
/// the data last time
#define FS_LAST_DAT	context< stRunning >().m_last_data
/// global data
#define FS_GLB_DAT	context< stRunning >().m_glb_data

/**
 * state: reset
 */
struct stResetBackward;
struct stReset : sc::state< stReset, fs_sm, stResetBackward > {
	typedef boost::mpl::list<
		  sc::custom_reaction< evMsg<zmsg::mid_t::fusion_splice_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::discharge_adjust_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::motor_test_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::dust_check_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::full_dust_check_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::stabilize_electrode_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::regular_test_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::realtime_revise_start> >

		, sc::custom_reaction< evMsg<zmsg::mid_t::stop> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::fusion_splice_reset> >
		, sc::custom_reaction< evCoverClose >
	> reactions;

	stReset(my_context ctx);
	~stReset();

	sc::result react(const evMsg<zmsg::mid_t::fusion_splice_start> &);
	sc::result react(const evMsg<zmsg::mid_t::discharge_adjust_start> &);
	sc::result react(const evMsg<zmsg::mid_t::motor_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::dust_check_start> &);
	sc::result react(const evMsg<zmsg::mid_t::full_dust_check_start> &);
	sc::result react(const evMsg<zmsg::mid_t::stabilize_electrode_start> &);
	sc::result react(const evMsg<zmsg::mid_t::regular_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::realtime_revise_start> &);

	sc::result react(const evMsg<zmsg::mid_t::stop> &);
	sc::result react(const evMsg<zmsg::mid_t::fusion_splice_reset> &);
	sc::result react(const evCoverClose &);
private:
	void clear_deferral(void)
	{
		m_deferral_processor = nullptr;
	}
	template< typename _event_t >
	void set_deferral(const _event_t & evt)
	{
		m_deferral_processor = [evt, this](void) -> void
		{
			return this->post_event(evt);
		};
	}
private:
	std::function<void (void)> m_deferral_processor;
};

/**
 * state: reset backward
 */
struct stResetBackward : sc::state< stResetBackward, stReset > {
	struct __evEntryAct : sc::event< __evEntryAct > {};

	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< evMotorStop<motorId_t::LZ> >,
		sc::custom_reaction< evMotorStop<motorId_t::RZ> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::LZ> >,
		sc::custom_reaction< evMotorBackEnd<motorId_t::RZ> >
	> reactions;

	stResetBackward(my_context ctx);
	~stResetBackward();

	sc::result react(const __evEntryAct &);
	sc::result react(const evMotorStop<motorId_t::LZ> &);
	sc::result react(const evMotorStop<motorId_t::RZ> &);
	sc::result react(const evMotorBackEnd<motorId_t::LZ> &);
	sc::result react(const evMotorBackEnd<motorId_t::RZ> &);
private:
	bool __handle_lz(void);
	bool __handle_rz(void);
	sc::result __transit(void);
};

/**
 * state: idle
 */
struct stIdle : sc::state< stIdle, fs_sm > {
	typedef boost::mpl::list<
		sc::custom_reaction< evMsg<zmsg::mid_t::fusion_splice_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::discharge_adjust_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::regular_test_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::motor_test_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::dust_check_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::full_dust_check_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::stabilize_electrode_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::shrinkage_test_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::realtime_revise_start> >,
		sc::custom_reaction< evCoverClose >
	> reactions;

	stIdle(my_context ctx);
	~stIdle();

	sc::result react(const evMsg<zmsg::mid_t::fusion_splice_start> &);
	sc::result react(const evMsg<zmsg::mid_t::discharge_adjust_start> &);
	sc::result react(const evMsg<zmsg::mid_t::regular_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::motor_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::dust_check_start> &);
	sc::result react(const evMsg<zmsg::mid_t::full_dust_check_start> &);
	sc::result react(const evMsg<zmsg::mid_t::stabilize_electrode_start> &);
	sc::result react(const evMsg<zmsg::mid_t::shrinkage_test_start> &);
	sc::result react(const evMsg<zmsg::mid_t::realtime_revise_start> &);

	sc::result react(const evCoverClose &);
private:
	template< typename _next_state_t, typename _cfg_msg_t, typename _ctx_t>
	sc::result __transit(
			const _cfg_msg_t & cfg,
			_ctx_t & svc_ctx,
			const char * info)
	{
		bool cover_openned = FS_DEV.m_cover.get_state();
		if (cover_openned) {
			log_warning("start svc [%s] need cover closed!!!", info);
                        svc_ctx.data.code = decltype(svc_ctx.data.code)::cover_openned;
			FS_REPORT(svc_ctx.data);
			return discard_event();
		}
		else {
			return __transit_force<_next_state_t>(cfg, svc_ctx, info);
		}
	}

	template< typename _next_state_t, typename _cfg_msg_t, typename _ctx_t>
	sc::result __transit_force(
			const _cfg_msg_t & cfg,
			_ctx_t & svc_ctx,
			const char * /*info*/)
	{
		svc_ctx.cfg = cfg;
		svc_ctx.data.init();
		svc_ctx.data.z_cfg = cfg;
		cfg_pre_process(svc_ctx.cfg, svc_ctx.data,
				svc_ctx.last_data, FS_SPEC,
				FS_DEV.m_env_temp.read(),
				FS_DEV.m_env_pressure.read());
		FS_DEV.m_hvb.reset_count();
		return transit< _next_state_t >();
	}
};

/**
 * \brief helper class
 * \note it only care disposable image events.
 */
template< typename _state_t >
struct helper {
		//static fs_sm sm;
	~helper(void)
	{
		fs_sm & sm = static_cast<_state_t *>(this)->template context<fs_sm>();
		this->template __clr<
			  evRecordVideoFrame
			, evCheckFiberReady
			, evMotorLZReady
			, evMotorXReady
			, evMotorRZReady
			, evMotorYReady
			, evHvbReady
			, evWaveFormInfo
			, evFiberDefectInfo
			, evFocusImg<cmosId_t::X>
			, evFocusImg<cmosId_t::Y>
			, evRealtimeRevise
		>(sm);
	}
private:
	template<typename _evt_t>
	void __clr_single(fs_sm & sm)
	{
		if (has_mem_func<_state_t, sc::result(const _evt_t &)>::value) {
			sm.deactive_waiter<_evt_t>();
		}
	}

	template< typename ... _evt_t >
	void __clr(fs_sm & sm)
	{
		auto res = {(
			__clr_single<_evt_t>(sm),
		0)...};
		(void)res;
	}
};

}
