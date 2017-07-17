#pragma once

#include <functional>

#include <boost/units/detail/utility.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <fs_log.hpp>

#include <zmsg/zmsg_packer.hpp>	/// \note only for report
#include <zmsg/zmsg_process_control.hpp>
#include <zmsg/zmsg_svc_state.hpp>

#include "evt_cmm.hpp"

#include "heat_ctx.hpp"

namespace svcHeat {

/**
 * events
 */
struct evCheckTimeout : boost::statechart::event< evCheckTimeout > {};
struct evHeatTimeout : boost::statechart::event< evHeatTimeout > {};

/// cover state change
struct evCoverClose final : sc::event< evCoverClose > {
};
struct evCoverOpen final : sc::event< evCoverOpen > {
};

/**
 * heating state machine
 */
struct stIdle;
struct heat_sm : boost::statechart::state_machine< heat_sm, stIdle > {
public:
	heat_sm(heat_ctx & ctx,
		zmsg::sender & packer,
		std::function<size_t(const void *, size_t)> reporter)
	: m_ctx(ctx)
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
			log_warning("svc heat report: %d fail!", (int)mid);
		}
	}
public:
	heat_ctx & m_ctx;
private:
	zmsg::sender & m_packer;
	std::function<size_t(const void *, size_t)> m_reporter;
};

#define HEAT_CTX context< heat_sm >().m_ctx
#define HEAT_REPORT context< heat_sm >().report
#define HEAT_CFG context< heat_sm >().m_ctx.m_cfg

/*
 * state: idle
 */
struct stRunning;
struct stIdle : sc::state< stIdle, heat_sm > {
	typedef boost::mpl::list<
		  sc::custom_reaction< evMsg<zmsg::mid_t::heat_start> >
		, sc::custom_reaction< evCoverClose >
		, sc::custom_reaction< evCheckTimeout >
	> reactions;

	stIdle(my_context ctx);
	~stIdle();

	sc::result react(const evMsg<zmsg::mid_t::heat_start> &);

	sc::result react(const evCoverClose &);

	sc::result react(const evCheckTimeout &);
};

/*
 * state: running
 */
struct stAscending;
struct stRunning : sc::state< stRunning, heat_sm, stAscending > {
	typedef sc::transition< evMsg<zmsg::mid_t::stop>, stIdle > reactions;
public:
	stRunning(my_context ctx)
	: my_base(ctx)
	{
		HEAT_CTX.m_check_tmr.start();
	}

	~stRunning()
	{
		HEAT_CTX.m_check_tmr.stop();
	}
};

/*
 * state: ascending
 */
struct stStabling;
struct stAscending : sc::state< stAscending, stRunning > {
	typedef boost::mpl::list<
		sc::custom_reaction< evCheckTimeout >
	> reactions;
public:
	stAscending(my_context ctx)
	: my_base(ctx)
	{
		DCL_ZMSG(heat_state) msg;
		msg.sstate = svc_heat_state_t::heat_ascending;
		HEAT_REPORT(msg);

		HEAT_CTX.m_led.set(bi_color_state::green);
		switch(HEAT_CFG.Fiberlen) {
		case shrink_tube_t::len_60mm:
			HEAT_CTX.m_heater.fast_enable();
			break;
		default:
			HEAT_CTX.m_heater.slow_enable();
			break;
		}
	}

	~stAscending()
	{
		switch(HEAT_CFG.Fiberlen) {
		case shrink_tube_t::len_60mm:
			HEAT_CTX.m_heater.fast_disable();
			break;
		default:
			HEAT_CTX.m_heater.slow_disable();
			break;
		}
		HEAT_CTX.m_led.set(bi_color_state::off);
	}

	sc::result react(const evCheckTimeout &)
	{
		double temp = HEAT_CTX.m_heater.get_temp();
		log_debug("ascending: %f to %d", temp, HEAT_CTX.m_cfg.heat_temp);
		if (temp >= HEAT_CTX.m_cfg.heat_temp) {
			return transit< stStabling >();
		}

		return discard_event();
	}
};

/*
 * state: stabling
 */
struct stDescending;
struct stStabling : sc::state< stStabling, stRunning > {
	typedef boost::mpl::list<
		sc::custom_reaction< evCheckTimeout >,
		sc::transition< evHeatTimeout, stDescending >
	> reactions;
public:
	stStabling(my_context ctx);

	~stStabling();

	sc::result react(const evCheckTimeout &);
};

/*
 * state: descending
 */
struct stDescending : sc::state< stDescending, stRunning > {
	typedef boost::mpl::list<
		sc::custom_reaction< evCheckTimeout >
	> reactions;
public:
	stDescending(my_context ctx)
	: my_base(ctx)
	{
		DCL_ZMSG(heat_state) msg;
		msg.sstate = svc_heat_state_t::heat_descending;
		HEAT_REPORT(msg);

		HEAT_CTX.m_led.set(bi_color_state::green);
	}
	~stDescending()
	{
		HEAT_CTX.m_led.set(bi_color_state::off);
	}

	sc::result react(const evCheckTimeout &)
	{
		double temp = HEAT_CTX.m_heater.get_temp();
		log_debug("stDescending: %f to %d", temp, HEAT_CTX.m_cfg.finish_temp);
		if (temp <= HEAT_CTX.m_cfg.finish_temp) {
			DCL_ZMSG(heat_result) msg;
			msg.code = fs_err_t::success;
			HEAT_REPORT(msg);
			return transit< stIdle >();
		}

		return discard_event();
	}
};

}
