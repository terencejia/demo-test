#include "heat_sm.hpp"

namespace svcHeat {

stIdle::stIdle(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(heat_state) msg;
	msg.sstate = svc_heat_state_t::heat_idle;
	HEAT_REPORT(msg);
	if(HEAT_CTX.m_cfg.fast_heat)
	{
		HEAT_CTX.m_check_tmr.start();
	}
}

stIdle::~stIdle()
{
	if(HEAT_CTX.m_cfg.fast_heat)
	{
		HEAT_CTX.m_check_tmr.stop();
	}
}

sc::result stIdle::react(const evMsg<zmsg::mid_t::heat_start> & evt)
{
	HEAT_CTX.m_cfg = evt;

	return transit<stRunning>();
}

sc::result stIdle::react(const evCoverClose &)
{
	log_debug("report heat_ready");
	DCL_ZMSG(heat_state) msg;
	msg.sstate = svc_heat_state_t::heat_ready;
	HEAT_REPORT(msg);

	return discard_event();
}

sc::result stIdle::react(const evCheckTimeout &)
{
	const double temp = HEAT_CTX.m_heater.get_temp();
	log_debug("stIdle temp: %f", temp);
        if(!HEAT_CTX.m_cfg.fast_heat) {
		return discard_event();
	} else {
		switch(HEAT_CFG.Fiberlen) {
		case shrink_tube_t::len_60mm:
			if (temp < HEAT_CTX.m_cfg.stable_temp) {
				HEAT_CTX.m_heater.fast_enable();
			}
			else {
				HEAT_CTX.m_heater.fast_disable();
			}
			break;
		default:
			if (temp < HEAT_CTX.m_cfg.stable_temp) {
				HEAT_CTX.m_heater.slow_enable();
			}
			else {
				HEAT_CTX.m_heater.slow_disable();
			}
			break;
		}
		return discard_event();
	}
	return discard_event();
}

}
