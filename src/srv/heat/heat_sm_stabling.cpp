#include "heat_sm.hpp"

namespace svcHeat {

stStabling::stStabling(my_context ctx)
: my_base(ctx)
{
	DCL_ZMSG(heat_state) msg;
	msg.sstate = svc_heat_state_t::heat_stabling;
	HEAT_REPORT(msg);

	HEAT_CTX.m_led.set(bi_color_state::red);
	HEAT_CTX.m_heat_tmr.start({{0, 0}, {HEAT_CTX.m_cfg.heat_time, 0}});
}

stStabling::~stStabling()
{
	HEAT_CTX.m_heat_tmr.stop();
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

sc::result stStabling::react(const evCheckTimeout &)
{
	const double temp = HEAT_CTX.m_heater.get_temp();
	log_debug("stStabling: %f to %d", temp, HEAT_CTX.m_cfg.heat_temp);

	switch(HEAT_CFG.Fiberlen) {
	case shrink_tube_t::len_60mm:
		if (temp < HEAT_CTX.m_cfg.heat_temp) {
			HEAT_CTX.m_heater.fast_enable();
		}
		else {
			HEAT_CTX.m_heater.fast_disable();
		}
		break;
	default:
		if (temp < HEAT_CTX.m_cfg.heat_temp) {
			HEAT_CTX.m_heater.slow_enable();
		}
		else {
			HEAT_CTX.m_heater.slow_disable();
		}
		break;
	}

	return discard_event();
}

} /* namespace svcHeat */
