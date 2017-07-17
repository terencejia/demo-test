#include <fs_log.hpp>

#include <zmsg/zmsg_utils.hpp>

#include "cfg_pre_process.hpp"

namespace svcFS {

static void clamp_hvb_volt(double & dst, const double & max_volt)
{
	if (dst < 0.0) {
		log_warning("hvb strength underflow %f increase to 0.0", dst);
		dst = 0.0;
		return;
	}

	if (dst > max_volt) {
		log_warning("hvb strength overflow %f decreased to %f", dst, max_volt);
		dst = max_volt;
		return;
	}
}

/**
 * \brief calculate the delta value for revising discharge magnitude
 */
double delta_discharge_magnitude(
	const discharge_data_t & base_data,
	const discharge_data_t & revise_data,
	const double prs_a1,
	const double prs_a2,
	const bool revise_for_temp,
	const double env_temp,
	const bool revise_for_prs,
	const double env_prs,
	const double logic_magnitude)
{
	log_debug("logic_mag: %f, env_temp: %f, revise_temp: %f", logic_magnitude, env_temp, revise_data.temp);
	if (base_data.empty()) {
		log_warning("fs_spec: discharge base info is empty");
		return 0;
	}
	if (revise_data.empty()) {
		log_warning("fs_spec: discharge revise info is empty");
		return 0;
	}

	/// \note use double for all caclulation, to avoid minus on uint16_t
	double x = logic_magnitude;

	auto const pb = base_data.p;
	double y = (x - pb[0].x) * ((double)(pb[1].y) - pb[0].y) / ((double)(pb[1].x) - pb[0].x) + pb[0].y;

	auto const pr = revise_data.p;
	double ret = (y - pr[0].y) * ((double)(pr[1].x) - pr[0].x) / ((double)(pr[1].y) - pr[0].y) + pr[0].x;
	log_debug("b0x = %f, b0y = %f, b1x = %f, b1y = %f", pb[0].x, pb[0].y, pb[1].x, pb[1].y);
	log_debug("r0x = %f, r0y = %f, r1x = %f, r1y = %f", pr[0].x, pr[0].y, pr[1].x, pr[1].y);

	ret -= logic_magnitude;

	if (revise_for_temp) {
		log_warning("before temp revise %f", ret);
		ret += (revise_data.temp - env_temp) * 0.002;
		log_warning("after temp revisr %f", ret);
	}

	log_warning("before pressure revise : %f, %d", ret, (int)revise_for_prs);
	if (revise_for_prs) {
		static const double prs_p = 1.013;
		ret += (revise_data.pressure - env_prs) * (prs_a1 + prs_a2 * (prs_p * 2 - revise_data.pressure - env_prs));
		log_warning("env pressure (%f). store pressure (%f), a1(%f), a2(%f)", env_prs, revise_data.pressure, prs_a1, prs_a2);
	}
	log_warning("after pressure revise : %f", ret);

	return ret;
}

void cfg_pre_process(fs_cfg_t & cfg, fs_data_t & new_data, const fs_data_t & last_data, fs_spec & spec, double env_temp, double env_prs)
{
	const double delta_clr = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.CleanDischargeStrength);
	const double delta_pre = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.FiberPreFSStrength);
	const double delta_dis1 = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.Discharge1Strength);
	const double delta_dis2 = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.Discharge2Strength);

	bool realtime_revise = false;
	switch (cfg.FSPattern) {
	case fs_pattern_t::automatic:
	case fs_pattern_t::calibrate:
	case fs_pattern_t::normal:
		if (zmsg::to_val(fiber_t::sm) <= cfg.FiberType
		    && cfg.FiberType < zmsg::to_val(fiber_t::max)) {
			realtime_revise = cfg.RealTimeRevise;
		}
		break;
	default:
		break;
	}
	cfg.RealTimeRevise = realtime_revise;

	double delta_clr_rt = 0;
	double delta_dis1_rt = 0;
	double delta_pre_rt = 0;
	double delta_dis2_rt = 0;
	if (realtime_revise
	    && zmsg::to_val(fiber_t::sm) <= cfg.FiberType
	    && cfg.FiberType <= zmsg::to_val(fiber_t::mm)) {
		double * prt_offset = nullptr;
		auto & rt_rev_info = spec.rt_revise_data[cfg.FiberType];
		switch(cfg.FSPattern) {
		case fs_pattern_t::automatic:
			prt_offset = &rt_rev_info.rt_offset_auto;
			break;
		case fs_pattern_t::calibrate:
		case fs_pattern_t::normal:
			prt_offset = &rt_rev_info.rt_offset_cal;
			break;
		default:
			break;
		}
		if (prt_offset) {
			const double dis1_final = cfg.Discharge1Strength + delta_dis1 + *prt_offset;
			if (dis1_final > spec.hvb_max_volt) {
				*prt_offset -= dis1_final - spec.hvb_max_volt;
			}
			else if (dis1_final < 0) {
				*prt_offset -= dis1_final;
			}
			delta_dis1_rt = *prt_offset;
			delta_clr_rt = delta_dis1_rt * cfg.CleanDischargeStrength / cfg.Discharge1Strength;
			delta_dis2_rt = delta_dis1_rt * cfg.Discharge2Strength / cfg.Discharge1Strength;
			delta_pre_rt = delta_dis1_rt * cfg.FiberPreFSStrength / cfg.Discharge1Strength;
		}
	}

	cfg.CleanDischargeStrength += delta_clr + delta_clr_rt;	clamp_hvb_volt(cfg.CleanDischargeStrength, spec.hvb_max_volt);
	cfg.FiberPreFSStrength += delta_pre + delta_pre_rt;	clamp_hvb_volt(cfg.FiberPreFSStrength, spec.hvb_max_volt);
	cfg.Discharge1Strength += delta_dis1 + delta_dis1_rt;	clamp_hvb_volt(cfg.Discharge1Strength, spec.hvb_max_volt);
	cfg.Discharge2Strength += delta_dis2 + delta_dis2_rt;	clamp_hvb_volt(cfg.Discharge2Strength, spec.hvb_max_volt);
#if 0 /// \todo
	if (last_data.cfg.FSPattern == cfg.FSPattern) {
		switch (cfg.FSPattern) {
		case fs_pattern_t::automatic:
			if (0.04 <= last_data.loss_db
			    && last_data.loss_db <= 0.1
			    && last_data.pattern_compensate < 0.1) {
				new_data.pattern_compensate = last_data.pattern_compensate + 0.02;

				cfg.Discharge1Strength *= (1.0 + new_data.pattern_compensate);
			}
			break;
		case fs_pattern_t::calibrate:
			break;
		default:
			break;
		}
	}
#else
	(void)new_data;
	(void)last_data;
#endif
}

void cfg_pre_process(rt_cfg_t & /*cfg*/, rt_data_t & /*new_data*/, const rt_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(da_cfg_t & /*cfg*/, da_data_t & /*new_data*/, const da_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(mt_cfg_t & /*cfg*/, mt_data_t & /*new_data*/, const mt_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(dc_cfg_t & /*cfg*/, dc_data_t & /*new_data*/, const dc_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(fc_cfg_t & /*cfg*/, fc_data_t & /*new_data*/, const fc_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(se_cfg_t & /*cfg*/, se_data_t & /*new_data*/, const se_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(st_cfg_t & /*cfg*/, st_data_t & /*new_data*/, const st_data_t & /*last_data*/, fs_spec & /*spec*/, double /*env_temp*/, double /*env_prs*/)
{
}

void cfg_pre_process(rr_cfg_t & cfg, rr_data_t & new_data, const rr_data_t & last_data, fs_spec & spec, double env_temp, double env_prs)
{
	const double delta_clr = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.CleanDischargeStrength);
	const double delta_pre = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.FiberPreFSStrength);
	const double delta_dis1 = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.Discharge1Strength);
	const double delta_dis2 = delta_discharge_magnitude(spec.discharge_base, spec.discharge_revise,
		spec.hvb_pressure_compensate_coefficent1,
		spec.hvb_pressure_compensate_coefficent2,
		cfg.Temperature, env_temp,
		cfg.AirPressure, env_prs,
		cfg.Discharge2Strength);

	cfg.CleanDischargeStrength += delta_clr; clamp_hvb_volt(cfg.CleanDischargeStrength, spec.hvb_max_volt);
	cfg.FiberPreFSStrength += delta_pre;	clamp_hvb_volt(cfg.FiberPreFSStrength, spec.hvb_max_volt);
	cfg.Discharge1Strength += delta_dis1;	clamp_hvb_volt(cfg.Discharge1Strength, spec.hvb_max_volt);
	cfg.Discharge2Strength += delta_dis2;	clamp_hvb_volt(cfg.Discharge2Strength, spec.hvb_max_volt);

	(void)new_data;
	(void)last_data;
}

} /* namespace svcFS */
