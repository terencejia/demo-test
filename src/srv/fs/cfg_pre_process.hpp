#pragma once

#include "fs_ctx.hpp"
#include "fs_spec.hpp"

namespace svcFS {

double delta_discharge_magnitude(
	const discharge_data_t & base_data,
	const discharge_data_t & revise_data,
	const double prs_a1,
	const double prs_a2,
	const bool revise_for_temp,
	const double env_temp,
	const bool revise_for_prs,
	const double env_prs,
	const double logic_magnitude);

void cfg_pre_process(fs_cfg_t & cfg, fs_data_t & new_data, const fs_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(rt_cfg_t & cfg, rt_data_t & new_data, const rt_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(da_cfg_t & cfg, da_data_t & new_data, const da_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(mt_cfg_t & cfg, mt_data_t & new_data, const mt_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(dc_cfg_t & cfg, dc_data_t & new_data, const dc_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(fc_cfg_t & cfg, fc_data_t & new_data, const fc_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(se_cfg_t & cfg, se_data_t & new_data, const se_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(st_cfg_t & cfg, st_data_t & new_data, const st_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);
void cfg_pre_process(rr_cfg_t & cfg, rr_data_t & new_data, const rr_data_t & last_data, fs_spec & spec, double env_temp, double env_prs);

} /* namespace svcFS */
