#pragma once

#include <zmsg/zmsg_fusion_splice.hpp>
#include <zmsg/zmsg_discharge_adjust.hpp>
#include <zmsg/zmsg_motor_test.hpp>
#include <zmsg/zmsg_regular_test.hpp>
#include <zmsg/zmsg_dust_check.hpp>
#include <zmsg/zmsg_stabilize_electrode.hpp>
#include <zmsg/zmsg_shrinkage_test.hpp>
#include <zmsg/zmsg_realtime_revise.hpp>

#include "img_process_cmm.hpp"

namespace svcFS {

typedef zmsg::zmsg<zmsg::mid_t::fusion_splice_start> fs_cfg_t;
typedef struct {} fs_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::fusion_splice_result> {
	void init(void)
	{
		this->code = fs_err_t::cover_openned;
		this->defect_data.init();

		/// \todo fiber recognition
		this->rec_info.wrap_diameter = 125000;

		this->z_record_off_set.core_diff_pre = 0;
		this->z_record_off_set.cladding_diff_pre = 0;
		this->z_record_off_set.vertex_intersect_angle = 0;

		this->pattern_compensate = 0.0;
		this->loss_db = 0.0;

		this->z_tense_test_result.is_tense_test = false;
		this->z_tense_test_result.is_success = false;
		this->z_manual_discharge_counts.counts = 0;

		/**
		 * self data init
		 */
		x_left_region.init();
		y_left_region.init();
		x_right_region.init();
		y_right_region.init();
	}

	/// \note data for fiber indentify
	img_process::region_t x_left_region, y_left_region, x_right_region, y_right_region;
} fs_data_t;

typedef zmsg::zmsg<zmsg::mid_t::regular_test_start> rt_cfg_t;
typedef struct {} rt_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::regular_test_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;
	}
} rt_data_t;

typedef zmsg::zmsg<zmsg::mid_t::discharge_adjust_start> da_cfg_t;
typedef struct {
	void init(double hvb_max_volt)
	{
		this->cnt = 0;
		this->continuous_success_cnt = 0;
		this->discharge1_min = {0,0.01};
		this->discharge1_max = {hvb_max_volt,200000};
		this->discharge2_min = {0.6,0.3};  ///\note discharge2_min.x can't be 0
		this->discharge2_max = {hvb_max_volt,1500};

		this->d1_seq.clear();
		this->d2_seq.clear();
	}

	int cnt;
	int continuous_success_cnt;

	mag2shrink_t discharge1_min;
	mag2shrink_t discharge1_max;

	mag2shrink_t discharge2_min;
	mag2shrink_t discharge2_max;

	std::vector< mag2shrink_t > d1_seq;
	std::vector< mag2shrink_t > d2_seq;
} da_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::discharge_adjust_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;

		/// \todo fiber recognition
		this->rec_info.wrap_diameter = 125000;

		this->base = {{ {0,0}, {0,0} }, 0, 0 };
		this->revise = {{ {0,0}, {0,0} }, 0, 0 };

		/**
		 * self data init
		 */
		y_left_abs.init();
		y_right_abs.init();
		x_left_abs.init();
		x_right_abs.init();
	}

	struct img_process::img_abstract_t y_left_abs;
	struct img_process::img_abstract_t y_right_abs;
	struct img_process::img_abstract_t x_left_abs;
	struct img_process::img_abstract_t x_right_abs;
} da_data_t;

typedef zmsg::zmsg<zmsg::mid_t::motor_test_start> mt_cfg_t;
typedef struct {} mt_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::motor_test_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;

		/// \todo fiber recognition
		this->rec_info.wrap_diameter = 125000;

		this->motor_tested_times = 0;
		this->ele_arc_tested_times = 0;

		this->reset = 0;
		this->push = 0;
		this->calibrate = 0;
		this->ele_arc = 0;
		this->img = 0;
	}
} mt_data_t;

typedef zmsg::zmsg<zmsg::mid_t::dust_check_start> dc_cfg_t;
typedef struct {} dc_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::dust_check_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;
	}
} dc_data_t;

typedef zmsg::zmsg<zmsg::mid_t::full_dust_check_start> fc_cfg_t;
typedef struct {} fc_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::full_dust_check_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;
	}
} fc_data_t;

typedef zmsg::zmsg<zmsg::mid_t::stabilize_electrode_start> se_cfg_t;
typedef struct {} se_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::stabilize_electrode_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;
		this->number = 0;
	}
} se_data_t;

typedef zmsg::zmsg<zmsg::mid_t::shrinkage_test_start> st_cfg_t;
typedef struct {} st_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::shrinkage_test_result> {
	void init(void)
	{
		this->code = fs_err_t::failed;
	}
} st_data_t;

typedef zmsg::zmsg<zmsg::mid_t::realtime_revise_start> rr_cfg_t;
typedef struct {} rr_glb_data_t;
typedef struct : public zmsg::zmsg<zmsg::mid_t::realtime_revise_result> {
	void init(void)
	{
		this->code = fs_err_t::cover_openned;
		this->defect_data.init();

		/// \todo fiber recognition
		this->rec_info.wrap_diameter = 125000;

		this->z_record_off_set.core_diff_pre = 0;
		this->z_record_off_set.cladding_diff_pre = 0;
		this->z_record_off_set.vertex_intersect_angle = 0;

		this->pattern_compensate = 0.0;
		this->loss_db = 0.0;

		this->z_tense_test_result.is_tense_test = false;
		this->z_tense_test_result.is_success = false;
		this->z_manual_discharge_counts.counts = 0;
	}
} rr_data_t;
template<typename _cfg_t, typename _data_t, typename _glb_data_t>
struct ctx {
	_cfg_t cfg;
	_data_t data;
	_data_t last_data;
					/// \note only save success data or all
					/// depends on specific service
	_glb_data_t glb_data;
};

class fs_ctx {
public:
	ctx<fs_cfg_t, fs_data_t, fs_glb_data_t> fs;	/// fusion splice
	ctx<rt_cfg_t, rt_data_t, rt_glb_data_t> rt;	/// regular test
	ctx<da_cfg_t, da_data_t, da_glb_data_t> da;	/// discharge adjust
	ctx<mt_cfg_t, mt_data_t, mt_glb_data_t> mt;	/// motor test
	ctx<dc_cfg_t, dc_data_t, dc_glb_data_t> dc;	/// dust check
	ctx<fc_cfg_t, fc_data_t, fc_glb_data_t> fc;	/// full dust check
	ctx<se_cfg_t, se_data_t, se_glb_data_t> se;	/// stabilize electrode
	ctx<st_cfg_t, st_data_t, st_glb_data_t> st;	/// shrinkage test
	ctx<rr_cfg_t, rr_data_t, rr_glb_data_t> rr;    /// realtime revise
};

}
