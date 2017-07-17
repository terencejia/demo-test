#include <fs_log.hpp>

#include "fs_debug.hpp"

#include "fs_spec.hpp"


namespace svcFS {

bool g_log_debug = false;
bool debug_precise_calibrating = false;
bool g_fs_state_time = false;
//double g_log_base = 2;

double fs_spec::z_speed_from_pixel(double pixel)
{
	const double f = std::abs(pixel * 1380 / hwinfo.yz_nm_per_pixel) / zmotor_speed_factor;
	if (f >= 1.0) {
		return zmotor_speed_max / hwinfo.push_motor_speed_ratio;
	}

	return std::pow(f, zmotor_speed_pow) * zmotor_speed_max / hwinfo.push_motor_speed_ratio;
}

double fs_spec::xy_speed_from_pixel(double pixel)
{
	const double f = std::abs(pixel) / xymotor_speed_factor;
	if (f >= 1.0) {
		return xymotor_speed_max;
	}

	return std::pow(f, xymotor_speed_pow) * xymotor_speed_max;
}

bool fs_spec::is_z_dist_clr_ok(double dist) const
{
	return (-zdist_clr_tolerance <= dist && dist <= zdist_clr_tolerance);
}

fs_spec::fs_spec(const hw_info & hwinfo)
: zmsg::zmsg()
, hvb_max_volt(2.3)
, hvb_pressure_compensate_coefficent1(0.6940232)
, hvb_pressure_compensate_coefficent2(7.046065)
, hwinfo(hwinfo)
{
	this->window_x_row = 0;
	this->window_x_col = 0;
	this->window_y_row = 0;
	this->window_y_col = 0;
	this->zmotor_spec_nm_per_step = 62.5;
	this->lz_nm_per_step = 62.5;
	this->rz_nm_per_step = 62.5;
	this->zmotor_spec_nm_per_step *= hwinfo.push_motor_speed_ratio;
	this->lz_nm_per_step *= hwinfo.push_motor_speed_ratio;
	this->rz_nm_per_step *= hwinfo.push_motor_speed_ratio;

	this->img_cap_delay = 200;
	this->clr_discharge_gap = 200000;
	this->check_fiber_exist_time = 5000;
	for (auto & i : this->motor_max_speed) {
		i = 0x0180;
	}
	for (auto & i : this->motor_min_speed) {
		i = 0xFFFF;
	}
	this->tensionSpeed = 0x0180;
	this->motor_xy_steps_per_pixel = 80;
	this->fiber_outline_blacklevel = 0.5;
	this->calibrating_xy_dist_threshold = 5;
	this->precise_calibrating_xy_dist_threshold = 0.1;
	this->z_dist_threshold = 1;
	this->discharge_base = {{ { 254, 8000 }, { 434, 55000 } }, 25.0, 1.013 };
	this->discharge_revise.clear();
	this->dust_check_threshold0 = 8;
	this->dust_check_threshold1 = 24;
	this->img_denoise_threshold = 1;
	for (auto & i : this->led_brightness) {
		i = 0.5;
	}
	this->x_focal_distance = 0.5;
	this->y_focal_distance = 0.5;

	this->entering_speed = 1.0;
	this->push1_speed = 0.25;
	this->push2_stage1_speed = 0.25;
	this->push2_stage2_speed = 0.25;
	this->manual_calibrate_speed = 0.05;
	this->motor_xy_precise_calibrate_speed = 0.3;

	this->xz_nm_per_pixel = 1250;
	this->yz_nm_per_pixel = 1250;

	this->ipp.bg = 127;

	this->x_spliter = 0;
	this->y_spliter = 0;

	this->cover_delay_time = 1000;
	this->x_motor_retrace_steps = 1;
	this->y_motor_retrace_steps = 1;
	this->push2_stage2_dist_coe = 3;


	this->zmotor_speed_factor = 50;
	this->zmotor_speed_pow = 1.2;
	this->zmotor_speed_max = 0.8;

	this->xymotor_speed_factor = 30;
	this->xymotor_speed_pow = 1.2;
	this->xymotor_speed_max = 1.0;

	this->zmotor_forward_distance = 256 * 1000;	/// 256um
	this->zmotor_stroke = 2 * 1000 * 1000;		/// 2mm

	this->dbg_hangle_limit = 0.1;
	this->dbg_vangle_limit = 90;

	this->pre_cal_xy_dist_threshold_relax_ratio = 4;

	this->moter_push_timeout = 8000;

	for (auto & i : this->rt_revise_data) {
		i.clear();
	}

	this->loss_factor = 1.0;
}

fs_spec::~fs_spec()
{
}

void fs_spec::reinit(const spec_t & spec, const hw_info & hwinfo)
{
	spec_t & base_data = *this;

	/// base data
	base_data = spec;

	/// fix base data
	if (this->img_denoise_threshold == 0) {
		this->img_denoise_threshold = 1;
	}

	/// reinit self data
	if(hwinfo.cmos_type == "ov9121")
		this->img_cap_delay = static_cast<uint16_t>(spec.img_cap_delay + 100);	
	log_debug("cmos:%s,img_cap_delay = %d",hwinfo.cmos_type.c_str(),this->img_cap_delay);

	this->zmotor_spec_nm_per_step *= hwinfo.push_motor_speed_ratio;
	this->lz_nm_per_step *= hwinfo.push_motor_speed_ratio;
	this->rz_nm_per_step *= hwinfo.push_motor_speed_ratio;

	this->ipp.bg = (this->fiber_outline_blacklevel + 0.05);

	this->x_spliter = 0;
	this->y_spliter = 0;

	this->xz_nm_per_pixel = hwinfo.xz_nm_per_pixel;
	this->yz_nm_per_pixel = hwinfo.yz_nm_per_pixel;
	this->zdist_clr_tolerance = hwinfo.zdist_clr_tolerance;

	if (hwinfo.hvb_alti == 3000) {
		this->hvb_max_volt = 2.3;
		this->hvb_pressure_compensate_coefficent1 = 0.6940232;
		this->hvb_pressure_compensate_coefficent2 = 7.046065;
	}
	else if (hwinfo.hvb_alti == 5000) {
		this->hvb_max_volt = 2.7;
		this->hvb_pressure_compensate_coefficent1 = 1.073992;
		this->hvb_pressure_compensate_coefficent2 = 2.92936;
	}
	else {
		this->hvb_max_volt = 2.3;
		this->hvb_pressure_compensate_coefficent1 = 0.6940232;
		this->hvb_pressure_compensate_coefficent2 = 7.046065;
	}
}

void fs_spec::update_yz_nm_per_pixel(double left_npp, double right_npp)
{
	double & nm_per_pixel = this->yz_nm_per_pixel;

	if (left_npp > 0
	    && right_npp > 0
	    && std::abs(left_npp / right_npp - 1.0) < 0.2
	    && std::abs(left_npp / nm_per_pixel - 1.0) < 0.2
	    && std::abs(right_npp / nm_per_pixel - 1.0) < 0.2) {
		nm_per_pixel = (left_npp + right_npp) / 2;
	}
	else {
		log_warning("update_yz_nm_per_pixel wrong base(%f) left(%f) right(%f)", nm_per_pixel, left_npp, right_npp);
	}
}

void fs_spec::update_xz_nm_per_pixel(double left_npp, double right_npp)
{
	double & nm_per_pixel = this->xz_nm_per_pixel;

	if (left_npp > 0
	    && right_npp > 0
	    && std::abs(left_npp / right_npp - 1.0) < 0.2
	    && std::abs(left_npp / nm_per_pixel - 1.0) < 0.2
	    && std::abs(right_npp / nm_per_pixel - 1.0) < 0.2) {
		nm_per_pixel = (left_npp + right_npp) / 2;
	}
	else {
		log_warning("update_xz_nm_per_pixel wrong base(%f) left(%f) right(%f)", nm_per_pixel, left_npp, right_npp);
	}
}

void fs_spec::update_motor_push_timeout(const uint16_t time)
{
	this->moter_push_timeout = time;
}

/**
 * data {current positon, steps will move}
 */
void fs_spec::update_motor_retrace_steps(bool is_x, const std::vector< std::tuple<double, int32_t>> & data)
{
	if (data.size() < 2)
		return;

	auto & motor_retrace_steps = (is_x ? x_motor_retrace_steps : y_motor_retrace_steps);
	int32_t new_retrace_steps = 0;

	for(uint32_t i = 1; i < data.size(); i++)
	{
		/// TODO: maybe we should compare with the init positon, to calculate ///\param fiber_moved_pixel
		const double fiber_moved_pixel = std::abs(std::get<0>(data[i]) - std::get<0>(data[i-1]));
		const int32_t motor_moved_steps = std::abs(std::get<1>(data[i-1]));
		new_retrace_steps += motor_moved_steps;

		if (is_fiber_moved(fiber_moved_pixel)) {
			new_retrace_steps -= std::abs(pixel_to_xy_step(fiber_moved_pixel));
			break;
		}
	}

	///\note assume the min retrace step is 1
	if (new_retrace_steps < 1)
		new_retrace_steps = 1;

	log_debug("update %s motor retrace steps %d ====>> %d", (is_x ? "x" : "y"), motor_retrace_steps, new_retrace_steps);
	motor_retrace_steps = new_retrace_steps;

}

} /* namespace svcFS */
