#include <fs_log.hpp>

#include "math_ext.hpp"

#include "fs_spec.hpp"
#include "img_process.hpp"
#include "sm_discharge_adjust.hpp"

#include "fs_debug.hpp"

namespace svcFS {

namespace smDischargeAdjust {

stDischarge1::stDischarge1(my_context ctx)
: my_base(ctx)
, m_left_pos(0)
, m_right_pos(0)
, m_x_ready(false)
, m_y_ready(false)
{
	log_debug("da: discharge1...");
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
}

stDischarge1::~stDischarge1()
{
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stDischarge1::react(const __evImgReady & evt)
{
	m_left_pos = img_process::left_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	m_right_pos = img_process::right_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.FiberPreFSTime)});

	FS_DEV.m_hvb.start(FS_CFG.FiberPreFSStrength);

	const int tmp = static_cast<int>(0.5 * FS_CFG.FiberPreFSTime);
	FS_SM.active_waiter<__evImgArcSteady>(cmosId_t::Y, tmp);
	FS_SM.active_waiter<__evImgArcSteady>(cmosId_t::X, tmp);

	return discard_event();
}

sc::result stDischarge1::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();
	FS_SM.active_delay_waiter<evHvbReady>(cmosId_t::X);
	FS_SM.active_delay_waiter<evHvbReady>(cmosId_t::Y);
	return discard_event();
}
sc::result stDischarge1::react(const evHvbReady & evt)
{
	img_process::save_pgm("/tmp/da1y.pgm",evt.img.yz_img);
	const double y_left_pos = img_process::left_vertex_fine_pos(evt.img.yz_img, FS_DAT.y_left_abs);
	const double y_right_pos = img_process::right_vertex_fine_pos(evt.img.yz_img, FS_DAT.y_right_abs);

	const double error_max = 6.0;
	const double error_min = -3.0;

	const double unrevise_strength = FS_CFG.FiberPreFSStrength;
	/// \TODO we should has independent wrap_diameter for left and right fiber
	const double real_shrink_um = (FS_SPEC.yz_pixel_to_nm((y_right_pos - y_left_pos) - (m_right_pos - m_left_pos))
					+ FS_DAT.rec_info.wrap_diameter * std::tan(deg2rad(FS_DAT.defect_data.left_cut_angle())) / 2
					+ FS_DAT.rec_info.wrap_diameter * std::tan(deg2rad(FS_DAT.defect_data.right_cut_angle())) / 2
					)/ 1000.0;
	/// check if valid
	const double base_shrink_um = (FS_SPEC.discharge_base.empty() ?
				8.0 : FS_SPEC.discharge_base.p[0].y);

	const double adjust1_error = real_shrink_um - base_shrink_um;
	log_warning("da-1 unrevise_strength: %f, real_shrink_um: %f, base_shrink_um: %f", unrevise_strength, real_shrink_um, base_shrink_um);
	if (unrevise_strength < FS_GLB_DAT.discharge1_min.x  //when the user changes unrevise_strength manually
		|| (FS_GLB_DAT.discharge1_min.x < unrevise_strength && unrevise_strength < FS_GLB_DAT.discharge1_max.x && real_shrink_um < (base_shrink_um + error_min))) {
		FS_GLB_DAT.discharge1_min = { unrevise_strength, real_shrink_um };
	}
	if (unrevise_strength > FS_GLB_DAT.discharge1_max.x //when the user changes unrevise_strength manually
		|| (FS_GLB_DAT.discharge1_max.x > unrevise_strength && unrevise_strength > FS_GLB_DAT.discharge1_min.x && real_shrink_um > (base_shrink_um + error_max))) {
		FS_GLB_DAT.discharge1_max = { unrevise_strength, real_shrink_um };
	}
	log_warning("da-1 max.strength: %f, max.shrink: %f, min.strength: %f, min.shrink: %f",
		FS_GLB_DAT.discharge1_max.x,
		FS_GLB_DAT.discharge1_max.y,
		FS_GLB_DAT.discharge1_min.x,
		FS_GLB_DAT.discharge1_min.y);
	if (adjust1_error > error_max || adjust1_error < error_min) {
		FS_DAT.code = fs_err_t::revise1_mag;
		FS_DAT.base = FS_SPEC.discharge_base;
		FS_DAT.suggest1 = __autoAdjust_dischargeStrength(base_shrink_um);
		FS_DAT.revise.p[0].x = unrevise_strength;
		FS_DAT.revise.p[0].y = real_shrink_um;
		log_warning("da1 mag %.3f shrink %.3f suggest %.3f", unrevise_strength, real_shrink_um, FS_DAT.suggest1);

		return transit<stWaitReset>();
	}

	double trimming = 0.0;
	if (adjust1_error <= -1.0 && adjust1_error >= -3.0) {
		trimming = 0.01;
	}

	if (adjust1_error <= 6.0 && adjust1_error >= 2.0) {
		trimming = -0.01;
	}

	FS_GLB_DAT.d1_seq.push_back({ unrevise_strength, real_shrink_um, });
	FS_DAT.suggest1 = unrevise_strength + trimming;
	FS_DAT.revise.p[0].x = unrevise_strength;
	FS_DAT.revise.p[0].y = real_shrink_um;
	log_warning("da1 mag %.3f shrink %.3f suggest %.3f", unrevise_strength, real_shrink_um, FS_DAT.suggest1);

	return transit<stDischarge2>();
}

sc::result stDischarge1::react(const __evImgArcSteady & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			{
				m_xz_img = evt.img.xz_img;
				m_x_ready = true;
			}
			break;
		case cmosId_t::Y:
			{
				m_yz_img = evt.img.yz_img;
				m_y_ready = true;
			}
			break;
		default:
			break;
	}

	if (m_x_ready && m_y_ready)
	{
		const int middle_of_arc_y = img_process::get_arc_horizental_center(evt.img.yz_img, FS_DAT.y_left_abs,  FS_DAT.y_right_abs);
		const int col_off = middle_of_arc_y - evt.img.yz_img.width / 2;
		log_debug("middle_of_arc_y %d col_off %d", middle_of_arc_y, col_off);

		/// adjust cmos window horizontal coordinate by arc center, x follow y
		if (col_off != 0) {
			///\ Y window
			{
				///\WARNNING move_window_pos_* maybe throw exception
				FS_DEV.m_camera.move_window_pos_y(col_off, 0);
				FS_SPEC.window_y_col += col_off;

				DCL_ZMSG(update_window_position) msg;
				msg.is_pos_x = false;
				msg.row = FS_SPEC.window_y_row;
				msg.column = FS_SPEC.window_y_col;
				FS_REPORT(msg);
			}

			///\ x window
			{
				FS_DEV.m_camera.move_window_pos_x(col_off, 0);
				FS_SPEC.window_x_col += col_off;

				DCL_ZMSG(update_window_position) msg;
				msg.is_pos_x = true;
				msg.row = FS_SPEC.window_x_row;
				msg.column = FS_SPEC.window_x_col;
				FS_REPORT(msg);
			}
		}

		/// check if the center of the two fiber-ends is at the right position
		if (std::abs(col_off) > 20) {
			log_warning("arc center is offed window's center too much!");
		}
	}

	return discard_event();
}
double stDischarge1::__autoAdjust_dischargeStrength(const double base_shrink_um)
{
	double revise_strength = 0.0;
	if( FS_GLB_DAT.discharge1_min.y <= 0 ) {
		FS_GLB_DAT.discharge1_min.y = 1;
	}
	const auto x1 = FS_GLB_DAT.discharge1_max.x;
	const auto x2 = FS_GLB_DAT.discharge1_min.x;
	const auto ln_y1 = std::log(FS_GLB_DAT.discharge1_max.y);
	const auto ln_y2 = std::log(FS_GLB_DAT.discharge1_min.y);
	const auto ln_y3 = std::log(base_shrink_um);
	//const auto x3 = ?;
	revise_strength = (ln_y3 * (x1 - x2) - x1 * ln_y2 + x2 * ln_y1) / (ln_y1 - ln_y2);

	if (revise_strength < 0.01) {
		revise_strength = 0.0;
		log_warning("hvb exception,the discharge capacity is too strong %f", revise_strength);
	}
	if (revise_strength > FS_SPEC.hvb_max_volt) {
		revise_strength = FS_SPEC.hvb_max_volt;
		log_warning("hvb exception, the discharge capacity is too weak %f (%f)", revise_strength, FS_SPEC.hvb_max_volt);
	}

	return revise_strength;
}

}

}
