#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {

stDischarge1::stDischarge1(my_context ctx)
: my_base(ctx)
, m_left_pos(0)
, m_right_pos(0)
{
	log_debug("shrinkage test");
	FS_SM.active_delay_waiter<__evImgReady>();
}

stDischarge1::~stDischarge1()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stDischarge1::react(const __evImgReady & evt)
{
	m_left_pos = img_process::left_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	m_right_pos = img_process::right_vertex_fine_pos(evt.img.yz_img, FS_IPP);

	FS_AUX.m_discharge_tmr.start({{0, 0}, exemodel::ms_to_timespec(FS_CFG.Discharge1Time)});

	FS_DEV.m_hvb.start(FS_CFG.Discharge1Strength);

	return discard_event();
}

sc::result stDischarge1::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();
	FS_SM.active_delay_waiter<evHvbReady>();
	return discard_event();
}

sc::result stDischarge1::react(const evHvbReady & evt)
{

	const double y_left_pos = img_process::left_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	const double y_right_pos = img_process::right_vertex_fine_pos(evt.img.yz_img, FS_IPP);
	const double x_left_pos = img_process::left_vertex_fine_pos(evt.img.xz_img, FS_IPP);
	const double x_right_pos = img_process::right_vertex_fine_pos(evt.img.xz_img, FS_IPP);

	const int y_col_off = int(y_left_pos + y_right_pos) / 2 - evt.img.yz_img.width / 2;
	const int x_col_off = int(x_left_pos + x_right_pos) / 2 - evt.img.xz_img.width / 2;


	const double init_gap = FS_SPEC.yz_pixel_to_nm(m_right_pos - m_left_pos) / 1000.0;
	const double shrinkage = FS_SPEC.yz_pixel_to_nm(
		(y_right_pos - y_left_pos) - (m_right_pos - m_left_pos)) / 1000.0;

	if (std::abs(y_col_off) > 10 || std::abs(x_col_off) > 10) {
		log_warning("fibers' gap center is offed window's center too much!");
		FS_DAT.code = fs_err_t::fiber_off_center;

		return transit<stWaitReset>();
	}

	/// adjust y window
	if (y_col_off != 0) {
		FS_SPEC.window_y_col += y_col_off;
		FS_DEV.m_camera.move_window_pos_y(y_col_off, 0);

		DCL_ZMSG(update_window_position) msg;
		msg.is_pos_x = false;
		msg.row = FS_SPEC.window_y_row;
		msg.column = FS_SPEC.window_y_col;
		FS_REPORT(msg);
	}

	/// adjust x window
	if (x_col_off != 0) {
		FS_SPEC.window_x_col += x_col_off;
		FS_DEV.m_camera.move_window_pos_x(x_col_off, 0);

		DCL_ZMSG(update_window_position) msg;
		msg.is_pos_x = true;
		msg.row = FS_SPEC.window_x_row;
		msg.column = FS_SPEC.window_x_col;
		FS_REPORT(msg);
	}

	FS_DAT.real_gap = init_gap;
	FS_DAT.shrinkage = shrinkage;
	return transit<stWaitReset>();
}

}

}
