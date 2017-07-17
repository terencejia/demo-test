#include <zmsg/zmsg_process_control.hpp>

#include <fs_log.hpp>

#include "fsconf.h"

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_full_dust_check.hpp"

static constexpr double img_cap_ratio = 0.80;

namespace svcFS {

namespace smFullDustCheck {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.fc.cfg)
, m_data(context<fs_sm>().m_ctx.fc.data)
, xz_cur_x(0)
, xz_cur_y(0)
, yz_cur_x(0)
, yz_cur_y(0)
, xz_init_x(0)
, xz_init_y(0)
, yz_init_x(0)
, yz_init_y(0)
, m_x_continue(true)
, m_y_continue(true)
{
	FS_DEV.enable();
	log_debug("fc: dust check...");

	FS_DEV.m_camera.get_window_pos_x(xz_init_x, xz_init_y);
	FS_DEV.m_camera.get_window_pos_y(yz_init_x, yz_init_y);

	FS_DEV.m_camera.set_window_pos_x(xz_cur_x, xz_cur_y);
	FS_DEV.m_camera.set_window_pos_y(yz_cur_x, yz_cur_y);
}

stRunning::~stRunning()
{
	const DCL_ZMSG(full_dust_check_result) & re = FS_DAT;
	FS_REPORT(re);

	/// restore window position
	FS_DEV.m_camera.set_window_pos_x(xz_init_x, xz_init_y);
	FS_DEV.m_camera.set_window_pos_y(yz_init_x, yz_init_y);

	FS_DEV.stop();
	FS_DEV.disable();

	FS_AUX.stop();

	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::image_move> & evt)
{
 	log_debug("image move:column = %d,row = %d",evt.column, evt.row);
	if (evt.is_pos_x)
	{
		FS_DEV.m_camera.move_window_pos_x(evt.column, evt.row);
	}
	else
	{
		FS_DEV.m_camera.move_window_pos_y(evt.column, evt.row);
	}
	return discard_event();

}

sc::result stRunning::react(const evMsg<zmsg::mid_t::dust_check1_start> &)
{
	m_x_imgs.clear();
	m_y_imgs.clear();
	FS_SM.deactive_waiter<__evImgReady>();

	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
	return discard_event();
}

sc::result stRunning::react(const evMsg<zmsg::mid_t::set_window> & evt)
{
	log_debug("set window %s :column = %d,row = %d", (evt.is_pos_x ? "x" : "y"), evt.column, evt.row);

	if (evt.is_pos_x)
	{
		FS_DEV.m_camera.set_window_pos_x(evt.column, evt.row);
	}
	else
	{
		FS_DEV.m_camera.set_window_pos_y(evt.column, evt.row);
	}
	return discard_event();
}


sc::result stRunning::react(const evImgProcessError &)
{
	DCL_ZMSG(full_dust_check_result) & re = FS_DAT;
	re.code = fs_err_t::img_process_error;

	FS_REPORT(re);
	return transit< stIdle >();
}

sc::result stRunning::react(const evSystemError &)
{
	DCL_ZMSG(full_dust_check_result) & re = FS_DAT;
	re.code = fs_err_t::system_error;

	FS_REPORT(re);
	return transit< stIdle >();
}

sc::result stRunning::react(const __evImgReady & evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			m_x_imgs.push_back(evt.img.xz_img);
			if (m_x_imgs.size() < FS_SPEC.img_denoise_threshold) {
				FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
				return discard_event();
			}
			break;
		case cmosId_t::Y:
			m_y_imgs.push_back(evt.img.yz_img);
			if (m_y_imgs.size() < FS_SPEC.img_denoise_threshold) {
				FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
				return discard_event();
			}
			break;
		default:
			break;
	}

	if (m_x_imgs.size() < FS_SPEC.img_denoise_threshold
	    || m_y_imgs.size() < FS_SPEC.img_denoise_threshold) {
		/// \todo report progress
		return discard_event();
	}

	DCL_ZMSG(full_dust_check_result) & re = FS_DAT;
	FS_DAT.xz_ori = "/tmp/xz_dc_ori.pgm";
	FS_DAT.xz_dust = "/tmp/xz_dc_dust.png";
	FS_DAT.yz_ori = "/tmp/yz_dc_ori.pgm";
	FS_DAT.yz_dust = "/tmp/yz_dc_dust.png";

	const double dust_check_threshold0 = 12;
	const double dust_check_threshold1 = 36;

	/// yz
	gray_img yz_denoise;
	gray_img yz_dust;
	img_process::img_denoise(yz_denoise, m_y_imgs);
	img_process::check_dust(yz_denoise, dust_check_threshold0, dust_check_threshold1, yz_dust);

	/// xz
	gray_img xz_denoise;
	gray_img xz_dust;
	img_process::img_denoise(xz_denoise, m_x_imgs);
	img_process::check_dust(xz_denoise, dust_check_threshold0, dust_check_threshold1, xz_dust);

	re.xz_ok = img_process::is_img_clean(xz_dust);
	re.yz_ok = img_process::is_img_clean(yz_dust);
	img_process::save_pgm(FS_DAT.xz_ori, xz_denoise);
	img_process::save_pgm(FS_DAT.yz_ori, yz_denoise);
	img_process::save_dust(FS_DAT.xz_dust, xz_dust);
	img_process::save_dust(FS_DAT.yz_dust, yz_dust);

	///\note success does not mark the img clean or not
	re.code = fs_err_t::success;
	FS_REPORT(re);

	return discard_event();
}

} /* namespace smFullDustCheck */

} /* namespace svcFS */
