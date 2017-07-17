#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_dust_check.hpp"

namespace svcFS {

namespace smDustCheck {

stRunning::stRunning(my_context ctx)
: my_base(ctx)
, m_cfg(context<fs_sm>().m_ctx.dc.cfg)
, m_data(context<fs_sm>().m_ctx.dc.data)
, m_x_done(false)
, m_y_done(false)
{
	FS_DEV.enable();
	log_debug("dc: dust check...");
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
}

stRunning::~stRunning()
{
	FS_DEV.stop();
	FS_DEV.disable();

	FS_AUX.stop();

	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stRunning::react(const evImgProcessError &)
{
	DCL_ZMSG(dust_check_result) & re = FS_DAT;
	re.code = fs_err_t::img_process_error;

	FS_REPORT(re);
	return transit< stIdle >();
}

sc::result stRunning::react(const evSystemError &)
{
	DCL_ZMSG(dust_check_result) & re = FS_DAT;
	re.code = fs_err_t::system_error;

	FS_REPORT(re);
	return transit< stIdle >();
}

sc::result stRunning::react(const evCoverOpen &)
{
	DCL_ZMSG(dust_check_result) & re = FS_DAT;
	re.code = fs_err_t::cover_openned;

	FS_REPORT(re);
	return transit< stIdle >();
}

sc::result stRunning::react(const __evImgReady & evt)
{
	DCL_ZMSG(dust_check_result) & re = FS_DAT;
	FS_DAT.xz_ori = "/tmp/xz_dc_ori.pgm";
	FS_DAT.xz_dust = "/tmp/xz_dc_dust.png";
	FS_DAT.yz_ori = "/tmp/yz_dc_ori.pgm";
	FS_DAT.yz_dust = "/tmp/yz_dc_dust.png";

	const double dust_check_threshold0 = 12;
	const double dust_check_threshold1 = 36;

	switch (evt.id) {
		case cmosId_t::X:
		{
			m_x_imgs.push_back(evt.img.xz_img);
			if (m_x_imgs.size() < FS_SPEC.img_denoise_threshold) {
				FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
				/// \todo report progress
				return discard_event();
			}

			gray_img xz_denoise;
			gray_img xz_dust;
			img_process::img_denoise(xz_denoise, m_x_imgs);
			img_process::check_dust(xz_denoise, dust_check_threshold0, dust_check_threshold1, xz_dust);

			re.xz_ok = img_process::is_img_clean(xz_dust);
			img_process::save_pgm(FS_DAT.xz_ori, xz_denoise);
			img_process::save_dust(FS_DAT.xz_dust, xz_dust);
			m_x_done = true;
			break;
		}
		case cmosId_t::Y:
		{
			m_y_imgs.push_back(evt.img.yz_img);

			if (m_y_imgs.size() < FS_SPEC.img_denoise_threshold) {
				FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
				/// \todo report progress
				return discard_event();
			}

			gray_img yz_denoise;
			gray_img yz_dust;
			img_process::img_denoise(yz_denoise, m_y_imgs);
			img_process::check_dust(yz_denoise, dust_check_threshold0, dust_check_threshold1, yz_dust);

			re.yz_ok = img_process::is_img_clean(yz_dust);
			img_process::save_pgm(FS_DAT.yz_ori, yz_denoise);
			img_process::save_dust(FS_DAT.yz_dust, yz_dust);
			m_y_done = true;
			break;
		}
		default:
			break;
	}

	///\note success does not mark the img clean or not
	if (m_x_done && m_y_done){
		re.code = fs_err_t::success;
		FS_REPORT(re);

		return transit< stIdle >();
	}
	return discard_event();

}

} /* namespace smDustCheck */

} /* namespace svcFS */
