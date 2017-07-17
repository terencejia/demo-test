#include <fs_log.hpp>
#include "img_process.hpp"
#include <zmsg/zmsg_loss_estimating.hpp>

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stDischargeManual::stDischargeManual(my_context ctx)
: my_base(ctx)
, m_x_ready(false)
, m_y_ready(false)
{
	if (FS_CFG.ExtraManualDischargeTime > 0) {
		DCL_ZMSG(fs_state) msg;
		msg.sstate = svc_fs_state_t::fs_discharge_manual;
		FS_REPORT(msg);
		log_debug("%s...", FS_STATE_NAME);
	}

	m_discharge_counts = 0;
	m_is_discharge = false;
	FS_DEV.m_hvb.set_magnitude(FS_CFG.Discharge1Strength);

	m_is_processImg = true;
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
	FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
}

stDischargeManual::~stDischargeManual()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stDischargeManual::react(const evMsg<zmsg::mid_t::discharge> &)
{
	if (!m_is_discharge && !m_is_processImg) { ///not discharge, not processImg
		if (m_discharge_counts >= FS_CFG.ManualDischargeTimes) {
			return transit<stTenseTest>();
		}

		FS_AUX.m_discharge_tmr.start({
			{0, 0},
			exemodel::ms_to_timespec(FS_CFG.ExtraManualDischargeTime)});
		FS_DEV.m_hvb.start();
		m_is_discharge = true;
		m_discharge_counts ++;
		FS_DAT.z_manual_discharge_counts.counts = m_discharge_counts;
		log_debug("the %d times discharge!", m_discharge_counts);
	}

	return discard_event();
}

sc::result stDischargeManual::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();
	m_is_discharge = false;

	DCL_ZMSG(manual_discharge_counts)& msg = FS_DAT.z_manual_discharge_counts;
	FS_REPORT(msg);//update

	if (FS_CFG.LossEstimationMode != loss_estimate_mode_t::off) {
		m_is_processImg = true;
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
	}

	return discard_event();
}

sc::result stDischargeManual::react(const __evImgReady &evt)
{
	switch (evt.id)
	{
		case cmosId_t::X:
			{
				m_img.xz_img = evt.img.xz_img;
				m_x_ready = true;
			}
			break;
		case cmosId_t::Y:
			{
				m_img.yz_img = evt.img.yz_img;
				m_y_ready = true;
			}
			break;
		default:
			break;
	}
	if ( !(m_x_ready && m_y_ready) ) {
		return discard_event();
	}

	if (!m_is_discharge) {
		const auto lvx = img_process::left_vertex_pos(m_img.yz_img, FS_IPP);
		if (lvx < (m_img.yz_img.width - 1)) { //now the fiber was not broken ?
			FS_DAT.code = fs_err_t::fiber_broken;
			log_warning("fusion failed!!! fiber_broken yz img lvx %d, ipp.bg %f", lvx, FS_IPP.bg);
			img_process::save_pgm("fiber_broken.pgm", m_img.yz_img);

			return transit<stWaitReset>();
		}

		if (FS_CFG.LossEstimationMode != loss_estimate_mode_t::off) {
			DCL_ZMSG(loss_estimating_result) loss_msg = {false, -1.0};
			/// \todo maybe need denoise first
			const int fusion_point = m_img.width() / 2 + static_cast<int>(
				FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
			FS_DAT.loss_db = img_process::loss_estimate(
						m_img, FS_IPP, {
							FS_CFG.LossEstimationMode,
							fusion_point - m_img.width() / 4,
							m_img.width() / 2,
							FS_DAT.z_record_off_set.core_diff_pre,
							FS_DAT.z_record_off_set.cladding_diff_pre,
							FS_DAT.z_record_off_set.vertex_intersect_angle,
							std::max(std::abs(FS_DAT.defect_data.xzl.v_angle), std::abs(FS_DAT.defect_data.yzl.v_angle)),
							std::max(std::abs(FS_DAT.defect_data.xzr.v_angle), std::abs(FS_DAT.defect_data.yzr.v_angle)),
							FS_CFG.LeftFiberMFD,
							FS_CFG.RightFiberMFD,
							FS_CFG.LeastLoss,
							FS_CFG.MFDMismatchCoefficient,
							FS_CFG.RateOfSyntropyBending,
							FS_CFG.RateOfReverseBending,
							FS_SPEC.loss_factor,
						});

			loss_msg.loss_data = FS_DAT.loss_db;
			if (FS_DAT.loss_db >= 0) {
				/// if the cover suddenly opened , but we think the fs is complete;
				FS_DAT.code = fs_err_t::success;
				loss_msg.valid = true;
			} else {
				loss_msg.valid = false;
				FS_DAT.code = fs_err_t::loss_estimate;
			}
			log_debug("now the loss is %f", FS_DAT.loss_db);
			FS_REPORT(loss_msg);
		}
	}

	{
		///\ X img
		FS_DAT.fs_done_x_img = "/tmp/fs_done_x_img.pgm";
		img_process::save_pgm(FS_DAT.fs_done_x_img, m_img.xz_img);

		///\ y img
		FS_DAT.fs_done_y_img = "/tmp/fs_done_y_img.pgm";
		img_process::save_pgm(FS_DAT.fs_done_y_img, m_img.yz_img);
	}

	//complete once img process
	{
		m_x_ready = false;
		m_y_ready = false;
		m_is_processImg = false;
	}

	if (FS_CFG.ExtraManualDischargeTime > 0 && FS_CFG.ManualDischargeTimes > 0) {
		return discard_event();
	}
	else {
		return transit<stTenseTest>();
	}
}

}

}
