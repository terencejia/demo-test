#include <fs_log.hpp>
#include "img_process.hpp"

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stPause1::stPause1(my_context ctx)
: my_base(ctx)
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_pause1;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);
	log_debug("now pause1 !");

	m_maxzdiff = 100;
	m_minzdiff = 10;

	m_ProcessImg = false;
}

stPause1::~stPause1()
{
	context<stRunning>().m_stamps.push_back(std::make_tuple(FS_STATE_NAME, exemodel::monotonic_timeree::info::get_time()));
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stPause1::react(const evMsg<zmsg::mid_t::motor_start> & evt)
{
	if(!m_ProcessImg && FS_DEV.m_motorLZ->is_stopped() && FS_DEV.m_motorRZ->is_stopped()
		&& FS_DEV.m_motorX->is_stopped() && FS_DEV.m_motorY->is_stopped()) {
		m_evtms = evt;

		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		m_ProcessImg = true;
		log_debug("start manual motor now ...");
	}

	return discard_event();
}

sc::result stPause1::react(const evMotorBackEnd<motorId_t::LZ> &)
{
	/// \todo we should check if motor is running and backwarding,
	/// or need not stop it.
	/// \todo the driver should use onshot mode for interrupt
	m_ProcessImg = false;
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);

	FS_DEV.m_motorLZ->stop();
	return discard_event();
}

sc::result stPause1::react(const evMotorBackEnd<motorId_t::RZ> &)
{
	m_ProcessImg = false;
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);

	FS_DEV.m_motorRZ->stop();
	return discard_event();
}

sc::result stPause1::react(const evMsg<zmsg::mid_t::motor_stop> & evt)
{
	m_ProcessImg = false;
	FS_SM.deactive_waiter<__evImgReady>();

	switch (evt.id) {
	case motorId_t::LZ:
		FS_DEV.m_motorLZ->stop();
		break;
	case motorId_t::RZ:
		FS_DEV.m_motorRZ->stop();
		break;
	case motorId_t::X:
		FS_DEV.m_motorX->stop();
		break;
	case motorId_t::Y:
		FS_DEV.m_motorY->stop();
		break;
	default:
		break;
	}
	return discard_event();
}

sc::result stPause1::react(const evMsg<zmsg::mid_t::go_on> &)
{
	if (FS_DEV.m_motorX && FS_DEV.m_motorY) {
		return transit<stPreparePreciseCalibrating>();
	} else if (FS_CFG.Stop2) {
		return transit<stPause2>();
	} else {
		return transit<stRecordOffset>();
	}
}

sc::result stPause1::react(const evMsg<zmsg::mid_t::skip> &)
{
	if (FS_CFG.Stop2) {
		return transit<stPause2>();
	}
	else {
		return transit<stRecordOffset>();
	}
}

sc::result stPause1::react(const __evImgReady & evt)
{
	log_debug("once __evImgReady happened.......");

	if(m_ProcessImg) {
		switch (m_evtms.id)  {
		case motorId_t::LZ:
			m_lzdiff = evt.img.yz_img.width / 2 - img_process::left_vertex_pos(evt.img.yz_img, FS_IPP);

			if(((m_lzdiff > m_maxzdiff / 2.0) && (m_evtms.m_forward_dir == motor::go_backward))
				|| ((m_lzdiff < m_minzdiff / 2.0) && (m_evtms.m_forward_dir != motor::go_backward))) {
				if(!FS_DEV.m_motorLZ->is_stopped()) {
					FS_DEV.m_motorLZ->stop();
				}
				log_debug("lzdiff or rzdiff is limited ,now stop lz...");

			}
			else {
				if(FS_DEV.m_motorLZ->is_stopped()) {
					FS_DEV.m_motorLZ->start_by_speed(FS_SPEC.manual_calibrate_speed, m_evtms.m_forward_dir);
					log_debug("move lz...............");
				}

				FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
				m_ProcessImg = true;
			}

			break;
		case motorId_t::RZ:
			m_rzdiff = img_process::right_vertex_pos(evt.img.yz_img, FS_IPP) - evt.img.yz_img.width / 2 ;

			if(((m_rzdiff > m_maxzdiff / 2.0) && (m_evtms.m_forward_dir == motor::go_backward))
				|| ((m_rzdiff < m_minzdiff / 2.0) && (m_evtms.m_forward_dir != motor::go_backward))) {
				if(!FS_DEV.m_motorRZ->is_stopped()) {
					FS_DEV.m_motorRZ->stop();
				}
				log_debug("lzdiff or rzdiff is limited ,now stop rz..");
			}
			else {
				if(FS_DEV.m_motorRZ->is_stopped()) {
					FS_DEV.m_motorRZ->start_by_speed(FS_SPEC.manual_calibrate_speed, m_evtms.m_forward_dir);
					log_debug("move rz...............");
				}

				FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
				m_ProcessImg = true;

			}

			break;
		case motorId_t::X:
			if(FS_DEV.m_motorLZ->is_stopped()) {
				FS_DEV.m_motorX->start_by_speed(FS_SPEC.manual_calibrate_speed, m_evtms.m_forward_dir);
				log_debug("move x...............");
			}

			FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
			m_ProcessImg = true;

			break;
		case motorId_t::Y:
			if(FS_DEV.m_motorLZ->is_stopped()) {
				FS_DEV.m_motorY->start_by_speed(FS_SPEC.manual_calibrate_speed, m_evtms.m_forward_dir);
				log_debug("move y...............");
			}

			FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
			m_ProcessImg = true;

			break;
		default:
			break;
		}
	}  //m_ProcessImg

	return discard_event();
}

}

}
