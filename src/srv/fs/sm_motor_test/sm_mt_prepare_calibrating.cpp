#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_motor_test.hpp"

namespace svcFS {

namespace smMotorTest {

stPrepareCalibrating::stPrepareCalibrating(my_context ctx)
: my_base(ctx)
, m_x_stopped(false)
, m_y_stopped(false)
{
	log_debug("motor test: prepare calibrating");
	m_reverse_start = false;
	m_steps = -60;

	post_event(__evEntryAct());
}

stPrepareCalibrating::~stPrepareCalibrating()
{
	FS_AUX.m_mt_tmr.stop();
}

sc::result stPrepareCalibrating::react(const __evEntryAct &)
{
	FS_AUX.m_mt_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
			10000) });
	FS_DEV.m_motorX->start_by_step(FS_SPEC.pixel_to_xy_step(m_steps));
	FS_DEV.m_motorY->start_by_step(FS_SPEC.pixel_to_xy_step(m_steps));
	return discard_event();
}

sc::result stPrepareCalibrating::react(const evMotorStop<motorId_t::X> &)
{
	if (m_y_stopped) {
		if (!m_reverse_start) {
			FS_AUX.m_mt_tmr.stop();
			m_x_stopped = false;
			m_y_stopped = false;
			m_reverse_start = true;

			FS_AUX.m_mt_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
			20000) });
			FS_DEV.m_motorX->start_by_step(FS_SPEC.pixel_to_xy_step(-m_steps * 2));
			FS_DEV.m_motorY->start_by_step(FS_SPEC.pixel_to_xy_step(-m_steps * 2));

			log_debug("start reverse moving");
			return discard_event();
		}

		return transit<stCalibrating>();
	}
	m_x_stopped = true;
	return discard_event();
}

sc::result stPrepareCalibrating::react(const evMotorStop<motorId_t::Y> &)
{
	if (m_x_stopped) {
		if (!m_reverse_start) {
			FS_AUX.m_mt_tmr.stop();
			m_x_stopped = false;
			m_y_stopped = false;
			m_reverse_start = true;

			FS_AUX.m_mt_tmr.start({ { 0, 0 }, exemodel::ms_to_timespec(
			20000) });
			FS_DEV.m_motorX->start_by_step(FS_SPEC.pixel_to_xy_step(-m_steps * 2));
			FS_DEV.m_motorY->start_by_step(FS_SPEC.pixel_to_xy_step(-m_steps * 2));

			log_debug("start reverse moving");
			return discard_event();
		}

		return transit<stCalibrating>();
	}
	m_y_stopped = true;
	return discard_event();
}

sc::result stPrepareCalibrating::react(const evMotorTestTimeout &)
{
	log_debug("%s... calibrating timeout", FS_STATE_NAME);
	++FS_DAT.calibrate;
	FS_DAT.code = fs_err_t::calibrate_timeout;

	return transit<stWaitReset>();
}

}

}
