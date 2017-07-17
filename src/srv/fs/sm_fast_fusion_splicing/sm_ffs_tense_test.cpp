#include "img_process.hpp"

#include "fs_spec.hpp"

#include "sm_fast_fusion_splice.hpp"

namespace svcFS {

namespace smFastFusionSplicing {

stTenseTest::stTenseTest(my_context ctx)
: my_base(ctx)
, m_lz_ok(false)
, m_rz_ok(false)
{
	if (FS_CFG.TensionSet) {
		DCL_ZMSG(fs_state) msg;
		msg.sstate = svc_fs_state_t::fs_tension_testing;
		FS_REPORT(msg);
		log_debug("%s...", FS_STATE_NAME);
	}

	post_event(__evEntryAct());
}

stTenseTest::~stTenseTest()
{
	FS_SM.deactive_waiter<__evImgReady>(cmosId_t::Y);
}

sc::result stTenseTest::react(const __evEntryAct &)
{
	if (!FS_CFG.TensionSet) {
		FS_DAT.code = fs_err_t::success;

		return transit<stWaitReset>();
	}

	DCL_ZMSG(fs_state) msg;
	msg.sstate = svc_fs_state_t::fs_tension_testing;
	FS_REPORT(msg);
	log_debug("%s...", FS_STATE_NAME);

	uint32_t const lz_steps = static_cast<uint32_t>(
		FS_SPEC.nm_to_real_lz_step(
			static_cast<int32_t>(
				FS_SPEC.tensionStretchLength) * 1000
			) / 2
	);
	uint32_t const rz_steps = static_cast<uint32_t>(
		FS_SPEC.nm_to_real_rz_step(
			static_cast<int32_t>(
				FS_SPEC.tensionStretchLength) * 1000
			) / 2
	);

	FS_DEV.m_motorLZ->start_by_ss(
		FS_SPEC.tensionSpeed, lz_steps, motor::go_backward);
	FS_DEV.m_motorRZ->start_by_ss(
		FS_SPEC.tensionSpeed, rz_steps, motor::go_backward);

	FS_DAT.z_tense_test_result.is_tense_test = true;
	return discard_event();
}

sc::result stTenseTest::__check()
{
	if (m_rz_ok && m_lz_ok) {
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
	}

	return discard_event();
}

sc::result stTenseTest::react(const __evImgReady & evt)
{
	const auto lvx = img_process::left_vertex_pos(evt.img.yz_img, FS_IPP);
	if (lvx < (evt.img.yz_img.width - 1)) {
		FS_DAT.code = fs_err_t::tense_test_fail;
		FS_DAT.z_tense_test_result.is_success = false;
		log_warning("tense_test_fail!");
	}
	else {
		FS_DAT.code = fs_err_t::success;
		FS_DAT.z_tense_test_result.is_success = true;
	}

	DCL_ZMSG(tense_test_result)& tense_result_msg = FS_DAT.z_tense_test_result;
	FS_REPORT(tense_result_msg);

	return transit<stWaitReset>();
}

sc::result stTenseTest::react(const evMotorStop<motorId_t::LZ> &)
{
	m_lz_ok = true;
	return __check();
}

sc::result stTenseTest::react(const evMotorStop<motorId_t::RZ> &)
{
	m_rz_ok = true;
	return __check();
}

} /* namespace smFastFusionSplicing */

} /* namespace svcFS */
