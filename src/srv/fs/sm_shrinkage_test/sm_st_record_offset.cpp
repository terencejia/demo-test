#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_shrinkage_test.hpp"

namespace svcFS {

namespace smShrinkageTest {

stRecordOffset::stRecordOffset(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
}

stRecordOffset::~stRecordOffset()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stRecordOffset::react(const __evEntryAct &)
{
	FS_SM.active_delay_waiter<__evImgReady>();

	return discard_event();
}

sc::result stRecordOffset::react(const __evImgReady & evt)
{
	int fusion_point = static_cast<int>(evt.img.width() / 2
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FSPosSetup * 1000));
	int fusion_dist = static_cast<int>(FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberIntervalSetup * 1000) / 2
		+ FS_SPEC.yz_nm_to_pixel(FS_CFG.FiberOverlapSetup * 1000));

	const double core_diff_pixel = std::sqrt(
		std::pow(img_process::xy_precise_dist_core(evt.img.yz_img, FS_IPP, fusion_point, fusion_dist), 2)
		+ std::pow(img_process::xy_precise_dist_core(evt.img.xz_img, FS_IPP, fusion_point, fusion_dist), 2));
	FS_DAT.z_record_offset.core_diff_pre = FS_SPEC.yz_pixel_to_nm(core_diff_pixel) / 1000.0;

	const double cladding_diff_pixel =  std::sqrt(
		std::pow(img_process::xy_precise_dist_cladding(evt.img.yz_img, FS_IPP, fusion_point, fusion_dist), 2)
		+ std::pow(img_process::xy_precise_dist_cladding(evt.img.xz_img, FS_IPP, fusion_point, fusion_dist), 2));
	FS_DAT.z_record_offset.cladding_diff_pre = FS_SPEC.yz_pixel_to_nm(cladding_diff_pixel) / 1000.0;

	FS_DAT.z_record_offset.vertex_intersect_angle = img_process::vertex_axial_angle(evt.img, FS_IPP);

	DCL_ZMSG(record_off_set)& msg = FS_DAT.z_record_offset;
	FS_REPORT(msg);

	return transit<stDischarge1>();
}

} /* namespace smShrinkageTest */

} /* namespace svcFS */
