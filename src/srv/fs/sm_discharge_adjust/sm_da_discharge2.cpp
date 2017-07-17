#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_discharge_adjust.hpp"

#include "fs_debug.hpp"
namespace svcFS {

namespace smDischargeAdjust {
stDischarge2::stDischarge2(my_context ctx)
: my_base(ctx)
, m_left_pos(0)
, m_right_pos(0)
{
	log_debug("da: discharge2...");
	FS_SM.active_delay_waiter<__evImgReady>();
}

stDischarge2::~stDischarge2()
{
	FS_SM.deactive_waiter<__evImgReady>();
}

sc::result stDischarge2::react(const __evImgReady & evt)
{
	m_left_pos = img_process::left_vertex_pos(evt.img.yz_img, FS_IPP);
	m_right_pos = img_process::right_vertex_pos(evt.img.yz_img, FS_IPP);

	FS_AUX.m_discharge_tmr.start({
		{0, 0},
		exemodel::ms_to_timespec(FS_CFG.Discharge1Time)});

	FS_DEV.m_hvb.start(FS_CFG.Discharge1Strength);

	return discard_event();
}

sc::result stDischarge2::react(const evDischargeTimeout &)
{
	FS_DEV.m_hvb.stop();
	FS_SM.active_delay_waiter<evHvbReady>(cmosId_t::Y);
	return discard_event();
}

static mag2shrink_t average_val(const std::vector<mag2shrink_t> & src)
{
	auto cnt = src.size();
	if (cnt == 0) {
		return { 0, 0 };
	}

	mag2shrink_t sum = { 0, 0, };
	for (auto i : src) {
		sum.x += i.x;
		sum.y += i.y;
	}

	return { sum.x / cnt, sum.y / cnt, };
}

sc::result stDischarge2::react(const evHvbReady & evt)
{
	img_process::save_pgm("/tmp/da2y.pgm",evt.img.yz_img);
	const double y_left_pos = img_process::left_vertex_fine_pos(evt.img.yz_img, FS_DAT.y_left_abs);
	const double y_right_pos = img_process::right_vertex_fine_pos(evt.img.yz_img, FS_DAT.y_right_abs);

//	const double error_max = 12.0;
//	const double error_min = -8.0;
	const double error_max = 12.0;
	const double error_min = -8.0;

	const double unrevise_strength = FS_CFG.Discharge1Strength;
	const double real_shrink_um = FS_SPEC.yz_pixel_to_nm(
		(y_right_pos - y_left_pos) - (m_right_pos - m_left_pos)) / 1000.0;
	/// check if valid
	const double base_shrink_um = (FS_SPEC.discharge_base.empty() ?
				55.0 : FS_SPEC.discharge_base.p[1].y);

	const double adjust2_error = real_shrink_um - base_shrink_um;
	log_warning("da-2 unrevise_strength: %f, real_shrink_um: %f, base_shrink_um: %f", unrevise_strength, real_shrink_um, base_shrink_um);
	if (unrevise_strength < FS_GLB_DAT.discharge2_min.x  //when the user changes unrevise_strength manually
	    || (FS_GLB_DAT.discharge2_min.x < unrevise_strength && unrevise_strength < FS_GLB_DAT.discharge2_max.x && real_shrink_um < (base_shrink_um + error_min))) {
		FS_GLB_DAT.discharge2_min = { unrevise_strength, real_shrink_um };
	}
	if (unrevise_strength > FS_GLB_DAT.discharge2_max.x //when the user changes unrevise_strength manually
	    || (FS_GLB_DAT.discharge2_max.x > unrevise_strength && unrevise_strength > FS_GLB_DAT.discharge2_min.x && real_shrink_um > (base_shrink_um + error_max))) {
		FS_GLB_DAT.discharge2_max = { unrevise_strength, real_shrink_um };
	}
	log_warning("da-2 max.strength: %f, max.shrink: %f, min.strength: %f, min.shrink: %f",
		FS_GLB_DAT.discharge2_max.x,
		FS_GLB_DAT.discharge2_max.y,
		FS_GLB_DAT.discharge2_min.x,
		FS_GLB_DAT.discharge2_min.y);
	if (adjust2_error > error_max || adjust2_error < error_min) {
		FS_DAT.code = fs_err_t::revise2_mag;
		FS_DAT.base = FS_SPEC.discharge_base;
		FS_DAT.suggest2 = __autoAdjust_dischargeStrength(base_shrink_um);
		FS_DAT.revise.p[1].x = unrevise_strength;
		FS_DAT.revise.p[1].y = real_shrink_um;
		log_warning("da2 mag %.3f shrink %.3f suggest %.3f", unrevise_strength, real_shrink_um, FS_DAT.suggest2);
		return transit<stWaitReset>();
	}

	double trimming = 0.0;
	if (adjust2_error <= -2.0 && adjust2_error >= -8.0) {
		trimming = 0.01;
	}

	if (adjust2_error <= 12.0 && adjust2_error >= 3.0) {
		trimming = -0.01;
	}

	FS_GLB_DAT.d2_seq.push_back({ unrevise_strength, real_shrink_um, });
	FS_DAT.suggest2 = unrevise_strength + trimming;

	FS_DAT.revise.p[1].x = unrevise_strength;
	FS_DAT.revise.p[1].y = real_shrink_um;
	log_warning("da2 mag %.3f shrink %.3f suggest %.3f", unrevise_strength, real_shrink_um, FS_DAT.suggest2);

	FS_DAT.revise.p[0] = average_val(FS_GLB_DAT.d1_seq);
	FS_DAT.revise.p[1] = average_val(FS_GLB_DAT.d2_seq);

	return transit<stSuccess>();
}

double stDischarge2::__autoAdjust_dischargeStrength(const double base_shrink_um)
{
	double revise_strength = 0.0;
	if( FS_GLB_DAT.discharge2_min.y <= 0 ) {
		 FS_GLB_DAT.discharge2_min.y = 1;
	}
	const auto x1 = FS_GLB_DAT.discharge2_max.x;
	const auto x2 = FS_GLB_DAT.discharge2_min.x;
	const auto ln_y1 = std::log(FS_GLB_DAT.discharge2_max.y);
	const auto ln_y2 = std::log(FS_GLB_DAT.discharge2_min.y);
	const auto ln_y3 = std::log(base_shrink_um);
	//const auto x3 = ?;
	revise_strength = (ln_y1 - ln_y2) * x1 * x2 / ((ln_y3 - ln_y2) * x2 + (ln_y1 - ln_y3) * x1);

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
