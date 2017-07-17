#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise {

static bool __handle_img(const gray_img & gi,
			const double ref_br,
			const double diff_br,
			long & min_brightness,
			double & min_br,
			long & max_brightness,
			double & max_br,
			exemodel::dev_attr_rw<long> & dev)
{
	double cur_br = img_process::img_gray_scale(gi);
	if (std::fabs(cur_br - ref_br) < diff_br) {
		return true;
	}

	if (cur_br < ref_br) {
		min_brightness = dev.read();
		min_br = cur_br;
	}
	else {
		max_brightness = dev.read();
		max_br = cur_br;
	}

// 	if (max_brightness <= min_brightness) {
// 		throw std::runtime_error("adjust brightness through led!");
// 	}

	const double ratio = (ref_br - min_br) / (max_br - min_br);
	const long dst_brightness = min_brightness + static_cast<long>((max_brightness - min_brightness) * ratio);
	dev.write(dst_brightness);
	log_debug("brightness min(%ld, %.3f) max(%ld, %.3f) next(%ld)",  min_brightness, min_br, max_brightness, max_br, dst_brightness);
	if (max_brightness == dst_brightness || min_brightness == dst_brightness) {
		throw std::runtime_error("adjust brightness through led!");
	}
	return false;
}

struct stAdjustBrightness;
stAdjustBrightness_starter::stAdjustBrightness_starter(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
	log_debug("%s...", FS_STATE_NAME);
}

stAdjustBrightness_starter::~stAdjustBrightness_starter()
{
}

sc::result stAdjustBrightness_starter::react(const __evEntryAct &)
{
	return transit<stAdjustBrightness>();
}

/**
 * state: adjust brightness
 */
struct stAdjustBrightness
: sc::state< stAdjustBrightness, stFS_stage1::orthogonal< 0 > > {
private:
	struct __evEntryAct : sc::event< __evEntryAct > {};
	struct __evImgReady final : sc::event< __evImgReady > {
		__evImgReady(const fs_imgs & i, const cmosId_t cmosid)
		: img(i)
		, id(cmosid)
		{
		}

		const fs_imgs & img;
		const cmosId_t id;
	};

public:
	typedef boost::mpl::list<
		sc::custom_reaction< __evEntryAct >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stAdjustBrightness(my_context ctx)
	: my_base(ctx)
	, m_ref_br(FS_SPEC.background_brightness())
	, m_diff_br(0.02)
	, m_ori_x(FS_DEV.m_x_exposure->read())
	, m_ori_y(FS_DEV.m_y_exposure->read())
	, m_min_x(FS_HWINFO.x_cmos_exposure_min)
	, m_min_x_l(0.0)
	, m_max_x(FS_HWINFO.x_cmos_exposure_max)
	, m_max_x_l(1.0)
	, m_min_y(FS_HWINFO.y_cmos_exposure_min)
	, m_min_y_l(0.0)
	, m_max_y(FS_HWINFO.y_cmos_exposure_max)
	, m_max_y_l(1.0)
	, m_x_done(false)
	, m_y_done(false)
	{
		post_event(__evEntryAct());
		log_debug("%s...", FS_STATE_NAME);
	}

	~stAdjustBrightness()
	{
		FS_SM.deactive_waiter<__evImgReady>();
	}

	sc::result react(const __evEntryAct &)
	{
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
		FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const __evImgReady & evt)
	{
		exemodel::dev_attr_rw<long> & devx = *FS_DEV.m_x_exposure;
		exemodel::dev_attr_rw<long> & devy = *FS_DEV.m_y_exposure;

		try {
			switch (evt.id) {
				case cmosId_t::X:
					log_debug("adjust x brightness:");
					m_x_done = __handle_img(evt.img.xz_img, m_ref_br, m_diff_br, m_min_x, m_min_x_l, m_max_x, m_max_x_l, devx);
					if (!m_x_done)
						FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::X);
					break;
				case cmosId_t::Y:
					log_debug("adjust y brightness:");
					m_y_done = __handle_img(evt.img.yz_img, m_ref_br, m_diff_br, m_min_y, m_min_y_l, m_max_y, m_max_y_l, devy);
					if (!m_y_done)
						FS_SM.active_delay_waiter<__evImgReady>(cmosId_t::Y);
					break;
				default:
					break;
			}
			if (m_x_done && m_y_done) {
				if (devx.read() != m_ori_x) {
					DCL_ZMSG(update_led_brightness) msg;
					msg.id = ledId_t::CMOS_X;
					msg.brightness = static_cast<double>(devx.read()) / FS_HWINFO.x_cmos_exposure_max;
					FS_REPORT(msg);
				}
				if (devy.read() != m_ori_y) {
					DCL_ZMSG(update_led_brightness) msg;
					msg.id = ledId_t::CMOS_Y;
					msg.brightness = static_cast<double>(devy.read()) / FS_HWINFO.y_cmos_exposure_max;
					FS_REPORT(msg);
				}
				return transit<stAdjustBrightnessDone>();
			}
		} catch (...) {
			img_process::save_pgm("errorx.pgm", evt.img.xz_img);
			img_process::save_pgm("errory.pgm", evt.img.yz_img);
			log_err("%s adjust brightness error", FS_STATE_NAME);
			devx.write(m_ori_x);
			devy.write(m_ori_y);
			FS_DAT.code = fs_err_t::img_brightness;

			return transit<stReset>();
		}

		return discard_event();
	}
private:
	const double m_ref_br;
	const double m_diff_br;

	/// original brightness
	const long m_ori_x;
	const long m_ori_y;

	long m_min_x;
	double m_min_x_l;
	long m_max_x;
	double m_max_x_l;

	long m_min_y;
	double m_min_y_l;
	long m_max_y;
	double m_max_y_l;

	bool m_x_done;
	bool m_y_done;
};

stAdjustBrightnessDone::stAdjustBrightnessDone(my_context ctx)
: my_base(ctx)
{
	post_event(evAdjustBrightnessDone());
}

stAdjustBrightnessDone::~stAdjustBrightnessDone()
{
}

} /* smRealtimeRevise */

} /* namespace svcFS */
