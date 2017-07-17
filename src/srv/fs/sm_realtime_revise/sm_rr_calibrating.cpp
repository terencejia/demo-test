#include <fs_log.hpp>

#include "fs_spec.hpp"
#include "img_process.hpp"

#include "sm_realtime_revise.hpp"

namespace svcFS {

namespace smRealtimeRevise {
/**
 * stCalibrating_starter
 */
struct stCalibrating;
stCalibrating_starter::stCalibrating_starter(my_context ctx)
: my_base(ctx)
{
	post_event(__evEntryAct());
	log_debug("%s...", FS_STATE_NAME);
}

stCalibrating_starter::~stCalibrating_starter()
{
}

sc::result stCalibrating_starter::react(const __evEntryAct &)
{
	return transit<stCalibrating>();
}
/**
 * stCalibrating
 */
struct stCalibrating
: sc::state< stCalibrating, stFS_stage1::orthogonal< 2 > > {
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
	stCalibrating(my_context ctx)
	: my_base(ctx)
	, m_x_dist(std::numeric_limits<int32_t>::max())
	, m_y_dist(std::numeric_limits<int32_t>::max())
	{
		DCL_ZMSG(fs_state) msg;
		msg.sstate = svc_fs_state_t::fs_calibrating;
		FS_REPORT(msg);
		log_debug("%s...", FS_STATE_NAME);

		post_event(__evEntryAct());
	}

	~stCalibrating()
	{
		FS_SM.deactive_waiter<__evImgReady>();
	}

	sc::result react(const __evEntryAct &)
	{
		FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
		FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
		return discard_event();
	}

	sc::result react(const __evImgReady & evt)
	{
		try {
			switch (evt.id) {
				case cmosId_t::X:
					{
						const double xz_delta = img_process::xy_dist(evt.img.xz_img, {0});
						int32_t const x_dist = (FS_SPEC.is_xy_dist_ok_for_calibrating(xz_delta)
							? 0 : FS_SPEC.pixel_to_xy_step(xz_delta));

						if (FS_DEV.m_motorX->is_stopped()) {
						if (x_dist != 0) {
							FS_DEV.m_motorX->start_by_speed(
								FS_SPEC.xy_speed_from_pixel(xz_delta),
								x_dist > 0 ? motor::go_forward : motor::go_backward);
							}
						}
						else {
							if ((x_dist == 0)		/// already in postion
							|| (x_dist > 0 && m_x_dist < 0)
							|| (x_dist < 0 && m_x_dist > 0)) {		/// different direction
								FS_DEV.m_motorX->stop();
							}
							else {
								FS_DEV.m_motorX->set_speed(FS_SPEC.xy_speed_from_pixel(xz_delta));
							}
						}
						m_x_dist = x_dist;

						if(m_x_dist != 0)
							FS_SM.active_waiter<__evImgReady>(cmosId_t::X);
					}
					break;
				case cmosId_t::Y:
					{
						const double yz_delta = img_process::xy_dist(evt.img.yz_img, {0});
						int32_t const y_dist = (FS_SPEC.is_xy_dist_ok_for_calibrating(yz_delta)
							? 0 : FS_SPEC.pixel_to_xy_step(yz_delta));

						if (FS_DEV.m_motorY->is_stopped()) {
							if (y_dist != 0) {
								FS_DEV.m_motorY->start_by_speed(
									FS_SPEC.xy_speed_from_pixel(yz_delta),
									y_dist > 0 ? motor::go_forward : motor::go_backward);
							}
						}
						else {
							if ((y_dist == 0)
							|| (y_dist > 0 && m_y_dist < 0)
							|| (y_dist < 0 && m_y_dist > 0)) {
								FS_DEV.m_motorY->stop();
							}
							else {
								FS_DEV.m_motorY->set_speed(FS_SPEC.xy_speed_from_pixel(yz_delta));
							}
						}
						m_y_dist = y_dist;

						if(m_y_dist != 0)
							FS_SM.active_waiter<__evImgReady>(cmosId_t::Y);
					}
					break;
				default :
					break;
			}

			if (m_x_dist == 0 && m_y_dist == 0) {
				return transit<stCalibratingDone>();
			}
		}
		catch(const img_process::img_process_error & e) {
			if(e.err_type() == img_process::iae_t::no_fiber) {
				FS_SM.active_waiter<__evImgReady>(evt.id);
				return discard_event();
			}
		}

		return discard_event();
	}

	//sc::result react(const evMotorTestTimeout &); //?
private:
	int32_t m_x_dist;
	int32_t m_y_dist;
};

/**
 * stCalibratingDone
 */
stCalibratingDone::stCalibratingDone(my_context ctx)
: my_base(ctx)
{
	post_event(evCalibratingDone());
}

stCalibratingDone::~stCalibratingDone()
{
}

}

}
