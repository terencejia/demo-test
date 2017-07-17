
#include <zmsg/zmsg_regular_test.hpp>
#include <zmsg/zmsg_motor_oper.hpp>
#include <zmsg/zmsg_camera_oper.hpp>
#include <zmsg/zmsg_discharge.hpp>
#include <zmsg/zmsg_display_oper.hpp>
#include <zmsg/zmsg_led.hpp>
#include <zmsg/zmsg_wave_form.hpp>
#include <zmsg/zmsg_defect_detect.hpp>

#include "../fs_sm.hpp"

namespace svcFS {

namespace smRegularTest {

/**
 * state: running
 */
struct stRunning : sc::state< stRunning, fs_sm>
, helper< stRunning > {

	typedef boost::mpl::list<
		  sc::custom_reaction< evImgProcessError >
		, sc::custom_reaction< evSystemError >
		, sc::transition< evMsg<zmsg::mid_t::stop>, stReset >
		, sc::custom_reaction< evMsg<zmsg::mid_t::motor_start> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::motor_stop> >
		, sc::custom_reaction< evMotorBackEnd<motorId_t::LZ> >
		, sc::custom_reaction< evMotorBackEnd<motorId_t::RZ> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::discharge> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::stop_discharge> >
		, sc::custom_reaction< evDischargeTimeout >
		, sc::custom_reaction< evMsg<zmsg::mid_t::image_move> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::set_led> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::query_wave_form> >
		, sc::custom_reaction< evWaveFormInfo >
		, sc::custom_reaction< evMsg<zmsg::mid_t::manual_focal_distance> >
		, sc::custom_reaction< evMsg<zmsg::mid_t::get_fiber_defect_info> >
		, sc::custom_reaction< evFiberDefectInfo >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();
public:
	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);

	sc::result react(const evMsg<zmsg::mid_t::motor_start> & evt);
	sc::result react(const evMsg<zmsg::mid_t::motor_stop> & evt);

	sc::result react(const evMotorBackEnd<motorId_t::LZ> & evt);
	sc::result react(const evMotorBackEnd<motorId_t::RZ> & evt);

	sc::result react(const evMsg<zmsg::mid_t::discharge> &);
	sc::result react(const evMsg<zmsg::mid_t::stop_discharge> &);
	sc::result react(const evDischargeTimeout &);

	sc::result react(const evMsg<zmsg::mid_t::image_move> & evt);

	sc::result react(const evMsg<zmsg::mid_t::set_led> & evt);

	sc::result react(const evMsg<zmsg::mid_t::query_wave_form> &);
	sc::result react(const evWaveFormInfo &);

	sc::result react(const evMsg<zmsg::mid_t::manual_focal_distance> &);

	sc::result react(const evMsg<zmsg::mid_t::get_fiber_defect_info> &);
	sc::result react(const evFiberDefectInfo &);
public:
	const rt_cfg_t & m_cfg;
	rt_data_t & m_data;
private:
	fs_imgs m_dt_img;
	bool m_dt_x_ready;
	bool m_dt_y_ready;
private:
	fs_imgs m_wave_img;
	bool m_wave_x_ready;
	bool m_wave_y_ready;

};

} /* namespace smRegularTest */

} /* namespace svcFS */
