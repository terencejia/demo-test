#include <zmsg/zmsg_camera_oper.hpp>
#include "../fs_sm.hpp"

namespace svcFS {

namespace smFullDustCheck {

/**
 * state: running
 */
struct stRunning
: sc::state< stRunning, fs_sm> {
private:
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
		sc::custom_reaction< evMsg<zmsg::mid_t::image_move> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::dust_check1_start> >,
		sc::custom_reaction< evMsg<zmsg::mid_t::set_window> >,
		sc::custom_reaction< evImgProcessError >,
		sc::custom_reaction< evSystemError >,
		sc::transition< evMsg<zmsg::mid_t::stop>, stReset >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();
public:
	sc::result react(const evMsg<zmsg::mid_t::image_move> & evt);
	sc::result react(const evMsg<zmsg::mid_t::dust_check1_start> &);
	sc::result react(const evMsg<zmsg::mid_t::set_window> &);
	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);

	sc::result react(const __evImgReady &);
public:
	const fc_cfg_t & m_cfg;
	fc_data_t & m_data;
private:
	std::vector<gray_img> m_x_imgs;
	std::vector<gray_img> m_y_imgs;

	int32_t xz_cur_x;
	int32_t xz_cur_y;
	int32_t yz_cur_x;
	int32_t yz_cur_y;

	/// \note the init window position, should be const
	int32_t xz_init_x;
	int32_t xz_init_y;
	int32_t yz_init_x;
	int32_t yz_init_y;

	bool m_x_continue;
	bool m_y_continue;
};

} /* namespace smDustCheck */

} /* namespace svcFS */
