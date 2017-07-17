
#include "../fs_sm.hpp"

namespace svcFS {

namespace smDustCheck {

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
		sc::custom_reaction< evImgProcessError >,
		sc::custom_reaction< evSystemError >,
		sc::transition< evMsg<zmsg::mid_t::stop>, stReset >,
		sc::custom_reaction< evCoverOpen >,
		sc::custom_reaction< __evImgReady >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();
public:
	sc::result react(const evImgProcessError &);
	sc::result react(const evSystemError &);

	
	sc::result react(const evCoverOpen &);
	sc::result react(const __evImgReady &);
public:
	const dc_cfg_t & m_cfg;
	dc_data_t & m_data;
private:
	std::vector<gray_img> m_x_imgs;
	std::vector<gray_img> m_y_imgs;

	bool m_x_done;
	bool m_y_done;
};

} /* namespace smDustCheck */

} /* namespace svcFS */
