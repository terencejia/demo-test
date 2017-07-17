#pragma once

#include "../fs_sm.hpp"

namespace svcFS {

namespace smStabilizeElectrode {

/**
 * state: running
 */
struct stDischarge;
struct stRunning
: sc::state< stRunning, fs_sm, stDischarge > {
	typedef boost::mpl::list<
		sc::custom_reaction< evCoverOpen >,
		sc::transition< evMsg<zmsg::mid_t::stop>, stReset >
	> reactions;
public:
	stRunning(my_context ctx);
	~stRunning();

	sc::result react(const evCoverOpen &);
public:
	const se_cfg_t & m_cfg;
	se_data_t & m_data;
};

/**
 * state: discharge
 */
struct stDischarge : sc::state< stDischarge, stRunning > {
	typedef boost::mpl::list<
		sc::custom_reaction< evDischargeTimeout >
	> reactions;

	stDischarge(my_context ctx);
	~stDischarge();

	sc::result react(const evDischargeTimeout &);
};

/**
 * state: interval
 */
struct stInterval : sc::state< stInterval, stRunning > {
	typedef boost::mpl::list<
		sc::custom_reaction< evDischargeTimeout >
	> reactions;

	stInterval(my_context ctx);
	~stInterval();

	sc::result react(const evDischargeTimeout &);
};

} /* namespace smStabilizeElectrode */

} /* namespace svcFS */
