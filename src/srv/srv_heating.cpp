#include <iostream>
#include <regex>

#include <fs_log.hpp>

#include <zmsg/zmsg_heat.hpp>

#include "srv_heating.hpp"

namespace svcHeat {

svcHeating::svcHeating(const hw_info & hwinfo, uint16_t port): exemodel::poller()
, m_hwinfo(hwinfo)
, m_svr(port), m_recver(), m_sender()
, m_ctx(hwinfo)
, m_sm(m_ctx, m_sender,
	[this](const void * buf, size_t size) -> size_t {
		return m_svr.send(buf, size);
	})
, m_fifo("/dev/jilong/heat_info")
, m_sigterm(SIGUSR1)
{
	/// 1. connect
	m_svr.connect([this](exemodel::poller&,
			  uint32_t evts,
			  exemodel::connectee& conn)
	{
		if (evts&::EPOLLIN) {
			bool has_msg = m_recver.fill_from(conn);
			if (has_msg) {
				zmsg::mid_t mid = m_recver.id();
				m_msg_cb[zmsg::to_val(mid)]();
			}
		}
	});
	this->add(m_svr);

	if (m_ctx.m_cover) {
		m_ctx.m_cover->connect([this](exemodel::poller &, uint32_t) {
			do {
				uint16_t tmp;
				auto ret = m_ctx.m_cover->read(&tmp, sizeof(tmp));
				if (ret < (decltype(ret))sizeof(tmp)) {
					break;
				}
			} while (true);

			bool cover_openned = m_ctx.m_cover->get_state();
			log_debug("heater cover : %s",
					cover_openned ? "openned" : "closed");
			if (cover_openned) {
				m_sm.process_event(evCoverOpen());
			}
			else {
				m_sm.process_event(evCoverClose());
			}
		});
		this->add(*m_ctx.m_cover);
	}

	m_ctx.m_check_tmr.connect([this](exemodel::poller&, uint64_t)
	{
		m_sm.process_event(evCheckTimeout());
	});
	this->add(m_ctx.m_check_tmr);

	m_ctx.m_heat_tmr.connect([this](exemodel::poller&, uint64_t)
	{
		m_sm.process_event(evHeatTimeout());
	});
	this->add(m_ctx.m_heat_tmr);

	/// 2. init all msg callback
	__init_msgs_cb<
		  zmsg::mid_t::heat_start
		, zmsg::mid_t::stop
	>();

	/// 3. state machine init
	m_sm.initiate();

	/// heat debug interface
	m_fifo.connect([this](std::vector<uint8_t> & buf) {
		std::string options((char *)buf.data(), buf.size());
		std::vector<std::string> strs;
		// regular expression
		const std::regex pattern("([a-zA-Z0-9]+)");

		static const std::sregex_token_iterator end;
		for (std::sregex_token_iterator i(options.begin(),options.end(), pattern); i != end ; ++i)
		{
			strs.push_back(*i);
		}

		if (strs.size() < 1) {
			return;
		}

		std::cout << "input cmd : " << options <<std::endl;
		for (auto pLeafState = m_sm.state_begin();
			pLeafState != m_sm.state_end(); ++pLeafState) {
			std::cout
				<< "    state: "
				//<< boost::units::simplify_typename(*pLeafState)
				<< boost::units::detail::demangle(typeid(*pLeafState).name())
				<< std::endl;
		}

		std::size_t idx = 0;
		auto & cmd = strs[idx++];
		if (cmd == "heat") {
			if (strs.size() == 2) {
				const int delay_ms = std::stoi(strs[idx++]);
				this->m_ctx.m_check_tmr.set_spec({exemodel::ms_to_timespec(delay_ms), exemodel::ms_to_timespec(delay_ms)});
			}
		} else if (cmd == "help") {
				std::cout << "heat check timer (1 - 2000)" << std::endl
					<< "|  modify heater check timer (uint : ms)" << std::endl;

				return;
		}

		return;
	});
	this->add(m_fifo);

	/// for shutdown
	m_sigterm.connect([this]() {
		throw exemodel::exec_stop("service heating exit");
	});
	this->add(m_sigterm);
}

svcHeating::~svcHeating()
{
	this->del(m_sigterm);
	this->del(m_ctx.m_heat_tmr);
	this->del(m_ctx.m_check_tmr);
	this->del(m_svr);
	this->del(m_fifo);
}

}
