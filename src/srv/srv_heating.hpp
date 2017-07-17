#pragma once

#include <exemodel/poller.hpp>
#include <exemodel/fifo.hpp>
#include <exemodel/serveree.hpp>
#include <exemodel/signalee.hpp>

#include <zmsg/zmsg_cmm.hpp>
#include <zmsg/zmsg_utils.hpp>
#include <zmsg/zmsg_packer.hpp>
#include <zmsg/zmsg_unpacker.hpp>

#include "heat/heat_ctx.hpp"
#include "heat/heat_sm.hpp"

#include "hw_info.hpp"

namespace svcHeat {

class svcHeating final : public exemodel::poller {
private: typedef svcHeating self_t;
public:
	svcHeating(const hw_info & hwinfo, uint16_t port);
	virtual ~svcHeating();
private:
	template< zmsg::mid_t mid >
	void __init_msg_cb()
	{
		m_msg_cb[zmsg::to_val(mid)] = [this](void) {
			evMsg<mid> msg;
			m_recver.convert_to(msg);
			m_sm.process_event(msg);
		};
	}

	template< zmsg::mid_t ... mid >
	void __init_msgs_cb()
	{
		auto a = {
			(__init_msg_cb<mid>(), 1)...
		};
		(void)a;
	}
private:
	const hw_info & m_hwinfo;

	std::function<void (void)> m_msg_cb[zmsg::to_val(zmsg::mid_t::max)];
	exemodel::serveree m_svr;
	zmsg::rcver m_recver;
	zmsg::sender m_sender;

	heat_ctx m_ctx;
	heat_sm m_sm;

	/// for debug
	exemodel::fifo_readee m_fifo;
	/// for shutdown
	exemodel::signalee m_sigterm;
};

}
