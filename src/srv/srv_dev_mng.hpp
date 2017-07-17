#pragma once

#include <exemodel/poller.hpp>
#include <exemodel/serveree.hpp>
#include <exemodel/signalee.hpp>

#include <zmsg/zmsg_cmm.hpp>
#include <zmsg/zmsg_utils.hpp>
#include <zmsg/zmsg_packer.hpp>
#include <zmsg/zmsg_unpacker.hpp>

#include <evt_cmm.hpp>

#include "dm/dev_mng_ctx.hpp"

namespace svcDevMng {

class svcDeviceManager final : public exemodel::poller {
private: typedef svcDeviceManager self_t;
public:
	svcDeviceManager(const hw_info & hwinfo, uint16_t port);
	virtual ~svcDeviceManager();
private:
	const hw_info & m_hwinfo;

	std::function<void (void)> m_msg_cb[zmsg::to_val(zmsg::mid_t::max)];
	exemodel::serveree m_svr;
	zmsg::rcver m_recver;
	zmsg::sender m_sender;

	dev_mng_ctx m_ctx;
	/// for shutdown
	exemodel::signalee m_sigterm;
};

}
