
#include <fs_log.hpp>

#include <exemodel/poll_tools.hpp>
#include <zmsg/zmsg_dev_mng.hpp>
#include <zmsg/zmsg_lcd_oper.hpp>

#include "srv_dev_mng.hpp"
#include "heater_adc2temp.hpp"

namespace svcDevMng {

svcDeviceManager::svcDeviceManager(const hw_info & hwinfo, uint16_t port): exemodel::poller()
, m_hwinfo(hwinfo)
, m_svr(port), m_recver(), m_sender()
, m_ctx(hwinfo)
, m_sigterm(SIGUSR1)
{
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

	/// msg handling
	m_msg_cb[zmsg::to_val(zmsg::mid_t::query_dev_state)] = [this](void) {
		DCL_ZMSG(query_dev_state) msg;
		m_recver.convert_to(msg);

		DCL_ZMSG(report_dev_state) ack;
		ack.pressure = m_ctx.m_pressure.read();
		ack.humidity = (m_ctx.m_humidity ? m_ctx.m_humidity->read() : 0.0);
		ack.int_temp = m_ctx.m_int_temp.read();
		ack.env_temp = m_ctx.m_env_temp.read();

		Adc2Temp adc2temp;
		int x = m_ctx.m_heat_temp.read();
		ack.heat_temp = adc2temp(x);


		m_sender.fill_to<false, true>(ack,
			[this](const void * buf, size_t size) -> size_t {
				return m_svr.send(buf, size);
			});
	};
	m_msg_cb[zmsg::to_val(zmsg::mid_t::set_lcd_brightness)] = [this](void) {
		DCL_ZMSG(set_lcd_brightness) msg;
		m_recver.convert_to(msg);

		m_ctx.m_lcd_brightness.write(msg.brightness);
	};

	/// for shutdown
	m_sigterm.connect([this]() {
		throw exemodel::exec_stop("service dev mng exit");
	});
	this->add(m_sigterm);
}

svcDeviceManager::~svcDeviceManager()
{
	this->del(m_sigterm);
	this->del(m_svr);
}

}
