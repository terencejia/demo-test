#include <iostream>
#include <cstdlib>
#include <regex>

#include <boost/units/detail/utility.hpp>

#include <fs_log.hpp>

#include <zmsg/zmsg_process_control.hpp>
#include <zmsg/zmsg_svc_state.hpp>
#include <zmsg/zmsg_fusion_splice.hpp>
#include <zmsg/zmsg_discharge_adjust.hpp>
#include <zmsg/zmsg_regular_test.hpp>
#include <zmsg/zmsg_motor_oper.hpp>
#include <zmsg/zmsg_camera_oper.hpp>
#include <zmsg/zmsg_discharge.hpp>
#include <zmsg/zmsg_display_oper.hpp>
#include <zmsg/zmsg_led.hpp>
#include <zmsg/zmsg_wave_form.hpp>
#include <zmsg/zmsg_cmos_focal.hpp>
#include <zmsg/zmsg_defect_detect.hpp>
#include <zmsg/zmsg_shrinkage_test.hpp>
#include <zmsg/zmsg_cover.hpp>

#include "fs_debug.hpp"

#include "srv_fusion_splicing.hpp"

namespace svcFS {

svcFusionSplicing::svcFusionSplicing(const hw_info & hwinfo, uint16_t port): exemodel::poller()
, m_hwinfo(hwinfo)
, m_svr(port), m_recver(), m_sender()
, m_spec(m_hwinfo)
, m_devs(m_hwinfo)
, m_auxs()
, m_ctx()
, m_sm(m_hwinfo, m_spec, m_devs, m_auxs, m_ctx, m_sender,
	[this](const void * buf, size_t size) -> size_t {
		return m_svr.send(buf, size);
	})
, m_fifo("/dev/jilong/fs_info")
, m_sigterm(SIGUSR1)
{
	log_warning("svcFusionSplicing start .......");
	m_devs.reset(m_spec);
	m_auxs.reset(m_spec);

	/// 1. all callback
	/// 1.1 server
	m_svr.connect([this](exemodel::poller&,
				uint32_t evts,
				exemodel::connectee& conn) {
		if (evts&::EPOLLIN) {
			bool has_msg = m_recver.fill_from(conn);
			if (has_msg) {
				zmsg::mid_t mid = m_recver.id();
				log_debug("svc fs received msg %d", (int)mid);
				if (m_msg_cb[zmsg::to_val(mid)]) {
					m_msg_cb[zmsg::to_val(mid)]();
				}
				else {
					log_warning("svc fs can't handle msg %d", (int)mid);
				}
			}
		}
	});
	this->add(m_svr);

	/// 1.2 motor
	if (m_devs.m_motorLZ) {
		m_devs.m_motorLZ->connect([this](motor&) {
			m_sm.process_event(evMotorStop<motorId_t::LZ>());
		});
		this->add(*m_devs.m_motorLZ);
	}

	if (m_devs.m_motorRZ) {
		m_devs.m_motorRZ->connect([this](motor&) {
			m_sm.process_event(evMotorStop<motorId_t::RZ>());
		});
		this->add(*m_devs.m_motorRZ);
	}

	if (m_devs.m_motorX) {
		m_devs.m_motorX->connect([this](motor&) {
			m_sm.process_event(evMotorStop<motorId_t::X>());
		});
		this->add(*m_devs.m_motorX);
	}

	if (m_devs.m_motorY) {
		m_devs.m_motorY->connect([this](motor&) {
			m_sm.process_event(evMotorStop<motorId_t::Y>());
		});
		this->add(*m_devs.m_motorY);
	}

	/// 1.3 hvb timer
	m_auxs.m_discharge_tmr.connect([this](exemodel::poller&, uint64_t) {
		m_sm.process_event(evDischargeTimeout());
	});
	this->add(m_auxs.m_discharge_tmr);
	m_auxs.m_hvb_pwm.connect([this](exemodel::poller&, uint64_t) {
		m_devs.m_hvb.start();
	}, [this](exemodel::poller&, uint64_t) {
		m_devs.m_hvb.stop();
	}, [this](void) {
		m_devs.m_hvb.stop();
	});
	this->add(m_auxs.m_hvb_pwm);

	/// 1.4 motor back-end switch
	m_devs.m_peLZ.connect([this](exemodel::poller &, uint32_t) {
		do {
			uint16_t tmp;
			auto ret = m_devs.m_peLZ.read(&tmp, sizeof(tmp));
			if (ret < (decltype(ret))sizeof(tmp)) {
				break;
			}
		} while (true);

		if (m_devs.m_peLZ.get_state()) {
			log_debug("motor LZ reach end");
			m_sm.process_event(evMotorBackEnd<motorId_t::LZ>());
		}
	});
	this->add(m_devs.m_peLZ);

	m_devs.m_peRZ.connect([this](exemodel::poller &, uint32_t) {
		do {
			uint16_t tmp;
			auto ret = m_devs.m_peRZ.read(&tmp, sizeof(tmp));
			if (ret < (decltype(ret))sizeof(tmp)) {
				break;
			}
		} while (true);

		if (m_devs.m_peRZ.get_state()) {
			log_debug("motor RZ reach end");
			m_sm.process_event(evMotorBackEnd<motorId_t::RZ>());
		}
	});
	this->add(m_devs.m_peRZ);

	/// 1.5 check fiber exist
	m_auxs.m_push_timeout_tmr.connect([this](exemodel::poller&, uint64_t) {
		m_sm.process_event(evMotorPushTimeout());
	});
	this->add(m_auxs.m_push_timeout_tmr);

	/// 1.10 camera
	this->add(m_devs.m_camera);

	/// 1.11 cover
	m_auxs.m_cover_delay_tmr.connect([this](exemodel::poller &, uint64_t){
			this->m_sm.process_event(evCoverClose());
	});
	this->add(m_auxs.m_cover_delay_tmr);
	m_devs.m_cover.connect([this](exemodel::poller &, uint32_t) {
		do {
			uint16_t tmp;
			auto ret = m_devs.m_cover.read(&tmp, sizeof(tmp));
			if (ret < (decltype(ret))sizeof(tmp)) {
				break;
			}
		} while (true);

		bool is_cover_openned = m_devs.m_cover.get_state();
		log_debug("cover : %s",
				 is_cover_openned ? "openned" : "closed");
		if (is_cover_openned) {
			/// \note stop discharge
			m_devs.m_hvb.stop();

			m_devs.m_ledX.disable();
			m_devs.m_ledY.disable();
			m_devs.m_ledL.enable();
			this->m_auxs.m_cover_delay_tmr.stop();
			m_sm.process_event(evCoverOpen());
		}
		else {
			m_devs.m_ledX.enable();
			m_devs.m_ledY.enable();
			m_devs.m_ledL.disable();
			this->m_auxs.m_cover_delay_tmr.start();
		}

		/// \todo here just use m_sm to report state change,
		/// indeed we should use socket directly without touch state machine
		/// fs_cover_state for app Power Saving Mode
		DCL_ZMSG(fs_cover_state) msg;
		msg.is_openned = is_cover_openned;
		m_sm.report(msg);
	});
	this->add(m_devs.m_cover);

	/// 1.12 cone
	m_auxs.m_cone_waiter.connect([this](exemodel::poller&, uint64_t) {
		m_sm.process_event(evStartCone());
	});
	this->add(m_auxs.m_cone_waiter);

	/// 1.13 motor_test general timer : for pushing , calbrating timeout
	m_auxs.m_mt_tmr.connect([this](exemodel::poller&, uint64_t) {
		m_sm.process_event(evMotorTestTimeout());
	});
	this->add(m_auxs.m_mt_tmr);
	this->add(m_auxs.m_stair_tmr);
	this->add(m_auxs.m_user_tmr);

	/// 2. init state machine
	m_sm.initiate();

	/// 3. init all msg callback
	__init_msgs_cb<
		  zmsg::mid_t::fusion_splice_start
		, zmsg::mid_t::discharge_adjust_start
		, zmsg::mid_t::regular_test_start
		, zmsg::mid_t::motor_test_start
		, zmsg::mid_t::dust_check_start
		, zmsg::mid_t::dust_check1_start
		, zmsg::mid_t::full_dust_check_start
		, zmsg::mid_t::stabilize_electrode_start
		, zmsg::mid_t::shrinkage_test_start
		, zmsg::mid_t::realtime_revise_start
		, zmsg::mid_t::go_on
		, zmsg::mid_t::stop
		, zmsg::mid_t::skip
		, zmsg::mid_t::motor_start
		, zmsg::mid_t::motor_stop
		, zmsg::mid_t::discharge
		, zmsg::mid_t::stop_discharge
		, zmsg::mid_t::image_move
		, zmsg::mid_t::set_led
		//, zmsg::mid_t::set_fs_display_mode
		, zmsg::mid_t::fusion_splice_reset
		, zmsg::mid_t::query_wave_form
		, zmsg::mid_t::manual_focal_distance
		, zmsg::mid_t::get_fiber_defect_info
		, zmsg::mid_t::set_window
	>();

	m_msg_cb[zmsg::to_val(zmsg::mid_t::set_fs_display_mode)] = [this](void) {
		evMsg<zmsg::mid_t::set_fs_display_mode> msg;
		m_recver.convert_to(msg);
		switch (msg.mode) {
		case fs_display_mode_t::X:
			m_devs.m_display.set({false, 1,21,318,198, 0,0,0,0});
			break;
		case fs_display_mode_t::Y:
			m_devs.m_display.set({false, 0,0,0,0, 1,21,318,198});
			break;
		case fs_display_mode_t::TB:
			m_devs.m_display.set({false, 1,21,318,98, 1,121,318,98});
			break;
		case fs_display_mode_t::LR:
			m_devs.m_display.set({false, 1,21,158,198, 161,21,158,198});
			break;
		case fs_display_mode_t::NO:
			m_devs.m_display.set({false, 0,0,0,0, 0,0,0,0});
			break;
		default:
			break;
		}
	};

	m_msg_cb[zmsg::to_val(zmsg::mid_t::set_fs_display_mode_ext)] = [this](void) {
		evMsg<zmsg::mid_t::set_fs_display_mode_ext> msg;
		m_recver.convert_to(msg);
		const disp_data disp_param = {
			msg.order_xtoy,
			msg.x_left, msg.x_up, msg.x_width, msg.x_height,
			msg.y_left, msg.y_up, msg.y_width, msg.y_height,
		};
		m_devs.m_display.set(disp_param);
	};

	m_msg_cb[zmsg::to_val(zmsg::mid_t::set_fs_display_zoom_ext)] = [this](void) {
		evMsg<zmsg::mid_t::set_fs_display_zoom_ext> msg;
		m_recver.convert_to(msg);
		const zoom_data zoom_param = {
			msg.x_left, msg.x_up, msg.x_width, msg.x_height,
			msg.y_left, msg.y_up, msg.y_width, msg.y_height,
		};
		m_devs.m_display.set_zoom(zoom_param);
	};


	m_msg_cb[zmsg::to_val(zmsg::mid_t::set_fs_spec)] = [this](void) {
		evMsg<zmsg::mid_t::set_fs_spec> msg;
		m_recver.convert_to(msg);
		/// \todo maybe we need stop state machine first!
		m_spec.reinit(msg, this->m_hwinfo);

		m_devs.reset(m_spec);
		m_auxs.reset(m_spec);
	};

	m_msg_cb[zmsg::to_val(zmsg::mid_t::discharge_adjust_init)] = [this](void) {
		m_ctx.da.glb_data.init(m_spec.hvb_max_volt);
	};

	/// 4. for debug
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
		if (cmd == "d") {
			if (strs.size() == 2) {
				auto & tmp = strs[1];
				if (tmp == "1") {
					g_log_debug = true;
					log_info("openned debug switch");
					return;
				}
				else if (tmp == "0") {
					g_log_debug= false;
					log_info("closed debug switch");
					return;
				}
			}
		} else if (cmd == "c") {
			if (strs.size() == 2) {
				auto & tmp = strs[1];
				if (tmp == "1") {
					g_log_debug = true;
					log_info("test calibrating precision on");
					return;
				}
				else if (tmp == "0") {
					g_log_debug= false;
					log_info("test calibrating precision off");
					return;
				}
			}
		} else if (cmd == "cover") {
			if (strs.size() == 2) {
				const int delay_ms = std::stoi(strs[idx++]);
				this->m_spec.cover_delay_time = delay_ms;
				this->m_auxs.m_cover_delay_tmr.set_spec({{0, 0}, exemodel::ms_to_timespec(delay_ms)});
			}
		} else if (cmd == "time") {
			if (strs.size() == 2) {
				auto & tmp = strs[idx++];
				if (tmp == "on") {
					g_fs_state_time = true;
					log_info("fs_state spend time on");
					return;
				}
				else if (tmp == "off") {
					g_fs_state_time= false;
					log_info("fs_state spend time off");
					return;
				}
			}
		} else if (cmd == "help") {
				std::cout << "c [0|1]" << std::endl
					<< "|  test calibrating precision" << std::endl
					<< "d [0|1]" << std::endl
					<< "|  log debug message" << std::endl
					<< "cover (1 - 2000)" << std::endl
					<< "|  modify cover close event delay time (uint : ms)" << std::endl;

				return;
		}

		return;
	});
	this->add(m_fifo);

	/// 5. for shutdown
	m_sigterm.connect([this]() {
		throw exemodel::exec_stop("service fs exit");
	});
	this->add(m_sigterm);
}

svcFusionSplicing::~svcFusionSplicing()
{
	m_devs.stop();
	m_devs.disable();

	this->del(m_sigterm);
	this->del(m_fifo);

	this->del(m_auxs.m_user_tmr);
	this->del(m_auxs.m_stair_tmr);
	this->del(m_auxs.m_mt_tmr);
	this->del(m_auxs.m_cone_waiter);
	this->del(m_devs.m_cover);
	this->del(m_devs.m_camera);
	this->del(m_devs.m_peRZ);
	this->del(m_devs.m_peLZ);
	this->del(m_auxs.m_hvb_pwm);
	this->del(m_auxs.m_discharge_tmr);

	if (m_devs.m_motorY) {
		this->del(*m_devs.m_motorY);
	}

	if (m_devs.m_motorX) {
		this->del(*m_devs.m_motorX);
	}

	if (m_devs.m_motorRZ) {
		this->del(*m_devs.m_motorRZ);
	}

	if (m_devs.m_motorLZ) {
		this->del(*m_devs.m_motorLZ);
	}

	this->del(m_svr);
}

}
