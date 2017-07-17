#pragma once

#include <memory>

#include <exemodel/timeree.hpp>
#include <exemodel/soft_pwm.hpp>
#include <exemodel/dev_attr.hpp>

#include "fsconf.h"

#include "device/camera.hpp"
#include "device/hvb.hpp"
#include "device/motor.hpp"
#include "device/pe_switch.hpp"
#include "device/led.hpp"
#include "device/hall_switch.hpp"
#include "device/display.hpp"
#include "device/camera_set.hpp"

#include "hw_info.hpp"

#include "fs_spec.hpp"

namespace svcFS {

struct fs_devs final {
	fs_devs(const hw_info & hwinfo);
	~fs_devs();

	void reset(const fs_spec & spec);

	void enable(void);
	void disable(void);
	void stop(void);

	const hw_info & m_hwinfo;

	camera_set	m_camera;
	led	m_ledX;		/// X camera led
	led	m_ledY;		/// Y camera led
	led	m_ledL;		/// Light led

	std::unique_ptr< motor >	m_motorLZ;	/// left Z
	std::unique_ptr< motor >	m_motorRZ;	/// right Z
	std::unique_ptr< motor >	m_motorX;	/// X
	std::unique_ptr< motor >	m_motorY;	/// Y
	hvb	m_hvb;

	/// \note if the state of pe switch is 1, it represents reach the end.
	pe_switch m_peLZ;	/// motor LZ's pe switch
	pe_switch m_peRZ;	/// motor RZ's pe switch

	/// \note if the state of hall switch is 1, it represents cover closed.
	hall_switch m_cover;

	display	m_display;
	std::unique_ptr< exemodel::dev_attr_nor_rw<long> > m_x_focus;
	std::unique_ptr< exemodel::dev_attr_nor_rw<long> > m_y_focus;

	std::unique_ptr< exemodel::dev_attr_rw<long> > m_x_exposure;
	std::unique_ptr< exemodel::dev_attr_rw<long> > m_y_exposure;

	/// env	\todo maybe we should use 'svcDeviceManager' to get env info.
	exemodel::dev_attr_adv_ro<long, 1000> m_env_temp;
	exemodel::dev_attr_adv_ro<long, 100000> m_env_pressure;
};

struct fs_auxs final {
	fs_auxs();

	void reset(const fs_spec & spec);
	void stop(void);

	/// discharge-related timer
	exemodel::monotonic_timeree m_discharge_tmr;
	exemodel::soft_pwm m_hvb_pwm;

	/// cone-fusion-splicing -related timer
	exemodel::monotonic_timeree m_cone_waiter;

	/// motor test : general timer
	exemodel::monotonic_timeree m_mt_tmr;

	exemodel::monotonic_timeree m_push_timeout_tmr;

	///stair timer
	exemodel::monotonic_timeree m_stair_tmr;
	///user puporse timer
	exemodel::monotonic_timeree m_user_tmr;
	///\ cover delay timer
	exemodel::monotonic_timeree m_cover_delay_tmr;
};

}
