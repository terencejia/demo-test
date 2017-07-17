#include <cmath>

#include <exemodel/poll_tools.hpp>
#include <fs_log.hpp>
#include <plf_drv/motor_dev.h>

#include "device/motor.hpp"

motor::motor(const char *path, const double speed_ratio)
: poller()
, m_max_speed(0x0000)
, m_min_speed(0xFFFF)
, m_piror_dir(go_forward)
, m_speed_ratio(speed_ratio)
, m_motor(path)
, m_tmr({ {0,100*1000*1000}, {0,100*1000*1000} })
#ifdef CFG_MOTOR_VAR_SPEED
, m_vspeed_cnt(0)
, m_vspeed_tmr({ {0,100*1000*1000}, {0,100*1000*1000} })
#endif
{
	this->add(m_motor);
	this->add(m_tmr);
#ifdef CFG_MOTOR_VAR_SPEED
	m_vspeed_tmr.connect([this](exemodel::poller&, uint64_t) -> void {
		if (m_vspeed_cnt < 50) {
			++m_vspeed_cnt;
			double speed = std::pow(m_vspeed_cnt / 50.0, 2);
			this->set_speed(speed);
		}
		else {
			/// reach highest speed, don't need timer
			m_vspeed_tmr.stop();
		}
	});
	this->add(m_vspeed_tmr);
#endif
}

motor::~motor()
{
}

void motor::connect(cb_func_t const func)
{
	m_motor.connect([this, func](exemodel::poller&, uint32_t) -> void {
		do {
			uint16_t tmp;
			auto ret = m_motor.read(&tmp, sizeof(tmp));
			if (ret < (decltype(ret))sizeof(tmp)) {
				break;
			}
		} while (true);

		if (this->is_stopped()) {
			func(*this);
			m_tmr.stop();
#ifdef CFG_MOTOR_VAR_SPEED
			m_vspeed_tmr.stop();
#endif
		}
	});
	m_tmr.connect([this, func](exemodel::poller&, uint64_t) -> void {
		if (this->is_stopped()) {
			log_debug("%s stopped state is found by timer", m_motor.path());
			func(*this);
			m_tmr.stop();
#ifdef CFG_MOTOR_VAR_SPEED
			m_vspeed_tmr.stop();
#endif
		}
	});
}

void motor::set_speed_limit(uint16_t max_speed, uint16_t min_speed)
{
	if (min_speed < max_speed) {
		throw std::invalid_argument("motor speed max/min don't match");
	}

	m_max_speed = max_speed;
	m_min_speed = min_speed;
}

void motor::enable()
{
	int ret;
	ret = m_motor.io_ctl(MOTOR_IOC_ENABLE);
	exemodel::validate_ret(ret,"enable motor");

	return;
}

void motor::disable()
{
	int ret;
	ret = m_motor.io_ctl(MOTOR_IOC_DISABLE);
	exemodel::validate_ret(ret,"disable motor");

	return;
}

void motor::start_by_ss(
	double speed,
	uint32_t step,
	bool back_dir)
{
	if (step == 0 || MAX_STEP_NUM < step) {
		throw std::invalid_argument("motor step invalid!");
	}

	start_ll(step_factor_t::N1_8, __clamp_speed(speed), static_cast<uint16_t>(step), back_dir);
	return;
}

uint16_t motor::__clamp_speed(double speed) const
{
	if (speed < 0.0 || speed > 1.0) {
		throw std::invalid_argument("motor speed invalid!");
	}

	const double raw_speed = (speed < 0.001 ? m_min_speed : (m_max_speed / speed));

	if (raw_speed < m_max_speed) {
		return m_max_speed;
	}

	if (m_min_speed < raw_speed) {
		return m_min_speed;
	}

	return static_cast<uint16_t>(raw_speed);
}

void motor::start_ll(
	step_factor_t step_factor,
	uint16_t speed,
	uint16_t step_num,
	bool back_dir)
{
	if (this->is_running()) {
		log_warning("%s is already running", m_motor.path());
		this->stop();
	}

	int ret;

	m_piror_dir = back_dir;
	uint16_t dir = back_dir ? 1 : 0;
	ret = m_motor.io_ctl(MOTOR_IOC_SET_DIR, &dir);
	exemodel::validate_ret(ret, "set motor direction");
	ret = m_motor.io_ctl(MOTOR_IOC_SET_SPEED, &speed);
	exemodel::validate_ret(ret, "set motor speed");
	ret = m_motor.io_ctl(MOTOR_IOC_SET_STEP_FACTOR, &step_factor);
	exemodel::validate_ret(ret, "set motor step factor");
	ret = m_motor.io_ctl(MOTOR_IOC_SET_STEP_NUM, &step_num);
	exemodel::validate_ret(ret, "set motor step num");

	ret = m_motor.io_ctl(MOTOR_IOC_START);
	exemodel::validate_ret(ret, "start motor");

	/// \note start the timer to check if stopped.
	m_tmr.start();

	return;
}

void motor::stop()
{
	int ret = m_motor.io_ctl(MOTOR_IOC_STOP);
	exemodel::validate_ret(ret, "stop motor");
#ifdef CFG_MOTOR_VAR_SPEED
	m_vspeed_tmr.stop();
#endif
	return;
}

bool motor::is_stopped()
{
	uint16_t state;

	int tmp = m_motor.io_ctl(MOTOR_IOC_GET_STATE, &state);
	exemodel::validate_ret(tmp, "get motor state");

	return (state ? true : false);
}

void motor::set_speed(double speed)
{
	const uint16_t raw_speed = __clamp_speed(speed);

	int ret = m_motor.io_ctl(MOTOR_IOC_SET_SPEED, &raw_speed);
	exemodel::validate_ret(ret, "set motor speed");
}

void motor::start_by_var_speed(bool back_dir)
{
	const double speed = 0.005;
	this->start_by_speed(speed, back_dir);
#ifdef CFG_MOTOR_VAR_SPEED
	m_vspeed_cnt = 0;
	m_vspeed_tmr.start();
#endif
}
