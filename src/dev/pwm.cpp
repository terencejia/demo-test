#include <exemodel/poll_tools.hpp>
#include <fs_log.hpp>

#include <plf_drv/pwm_dev.h>
#include "device/pwm.hpp"

pwm::pwm(const char* path)
: m_f(path)
{
}

pwm::~pwm()
{
}

void pwm::enable()
{
	int ret = m_f.io_ctl(PWM_IOC_ENABLE);
	exemodel::validate_ret(ret, "enable pwm");
	return;
}

void pwm::disable()
{
	int ret = m_f.io_ctl(PWM_IOC_DISABLE);
	exemodel::validate_ret(ret, "disable pwm");
	return;
}

int pwm::get_frq()
{
	uint16_t frq;

	int tmp = m_f.io_ctl(PWM_IOC_GET_FREQUENCY, &frq);
	exemodel::validate_ret(tmp, "get pwm frequency");
	return frq;
}

int pwm::set_frq(uint16_t frq)
{
	int ret;

	ret = m_f.io_ctl(PWM_IOC_SET_FREQUENCY, &frq);
	exemodel::validate_ret(ret, "pwm set frequency");
	return ret;
}

double pwm::get_ratio()
{
	int ret;

	uint16_t frq;
	ret = m_f.io_ctl(PWM_IOC_GET_FREQUENCY, &frq);
	exemodel::validate_ret(ret, "get pwm frequency");

	uint16_t ratio;
	ret = m_f.io_ctl(PWM_IOC_GET_RATIO, &ratio);
	exemodel::validate_ret(ret, "pwm get ratio");

	if (ratio >= frq) {
		return 1.0;
	}
	else {
		/// \note in this branch, the 'frq' should not be '0'.
		return (static_cast<double>(ratio) / frq);
	}
}

void pwm::set_ratio(double val)
{
	int ret;

	uint16_t frq;
	ret = m_f.io_ctl(PWM_IOC_GET_FREQUENCY, &frq);
	exemodel::validate_ret(ret, "get pwm frequency");

	uint16_t ratio;
	if (val >= 1.0) {
		ratio = frq;
	}
	else if (val <= 0) {
		ratio = 0x0;
	}
	else {
		ratio = static_cast<uint16_t>(val * frq);
	}

	//log_info("set pwm ratio value to %d", (int)ratio);
	ret = m_f.io_ctl(PWM_IOC_SET_RATIO, &ratio);
	exemodel::validate_ret(ret, "pwm set ratio");

	return;
}
