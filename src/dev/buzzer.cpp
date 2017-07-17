#include "device/buzzer.hpp"

buzzer::buzzer(const uint8_t gpio_num)
:m_beep(new gpiodev(gpio_num))
{
}

buzzer::~buzzer()
{
	m_beep->set_val(0);
	delete m_beep;
}

int buzzer::enable()
{
	m_beep->set_val(1);
	return 0;
}

int buzzer::disable()
{
	m_beep->set_val(0);
	return 0;
}
