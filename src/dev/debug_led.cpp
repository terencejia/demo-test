
#include "device/debug_led.hpp"

using namespace std;

debug_led::debug_led(const uint8_t gpio_num)
:m_led(new gpiodev(gpio_num))
{
}

debug_led::~debug_led()
{
	delete m_led;
}

int debug_led::enable()
{
	m_led->set_val(0);
	return 0;
}

int debug_led::disable()
{
	m_led->set_val(1);
	return 0;
}
