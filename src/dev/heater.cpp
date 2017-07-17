#include <unistd.h>

#include <string>
#include <stdexcept>
#include <array>

#include <exemodel/poll_tools.hpp>

#include "device/heater.hpp"
#include "heater_adc2temp.hpp"

using namespace std;

heater::heater(const char * slow_path,
	       const char * fast_path,
	       const char * temp_path)
: m_heater1(slow_path)
, m_heater2(fast_path)
, temp_fd(exemodel::validate_fd(::open(temp_path, O_RDONLY), temp_path))
{
	m_heater1.set_ratio(1.0);
	m_heater2.set_ratio(1.0);
}

heater::~heater()
{
	::close(temp_fd);
}

int heater::slow_enable()
{
	m_heater1.enable();
	return 0;
}

int heater::fast_enable()
{
	m_heater2.enable();
	return 0;
}

int heater::slow_disable()
{
	m_heater1.disable();
	return 0;
}

int heater::fast_disable()
{
	m_heater2.disable();
	return 0;
}

static constexpr double temp_max = 999.99;
static constexpr double temp_min = -999.99;
const std::array<double, 4096> Adc2Temp::s_adc2temp_map{{
	#include "data/temp-map"
}};
double heater::get_temp()
{
	char val[8] = "";
	int r = pread(temp_fd, val, sizeof(val), 0);
	exemodel::validate_ret(r, "get heater temperature error!");

	int x = std::stoi(val);
	Adc2Temp adc2temp;

	return adc2temp(x);
}
