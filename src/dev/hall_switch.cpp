#include <exemodel/poll_tools.hpp>

#include <plf_drv/hall_switch_dev.h>
#include "device/hall_switch.hpp"

hall_switch::hall_switch(const char * path)
: exemodel::devicee(path)
{
#if 0
	int ret;
	ret = io_ctl(LDB_IOC_ENABLE_INT);
	exemodel::validate_ret(ret, "enable hall_switch interrupt");
#endif
}

hall_switch::~hall_switch()
{
}

bool hall_switch::get_state()
{
	uint16_t state;

	int tmp = this->io_ctl(HALL_SWITCH_IOC_GET_STATE, &state);
	exemodel::validate_ret(tmp, "get hall_switch state");

	return (state > 0);
}

