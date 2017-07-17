#include <exemodel/poll_tools.hpp>

#include <plf_drv/pe_switch_dev.h>
#include "device/pe_switch.hpp"

pe_switch::pe_switch(const char * path)
: exemodel::devicee(path)
{
#if 0
	int ret;
	ret = io_ctl(LDB_IOC_ENABLE_INT);
	exemodel::validate_ret(ret, "enable pe_switch interrupt");
#endif

}

pe_switch::~pe_switch()
{
}

bool pe_switch::get_state()
{
	uint16_t state;

	int tmp = this->io_ctl(PE_SWITCH_IOC_GET_STATE, &state);
	exemodel::validate_ret(tmp, "get pe_switch state");

	return (state > 0);
}
