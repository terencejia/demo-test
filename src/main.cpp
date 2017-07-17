#include <thread>
#include <functional>
#include <csignal>

#include <stdlib.h>
#include <unistd.h>

#include <exemodel/poller.hpp>
#include <exemodel/signalee.hpp>

#include "hw_info.hpp"

#include "srv/srv_heating.hpp"
#include "srv/srv_fusion_splicing.hpp"
#include "srv/srv_dev_mng.hpp"

static std::function<void(void)> s_sigterm_hdl;
void signal_handler(int /*sig*/)
{
	s_sigterm_hdl();
}

int main(/*int argc, char* argv[]*/)
{
	system("mkfifo /dev/jilong/fs_info");
	system("mkfifo /dev/jilong/heat_info");

	const hw_info hwinfo = get_hw_info();

	/*start fusion splice service*/
	std::thread th1([&hwinfo](void){
		exemodel::mask_signal(SIGUSR1);
		std::unique_ptr<svcFS::svcFusionSplicing> fs_svr(new svcFS::svcFusionSplicing(hwinfo, 5901));
		fs_svr->run();
	});

	/*start heat service*/
	std::thread th2([&hwinfo](void){
		exemodel::mask_signal(SIGUSR1);
		std::unique_ptr<svcHeat::svcHeating> heat_svr(new svcHeat::svcHeating(hwinfo, 5902));
		heat_svr->run();
	});

	/*start device manage service*/
	std::thread th3([&hwinfo](void){
		exemodel::mask_signal(SIGUSR1);
		std::unique_ptr<svcDevMng::svcDeviceManager> devmng_svr(new svcDevMng::svcDeviceManager(hwinfo, 5903));
		devmng_svr->run();
	});

	s_sigterm_hdl = [&th1, &th2, &th3](void) {
		::pthread_kill(th1.native_handle(), SIGUSR1);
		::pthread_kill(th2.native_handle(), SIGUSR1);
		::pthread_kill(th3.native_handle(), SIGUSR1);
	};

	std::signal(SIGTERM, signal_handler);

	th1.join();
	th2.join();
	th3.join();

	/// \todo we can't remove it, if you know why, tell me!
	system("rm /dev/jilong/fs_info");
	system("rm /dev/jilong/heat_info");

	return 0;
}
