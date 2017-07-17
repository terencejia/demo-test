
#include <fstream>
#include <string>

#include "fsconf.h"
#include "hw_info.hpp"
#include "fs_log.hpp"

static const hw_info kl350_an033_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0068/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0068/exposure",
	0, 400,

	0, 65535,
	320, 240,
	720, 480,
	6.35, 7.05,
	1380, 1380,
	3000,

	7,
	"an033",

	1.0,
};

static const hw_info kl350_ov9712_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0030/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0030/exposure",
	0, 400,

	0, 65535,
	320, 240,
	640, 400,
	3*2, 3*2,	/// subsample 2x2
	830*2, 830*2,
	3000,

	8,
	"ov9712",

	1.0,
};

static const hw_info kl350_ov9121_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0030/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0030/exposure",
	0, 400,

	0, 2047,
	320, 240,
	1280, 1024,
	5, 5,
	1380, 1380,
	3000,

	10,
	"ov9121",

	1.0,
};

static const hw_info kl360_model_info = {
	"/sys/class/i2c-dev/i2c-0/device/0-0023/focus",
	"/sys/class/i2c-dev/i2c-1/device/1-0023/focus",
	255,
	255,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	nullptr,
	0, 0,
	nullptr,
	0, 0,

	0, 2047,
	320, 240,
	1280, 1024,
	5, 5,
	1380, 1380,
	3000,

	5,
	"ov9121",

	1.0,
};

static const hw_info kl500_ov9121_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0030/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0030/exposure",
	0, 400,

	0, 2047,
	320, 240,
	1280, 1024,
	5, 5,
	2100, 2100,
	3000,

	5,
	"ov9121",

	1.0,
};

static const hw_info kl500_ov9712_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0030/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0030/exposure",
	0, 400,

	0, 2047,
	320, 240,
	640, 400,
	3*2, 3*2,
	2400, 2400,
	3000,

	5,
	"ov9712",

	1.0,
};

static const hw_info kl510_ov9121_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0030/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0030/exposure",
	0, 400,

	0, 2047,
	320, 240,
	1280, 1024,
	5, 5,
	2100, 2100,
	3000,

	5,
	"ov9121",

	1.0,
};
static const hw_info kl510_ov9712_model_info = {
	nullptr,
	nullptr,
	0,
	0,
	"/sys/class/i2c-dev/i2c-0/device/0-0028/pressure_input",
	nullptr,
	"/sys/class/hwmon/hwmon2/temp1_input",
	"/sys/class/hwmon/hwmon2/temp2_input",
	"/sys/devices/platform/s3c64xx-adc/s3c-hwmon/adc0_raw",
	"/sys/class/backlight/pwm-backlight/brightness",
	"/dev/jilong/cameractl",
	"/dev/jilong/ledX",
	"/dev/jilong/ledY",
	"/dev/jilong/ledL",
	"/dev/jilong/motorLZ",
	"/dev/jilong/motorRZ",
	"/dev/jilong/motorX",
	"/dev/jilong/motorY",
	"/sys/class/spi_master/spi1/spi1.0/resistor",
	"/dev/jilong/hvbSwitch",
	"/dev/jilong/endLZ",
	"/dev/jilong/endRZ",
	"/dev/jilong/cover",
	"/dev/jilong/display",
	"/dev/jilong/heaterCover",
	"/dev/jilong/heaterSlow",
	"/dev/jilong/heaterFast",
	"/sys/devices/platform/leds-gpio/leds/heaterRedLed/brightness",
	"/sys/devices/platform/leds-gpio/leds/heaterGreenLed/brightness",
	nullptr,	/* use fpga to adjust window, avoid tearing */
	nullptr,

	"/sys/devices/platform/s3c2440-i2c.0/i2c-0/0-0030/exposure",
	0, 400,
	"/sys/devices/platform/s3c2440-i2c.1/i2c-1/1-0030/exposure",
	0, 400,

	0, 2047,
	320, 240,
	640, 400,
	3*2, 3*2,
	2400, 2400,
	3000,

	5,
	"ov9712",

	1.0,
};

const hw_info get_hw_info(void)
{
	try {
		hw_info ret;

		std::ifstream model_info_file("/proc/modelinfo");
		std::string model_str;
		std::string hw_cfg;

		std::getline(model_info_file, model_str);
		std::getline(model_info_file, hw_cfg);

		log_info("model: %s", model_str.c_str());
		log_info("hwcfg: %s", hw_cfg.c_str());

		if (model_str == "kl350") {
			switch (hw_cfg[3]) {
				case 'O':
					ret = kl350_ov9121_model_info;
					break;
				case '7':
					ret = kl350_ov9712_model_info;
					break;
				case 'A':
					ret = kl350_an033_model_info;
					break;
				default:
					ret = kl350_ov9121_model_info;
					break;
			}
		}
		else if (model_str == "kl350c") {
			switch (hw_cfg[3]) {
				case 'O':
					ret = kl350_ov9121_model_info;
					break;
				default:
					ret = kl350_ov9121_model_info;
					break;
			}
		}
		else if (model_str == "kl360") {
			ret = kl360_model_info;
		}
		else if (model_str == "kl500") {
				switch (hw_cfg[3]) {
				case 'O':
					ret = kl500_ov9121_model_info;
					break;
				case '7':
					ret = kl500_ov9712_model_info;
					break;
				default:
					ret = kl500_ov9712_model_info;
					break;
			}
		}
		else if (model_str == "kl510") {
			switch (hw_cfg[3]) {
				case 'O':
					ret = kl510_ov9121_model_info;
					break;
				case '7':
					ret = kl510_ov9712_model_info;
					break;
				default:
					ret = kl510_ov9712_model_info;
					break;
			}
		}
		else {
			ret = kl350_ov9712_model_info;
		}

		/* cmos capture win
		 * hwcfg[2] lcd display resolution
		 * hwcfg[6] user config cmos capture window
		 * if hwcfg[6] == '-', use hwcfg[2] as default
		 */
		const auto cmos_cpt_win = (hw_cfg[6] == '_' ? hw_cfg[2] : hw_cfg[6]);
		switch (cmos_cpt_win) {
		case 'Q':
			ret.cmos_win_width = 320;
			ret.cmos_win_height = 240;
			break;
		case 'W':
			ret.cmos_win_width = 480;
			ret.cmos_win_height = 272;
			break;
		case 'V':
			ret.cmos_win_width = 640;
			ret.cmos_win_height = 480;
			break;
		default:
			break;
		}

		/// hvb
		switch (hw_cfg[4]) {
		case '3':
			ret.hvb_alti = 3000;
			break;
		case '5':
			ret.hvb_alti = 5000;
			break;
		default:
			ret.hvb_alti = 3000;
			break;
		}

		/// push motor
		switch (hw_cfg[5]) {
		case '5':
			ret.push_motor_speed_ratio = 5.0;
			break;
		case '1':
			ret.push_motor_speed_ratio = 1.0;
		default:
			break;
		}

		/// lens
		switch (hw_cfg[7]) {
		case 'C':
			ret.xz_nm_per_pixel = ret.yz_nm_per_pixel = ret.cmos_pixel_width * 1250.0 / 6.0;
			break;
		case 'B':
			ret.xz_nm_per_pixel = ret.yz_nm_per_pixel = ret.cmos_pixel_width * 1640.0 / 6.0;
			break;
		case 'A':
			ret.xz_nm_per_pixel = ret.yz_nm_per_pixel = ret.cmos_pixel_width * 2400.0 / 6.0;
			break;
		default:
			break;
		};

		ret.zdist_clr_tolerance = static_cast<int>(15000/ret.xz_nm_per_pixel);
		return ret;
	}
	catch (...) {
		return kl350_ov9712_model_info;
	}
}
