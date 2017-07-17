#include <unistd.h>

#include <exemodel/poll_tools.hpp>

#include "device/gpio_ctl.hpp"

#define GPIO_NBR(_x) #_x
#define GPIO_DIR_FN(_x) "/sys/class/gpio/gpio"#_x"/direction"
#define GPIO_VAL_FN(_x) "/sys/class/gpio/gpio"#_x"/value"

#define GPIO_INFO(_x) { \
	#_x, \
	sizeof(#_x), \
	"/sys/class/gpio/gpio"#_x"/direction", \
	"/sys/class/gpio/gpio"#_x"/value", \
}

static const struct {
	const char * nbr_str;
	size_t nbr_str_size;
	const char * dir_fn;
	const char * val_fn;
} s_gpio_info[256] = {
	GPIO_INFO(0),	GPIO_INFO(1),	GPIO_INFO(2),	GPIO_INFO(3),	GPIO_INFO(4),	GPIO_INFO(5),	GPIO_INFO(6),	GPIO_INFO(7),	GPIO_INFO(8),	GPIO_INFO(9),
	GPIO_INFO(10),	GPIO_INFO(11),	GPIO_INFO(12),	GPIO_INFO(13),	GPIO_INFO(14),	GPIO_INFO(15),	GPIO_INFO(16),	GPIO_INFO(17),	GPIO_INFO(18),	GPIO_INFO(19),
	GPIO_INFO(20),	GPIO_INFO(21),	GPIO_INFO(22),	GPIO_INFO(23),	GPIO_INFO(24),	GPIO_INFO(25),	GPIO_INFO(26),	GPIO_INFO(27),	GPIO_INFO(28),	GPIO_INFO(29),
	GPIO_INFO(30),	GPIO_INFO(31),	GPIO_INFO(32),	GPIO_INFO(33),	GPIO_INFO(34),	GPIO_INFO(35),	GPIO_INFO(36),	GPIO_INFO(37),	GPIO_INFO(38),	GPIO_INFO(39),
	GPIO_INFO(40),	GPIO_INFO(41),	GPIO_INFO(42),	GPIO_INFO(43),	GPIO_INFO(44),	GPIO_INFO(45),	GPIO_INFO(46),	GPIO_INFO(47),	GPIO_INFO(48),	GPIO_INFO(49),
	GPIO_INFO(50),	GPIO_INFO(51),	GPIO_INFO(52),	GPIO_INFO(53),	GPIO_INFO(54),	GPIO_INFO(55),	GPIO_INFO(56),	GPIO_INFO(57),	GPIO_INFO(58),	GPIO_INFO(59),
	GPIO_INFO(60),	GPIO_INFO(61),	GPIO_INFO(62),	GPIO_INFO(63),	GPIO_INFO(64),	GPIO_INFO(65),	GPIO_INFO(66),	GPIO_INFO(67),	GPIO_INFO(68),	GPIO_INFO(69),
	GPIO_INFO(70),	GPIO_INFO(71),	GPIO_INFO(72),	GPIO_INFO(73),	GPIO_INFO(74),	GPIO_INFO(75),	GPIO_INFO(76),	GPIO_INFO(77),	GPIO_INFO(78),	GPIO_INFO(79),
	GPIO_INFO(80),	GPIO_INFO(81),	GPIO_INFO(82),	GPIO_INFO(83),	GPIO_INFO(84),	GPIO_INFO(85),	GPIO_INFO(86),	GPIO_INFO(87),	GPIO_INFO(88),	GPIO_INFO(89),
	GPIO_INFO(90),	GPIO_INFO(91),	GPIO_INFO(92),	GPIO_INFO(93),	GPIO_INFO(94),	GPIO_INFO(95),	GPIO_INFO(96),	GPIO_INFO(97),	GPIO_INFO(98),	GPIO_INFO(99),
	GPIO_INFO(100),	GPIO_INFO(101),	GPIO_INFO(102),	GPIO_INFO(103),	GPIO_INFO(104),	GPIO_INFO(105),	GPIO_INFO(106),	GPIO_INFO(107),	GPIO_INFO(108),	GPIO_INFO(109),
	GPIO_INFO(110),	GPIO_INFO(111),	GPIO_INFO(112),	GPIO_INFO(113),	GPIO_INFO(114),	GPIO_INFO(115),	GPIO_INFO(116),	GPIO_INFO(117),	GPIO_INFO(118),	GPIO_INFO(119),
	GPIO_INFO(120),	GPIO_INFO(121),	GPIO_INFO(122),	GPIO_INFO(123),	GPIO_INFO(124),	GPIO_INFO(125),	GPIO_INFO(126),	GPIO_INFO(127),	GPIO_INFO(128),	GPIO_INFO(129),
	GPIO_INFO(130),	GPIO_INFO(131),	GPIO_INFO(132),	GPIO_INFO(133),	GPIO_INFO(134),	GPIO_INFO(135),	GPIO_INFO(136),	GPIO_INFO(137),	GPIO_INFO(138),	GPIO_INFO(139),
	GPIO_INFO(140),	GPIO_INFO(141),	GPIO_INFO(142),	GPIO_INFO(143),	GPIO_INFO(144),	GPIO_INFO(145),	GPIO_INFO(146),	GPIO_INFO(147),	GPIO_INFO(148),	GPIO_INFO(149),
	GPIO_INFO(150),	GPIO_INFO(151),	GPIO_INFO(152),	GPIO_INFO(153),	GPIO_INFO(154),	GPIO_INFO(155),	GPIO_INFO(156),	GPIO_INFO(157),	GPIO_INFO(158),	GPIO_INFO(159),
	GPIO_INFO(160),	GPIO_INFO(161),	GPIO_INFO(162),	GPIO_INFO(163),	GPIO_INFO(164),	GPIO_INFO(165),	GPIO_INFO(166),	GPIO_INFO(167),	GPIO_INFO(168),	GPIO_INFO(169),
	GPIO_INFO(170),	GPIO_INFO(171),	GPIO_INFO(172),	GPIO_INFO(173),	GPIO_INFO(174),	GPIO_INFO(175),	GPIO_INFO(176),	GPIO_INFO(177),	GPIO_INFO(178),	GPIO_INFO(179),
	GPIO_INFO(180),	GPIO_INFO(181),	GPIO_INFO(182),	GPIO_INFO(183),	GPIO_INFO(184),	GPIO_INFO(185),	GPIO_INFO(186),	GPIO_INFO(187),	GPIO_INFO(188),	GPIO_INFO(189),
	GPIO_INFO(190),	GPIO_INFO(191),	GPIO_INFO(192),	GPIO_INFO(193),	GPIO_INFO(194),	GPIO_INFO(195),	GPIO_INFO(196),	GPIO_INFO(197),	GPIO_INFO(198),	GPIO_INFO(199),
	GPIO_INFO(200),	GPIO_INFO(201),	GPIO_INFO(202),	GPIO_INFO(203),	GPIO_INFO(204),	GPIO_INFO(205),	GPIO_INFO(206),	GPIO_INFO(207),	GPIO_INFO(208),	GPIO_INFO(209),
	GPIO_INFO(210),	GPIO_INFO(211),	GPIO_INFO(212),	GPIO_INFO(213),	GPIO_INFO(214),	GPIO_INFO(215),	GPIO_INFO(216),	GPIO_INFO(217),	GPIO_INFO(218),	GPIO_INFO(219),
	GPIO_INFO(220),	GPIO_INFO(221),	GPIO_INFO(222),	GPIO_INFO(223),	GPIO_INFO(224),	GPIO_INFO(225),	GPIO_INFO(226),	GPIO_INFO(227),	GPIO_INFO(228),	GPIO_INFO(229),
	GPIO_INFO(230),	GPIO_INFO(231),	GPIO_INFO(232),	GPIO_INFO(233),	GPIO_INFO(234),	GPIO_INFO(235),	GPIO_INFO(236),	GPIO_INFO(237),	GPIO_INFO(238),	GPIO_INFO(239),
	GPIO_INFO(240),	GPIO_INFO(241),	GPIO_INFO(242),	GPIO_INFO(243),	GPIO_INFO(244),	GPIO_INFO(245),	GPIO_INFO(246),	GPIO_INFO(247),	GPIO_INFO(248),	GPIO_INFO(249),
	GPIO_INFO(250),	GPIO_INFO(251),	GPIO_INFO(252),	GPIO_INFO(253),	GPIO_INFO(254),	GPIO_INFO(255),
};

gpiodev::gpiodev(uint8_t nbr, bool is_out, uint8_t val)
: m_nbr(__export(nbr))
, m_dir(s_gpio_info[nbr].dir_fn)
, m_val(s_gpio_info[nbr].val_fn)
{
	set_dir(is_out);
	if (is_out) {
		set_val(val);
	}
}

gpiodev::~gpiodev()
{
	__unexport(m_nbr);
}

uint8_t gpiodev::__export(uint8_t nbr)
{
	std::string gpiodir = "/sys/class/gpio/gpio" + std::to_string((int)nbr);
	if (::access(gpiodir.c_str(), F_OK) == 0) {
		return -1;
	}

	exemodel::dev_attr_wo<int> exportgpio("/sys/class/gpio/export");
	exportgpio.write((int)nbr);

	return nbr;
}

void gpiodev::__unexport(uint8_t nbr)
{
	std::string gpiodir = "/sys/class/gpio/gpio" + std::to_string((int)nbr);
	if (::access(gpiodir.c_str(), F_OK) == -1) {
		return;
	}

	exemodel::dev_attr_wo<int> unexportgpio("/sys/class/gpio/unexport");
	unexportgpio.write((int)nbr);

	return;
}

void gpiodev::set_dir(bool is_out)
{
	m_dir.write(is_out ? "out" : "in");

        return;
}

void gpiodev::set_val(uint8_t val)
{
	m_val.write(val ? 1 : 0);

	return;
}

uint8_t gpiodev::read_val()
{
	int val = m_val.read();

	return (val ? 1 : 0);
}

uint8_t gpiodev::get_nbr() const
{
	return m_nbr;
}
