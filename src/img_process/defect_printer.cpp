#include <type_traits>
#include <iostream>
#include <fs_log.hpp>

#include "img_process.hpp"

namespace img_process {

static const struct {
	ifd_t d;
	const char * n;
	bool is_valid;
} s_info[] = {
#if 0
	{ ifd_spot,		"   spot  ", },
	{ ifd_break,		"  break  ", },
	{ ifd_bottom_sag,	"  b_sag  ", },
	{ ifd_bottom_crest,	" b_crest ", },
	{ ifd_bottom_corner,	" b_corner", },
	{ ifd_top_sag,		"  t_sag  ", },
	{ ifd_top_crest,	" t_crest ", },
	{ ifd_top_corner,	" t_corner", },
#endif
	{ ifd_end_crude,	"   end   ", true, },
	{ ifd_horizontal_angle,	" h_angle ", true, },
	{ ifd_vertical_angle,	" v_angle ", true, },
	{ ifd_cant_identify,	" identify", true, },
};

static void print_info_header(void)
{
	log_debug("     ");
	for (auto & i : s_info) {
		log_debug("%s", i.n);
	}
	log_debug("\n");
}

static void print_line(const char * h, const ifd_line & line_fd)
{
	char tmp[sizeof(s_info)/sizeof(s_info[0])];
	char * ptmp = tmp;
	for (auto & i : s_info) {
		if (!i.is_valid) {
			log_debug("    -    ");
			continue;
		}

		if (line_fd.dbmp & i.d) {
			*ptmp = 'x';
		}
		else {
			*ptmp = 'o';
		}
		++ptmp;
	}
	log_debug("%s: %c %c %c %c", h, tmp[0], tmp[1], tmp[2], tmp[3]);

	log_debug("    hangle(%2.2f), vangle(%2.2f)\n",
	       line_fd.h_angle, line_fd.v_angle);
}

void print_defects(const img_defects_t & d)
{
	print_info_header();
	print_line("yzl", d.yzl);
	print_line("yzr", d.yzr);
	print_line("xzl", d.xzl);
	print_line("xzr", d.xzr);
}

} /* namespace img_process */
