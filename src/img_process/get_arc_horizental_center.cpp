#include <fs_log.hpp>
#include <vector>
#include "img_process.hpp"
#include "math_ext.hpp"

namespace img_process {

static int __get_arc_horizental_center(const gray_img & img, const int top_row, const int bot_row)
{
	if (top_row < 0 || bot_row > img.height || top_row >= bot_row) {
		log_err("__get_arc_horizental_center: bad argument %d, %d", top_row, bot_row);
		return img.width / 2;
	}
	pixel_t median_br = static_cast<pixel_t>(img_process::get_rect_median_br(img, top_row, bot_row, 0, img.width) * pixel_max);

	std::vector<int> average;
	average.resize(img.width);
	for (int col = 0; col < img.width; col++)
	{
		average[col] = (int)(img_process::get_rect_median_br(img, top_row, bot_row, col, col+1) * pixel_max);
	}

	int median_pos_left = 0;
	for (; median_pos_left < img.width; ++median_pos_left) {
		if (average[median_pos_left] >= median_br) {
			break;
		}
	}

	int median_pos_right = img.width - 1;
	for (; median_pos_right > 0; --median_pos_right) {
		if (average[median_pos_right] >= median_br) {
			break;
		}
	}

	int middle_of_arc = (median_pos_left + median_pos_right) / 2;
	return middle_of_arc;
}

int get_arc_horizental_center(const gray_img & img, const img_abstract_t & labs, const img_abstract_t & rabs)
{
	const int sample_height = 10;
	const int center = (labs.cladding_center() + rabs.cladding_center())/2;
	const int middle = __get_arc_horizental_center(img, center - sample_height/2, center + sample_height/2);
	return middle;
}

}
