#include "img_process.hpp"

#include "fiber_analyzer_cmm.hpp"

namespace img_process {

bool has_electric_arc(const gray_img & img, const ipp_t & ipp)
{
	const int center_gray = part_gray_scale(img,
					(img.width / 2 - SAMPLE_POINT_NUM),
					0,
					SAMPLE_POINT_NUM,
					img.height);

	const int ref_bg = static_cast<int>(ipp.bg * pixel_max);

	if (ref_bg) {
		return (center_gray - ref_bg > FIND_ARC_GRAY_DIFF_THRESHOLD);
	}

	const auto left_abstract = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM);
	const auto right_abstract = get_abstract(
			img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM);

	if (center_gray - left_abstract.background_gray > FIND_ARC_GRAY_DIFF_THRESHOLD
	    && center_gray - right_abstract.background_gray > FIND_ARC_GRAY_DIFF_THRESHOLD) {
		return true;
	}

	return false;
}

}
