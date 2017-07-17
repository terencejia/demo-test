#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fs_log.hpp>

#include "img_process.hpp"
#include "fiber_analyzer_cmm.hpp"

namespace img_process {

double get_rect_br(const gray_img & img, const int row_start, const int row_stop, const int col_start, const int col_stop)
{
	if (row_start > img.height || row_stop > img.height
	    || row_start > row_stop) {
		log_err("error row param: %d %d", row_start, row_stop);
		throw std::invalid_argument("row param error");
	}

	if (col_start > img.width || col_stop > img.width
	    || col_start > col_stop) {
		log_err("error column param: %d %d", col_start, col_stop);
		throw std::invalid_argument("column param error");
	}

	const cv::Mat gray(img.height, img.width, CV_8UC1, (void *)img.data());
	cv::Range row_r(row_start, row_stop);
	cv::Range col_r(col_start, col_stop);
	return static_cast<double>(cv::mean(gray(row_r, col_r))[0]) / pixel_max;
}

double get_rect_median_br(const gray_img & img, const int row_start, const int row_stop,  const int col_start, const int col_stop)
{
	std::vector<std::size_t> data;
	data.resize(pixel_max + 1, 0);

	for (int i = row_start; i < row_stop; ++i) {
		for (int j = col_start; j < col_stop; ++j) {
			const pixel_t v = img.at(j, i);
			++data[v];
		}
	}

	auto median_v = get_median_pos(data);

	return static_cast<double>(median_v) / pixel_max;
}

double get_br_during_arc(const gray_img & img, const img_abstract_t img_abs)
{
	const int trow = img_abs.top_row;
	const int brow = img_abs.bottom_row;
	const int fiber_width = std::abs(img_abs.cladding_width());

	auto br_ld = img_process::get_rect_br(img, trow+fiber_width/9, trow+fiber_width/3, 0, img.width/40*19);
	auto br_lu = img_process::get_rect_br(img, brow-fiber_width/3, brow-fiber_width/9, 0, img.width/40*19);
	auto br_rd = img_process::get_rect_br(img, trow+fiber_width/9, trow+fiber_width/3, img.width/40*21, img.width);
	auto br_ru = img_process::get_rect_br(img, brow-fiber_width/3, brow-fiber_width/9, img.width/40*21, img.width);
	double br = (br_ld + br_lu + br_rd + br_ru)/4;
	log_debug("br = %f : [%f, %f, %f, %f]", br, br_ld, br_lu, br_rd, br_ru);

	return br;
}

}
