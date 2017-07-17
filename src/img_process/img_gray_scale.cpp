#include <algorithm>
#include <fs_log.hpp>
#include "img_process.hpp"
#include "fiber_analyzer_cmm.hpp"

namespace img_process {
#if 0
double img_gray_scale(const gray_img & img)
{
	const int w = img.width/8;
	const int h = img.height/8;

	int xl[3] = { 0, img.width/2 - w/2, img.width - w, };
	int yl[3] = { 0, img.height/2 - h/2, img.height - h, };

	std::vector<int> sl;
	sl.reserve(3 * 3);

	for (auto x : xl) {
		for (auto y : yl) {
			sl.push_back(part_gray_scale(img, x, y, w, h));
		}
	}

	std::nth_element(sl.begin(), sl.begin() + sl.size()/2, sl.end());
	int ret = sl[sl.size()/2];
}
#else
double img_gray_scale(const gray_img & img)
{
	ipp_t ipp;
	ipp.bg = 0;
	const auto labs = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM, true);
	const auto rabs = get_abstract(img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM, true);
	const auto lbg = labs.background_gray;
	const auto rbg = rabs.background_gray;
	const auto mbg = (lbg + rbg) / 2;
	const double diffbg = std::abs(lbg - rbg) / static_cast<double>(mbg);
	if (mbg > 0 && diffbg > (BG_GRAY_DIFF_THRESHOLD * 2)) {
		log_err("img_gray_scale error: lscale : %d, rscale: %d", labs.background_gray, rabs.background_gray);
		//throw img_process_error(img, "img_gray_scale lr diff too big", iae_t::gray_scale);
	}

	return static_cast<double>(mbg) / pixel_max;
}

double img_gray_scale(const gray_img & img, bool is_left)
{
	const int xstart = (is_left ? 0 : (img.width - 10));
	const int width = 10;
	const int xend = xstart + width;

	const auto abstract = get_abstract(img, ipp, xstart, width, true);

	const int height = imgs[0].height;
	const int trow = abstract.ext_top_row(0);
	const int brow = abstract.ext_bottom_row(height - 1);

	/// 1. back
	std::vector<int> back_data(height, 0);
	for (auto & img: imgs) {
		for (int y = 0; y < trow; ++y) {
			for (int x = xstart; x < xend; ++x) {
				back_data[y] += img.at(x, y);
			}
		}
		for (int y = brow; y < height; ++y) {
			for (int x = xstart; x < xend; ++x) {
				back_data[y] += img.at(x, y);
			}
		}
	}

	std::vector<mathext::point_t> points;
	for (int i = 0; i < (int)back_data.size(); ++i) {
		if (back_data[i]) {
			points.push_back({ (double)i, (double)back_data[i] / width / imgs.size(), });
		}
	}

	std::vector<double> coefficients;
	mathext::multifit_robust(points, 2, coefficients);
	std::vector<double> trans(height, 0.0);
	double peak = 0;
	for (int i = 0; i < height; ++i) {
		trans[i] = mathext::get_poly_v(coefficients, i);
		if (trans[i] > peak) {
			peak = trans[i];
		}
	}
	return (peak / pixel_max);
}
#endif

}
