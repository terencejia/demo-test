
#include <algorithm>

#include <fs_log.hpp>

#include "img_process.hpp"

#include "fiber_analyzer_cmm.hpp"

namespace img_process {

bool find_cont_valid_cols(const std::vector<bool> & col_valid, const bool is_left, const int cnt, int & valid_lx)
{
	int cont_cnt = 0;
	const int dst_cnt = cnt * 2;
	const int width = static_cast<int>(col_valid.size());
	if (is_left) {
		for (int i = width - 1; i >= 0; --i) {
			if (col_valid[i]) {
				++cont_cnt;
				if (cont_cnt >= dst_cnt) {
					valid_lx = i;
					break;
				}
			}
			else {
				cont_cnt = 0;
			}
		}
	}
	else {
		for (int i = 0; i < width; ++i) {
			if (col_valid[i]) {
				++cont_cnt;
				if (cont_cnt >= dst_cnt) {
					valid_lx = i + 1 - cont_cnt;
					break;
				}
			}
			else {
				cont_cnt = 0;
			}
		}
	}

	if (cont_cnt < dst_cnt) {
		return false;
	}
	else {
		valid_lx += (dst_cnt - cnt) / 2;
		return true;
	}
}

static double bmp_calc_cladding_y(const gray_img & img,
			 const int spliter,
			 const int trow, const int brow,
			 const int leftx, const int width,
			 const bool is_left)
{
	const int height = brow - trow + 1;
	int rightx = leftx + width;
	if (trow < 0 || img.height <= brow
	    || leftx < 0 || width < 1 || img.width < rightx) {
		log_err("%s: param err: lx(%d), rx(%d) ", __func__, leftx, rightx);
		throw img_process_error(img, "bmp_calc_cladding_y param error", iae_t::param_error);
	}

	/// 1. preprocess
	std::vector< row_info<int> > ri(width);
	for (int i = 0; i < width; ++i) {
		ri[i] = get_row_info(img, trow, brow, leftx + i, spliter);
	}

	/// 2. find valid cladding
	/// \note find continuous SAMPLE_POINT_NUM * 2 columns,
	/// but only use SAMPLE_POINT_NUM columns.
	int valid_lx = 0;	/// based on leftx
	int cont_cnt = SAMPLE_POINT_NUM * 2;
	const bool find_cladding = [height,
				width,
				&valid_lx,
				&cont_cnt,
				/* const*/ &ri,
				is_left,
				/* const*/ & img](void) -> bool {
		std::vector< std::size_t > cladding_widths(height + 1);
		for (auto & i : ri) {
			if (i.is_cladding_valid()) {
				++cladding_widths[i.cladding_width()];
			}
		}
		const int cladding_median_width = static_cast<int>(get_median_pos(cladding_widths));

		std::vector< bool > cladding_valid(ri.size());
		for (int i = 0; i < width; ++i) {
			const auto & tmp = ri[i];
			if (tmp.is_cladding_valid()
				&& std::abs(tmp.cladding_width() - cladding_median_width) <= CLADDING_WIDTH_DIFF_THRESHOLD) {
				cladding_valid[i] = true;
			}
			else {
				cladding_valid[i] = false;
			}
		}

		while (cont_cnt >= SAMPLE_POINT_NUM) {
			bool ret = find_cont_valid_cols(cladding_valid, is_left, cont_cnt, valid_lx);
			if (ret) {
				return ret;
			}
			if (valid_lx < (img.width * 0.25)
			    || (valid_lx + cont_cnt) > (img.width * 0.75)) {
				cont_cnt -= SAMPLE_POINT_NUM / 2;
			}
		}

		return false;
	}();

	if (!find_cladding) {
		throw img_process_error(img, "bmp_calc_cladding_y no valid cladding");
	}

	/// 3. denoise
	gray_img filter_img(img);
	{
		/// here we can ensure the right cont_cnt pixels of valid_lx are all valid
		static constexpr int extent_pixel = 5;
		const int roi_x = std::max(valid_lx + leftx - extent_pixel, 0);
		const int roi_w = std::min(valid_lx + leftx + cont_cnt + extent_pixel, img.width) - roi_x;
		{
			const int roi_y = std::max(ri[valid_lx + cont_cnt / 2].tro - extent_pixel, 0);
			const int roi_h = std::min(ri[valid_lx + cont_cnt / 2].tro + extent_pixel, img.height) - roi_y;
			if (roi_w > 0 && roi_h > 0) {
				median_blur(filter_img, img, roi_x, roi_y, roi_w, roi_h);
			}
		}
		{
			const int roi_y = std::max(ri[valid_lx + cont_cnt / 2].bro - extent_pixel, 0);
			const int roi_h = std::min(ri[valid_lx + cont_cnt / 2].bro + extent_pixel, img.height) - roi_y;
			if (roi_w > 0 && roi_h > 0) {
				median_blur(filter_img, img, roi_x, roi_y, roi_w, roi_h);
			}
		}
	}

	/// 4. use cladding external edge
	std::vector<int> adv_col_data(height);
	for (int j = 0; j < height; ++j) {
		const int y = trow + j;
		int row_sum = 0;
		for (int i = 0; i < cont_cnt; ++i) {
			const int x = valid_lx + leftx + i;
			row_sum += filter_img.at(x, y);
		}
		adv_col_data[j] = row_sum;
	}
	const row_info<std::size_t> fr = get_row_info(adv_col_data, spliter * cont_cnt);
	const row_info<double> frr = promote_row_info(img, adv_col_data, spliter * cont_cnt, fr);
	return frr.cladding_center() + trow;
}

double bmp_calc_core_y(const gray_img & img,
			 const int spliter,
			 const int trow, const int brow,
			 const int leftx, const int width,
			 const bool is_left)
{
	const int height = brow - trow + 1;
	int rightx = leftx + width;
	if (trow < 0 || img.height <= brow
	    || leftx < 0 || width < 1 || img.width < rightx) {
		log_err("%s: param err: lx(%d), rx(%d) ", __func__, leftx, rightx);
		throw img_process_error(img, "bmp_calc_core_y param error", iae_t::param_error);
	}

	/// 1. preprocess
	std::vector< row_info<int> > ri(width);
	for (int i = 0; i < width; ++i) {
		ri[i] = get_row_info(img, trow, brow, leftx + i, spliter);
	}

	/// 2. find valid core
	/// \note find continuous SAMPLE_POINT_NUM * 2 columns,
	/// but only use SAMPLE_POINT_NUM columns.
	int valid_lx = 0;	/// based on leftx
	int cont_cnt = SAMPLE_POINT_NUM * 2;
	const bool find_core = [height,
				width,
				&valid_lx,
				&cont_cnt,
				/* const*/ &ri,
				is_left,
				/* const*/ & img](void) -> bool {
		std::vector< std::size_t > core_widths(height + 1);
		for (auto & i : ri) {
			if (i.is_valid()) {
				++core_widths[i.core_width()];
			}
		}
		const int core_median_width = static_cast<int>(get_median_pos(core_widths));

		std::vector< bool > col_valid(ri.size());
		for (int i = 0; i < width; ++i) {
			const auto & tmp = ri[i];
			if (tmp.is_valid()
				&& std::abs(tmp.core_width() - core_median_width) <= CORE_WIDTH_DIFF_THRESHOLD) {
				col_valid[i] = true;
			}
			else {
				col_valid[i] = false;
			}
		}

		while (cont_cnt >= SAMPLE_POINT_NUM) {
			bool ret = find_cont_valid_cols(col_valid, is_left, cont_cnt, valid_lx);
			if (ret) {
				return ret;
			}
			if (valid_lx < (img.width * 0.25)
			    || (valid_lx + cont_cnt) > (img.width * 0.75)) {
				cont_cnt -= SAMPLE_POINT_NUM / 2;
			}
		}

		return false;
	}();

	if (!find_core) {
		log_warning("can't find valid core, use external edge");

		return bmp_calc_cladding_y(img, spliter, trow, brow, leftx, width, is_left);
	}
	/**
	 * \note the real core is not fine for precise calibrating,
	 * so we use cladding internal edge.
	 */
#if 1
	/// denoise
	gray_img filter_img(img);
	{
		/// here we can ensure the right cont_cnt pixels of valid_lx are all valid
		static constexpr int extent_pixel = 5;
		const int roi_x = std::max(valid_lx + leftx - extent_pixel, 0);
		const int roi_w = std::min(valid_lx + leftx + cont_cnt + extent_pixel, img.width) - roi_x;
		{
			const int roi_y = std::max(ri[valid_lx + cont_cnt / 2].tri - extent_pixel, 0);
			const int roi_h = std::min(ri[valid_lx + cont_cnt / 2].tri + extent_pixel, img.height) - roi_y;
			if (roi_w > 0 && roi_h > 0) {
				median_blur(filter_img, img, roi_x, roi_y, roi_w, roi_h);
			}
		}
		{
			const int roi_y = std::max(ri[valid_lx + cont_cnt / 2].bri - extent_pixel, 0);
			const int roi_h = std::min(ri[valid_lx + cont_cnt / 2].bri + extent_pixel, img.height) - roi_y;
			if (roi_w > 0 && roi_h > 0) {
				median_blur(filter_img, img, roi_x, roi_y, roi_w, roi_h);
			}
		}
	}
	/// 5. use cladding internal edge
	std::vector<int> adv_col_data(height);
	for (int j = 0; j < height; ++j) {
		const int y = trow + j;
		int row_sum = 0;
		for (int i = 0; i < cont_cnt; ++i) {
			const int x = valid_lx + leftx + i;
			row_sum += filter_img.at(x, y);
		}
		adv_col_data[j] = row_sum;
	}
	const row_info<std::size_t> fr = get_row_info(adv_col_data, spliter * cont_cnt);
	const row_info<double> frr = promote_row_info(img, adv_col_data, spliter * cont_cnt, fr);
	return frr.core_center() + trow;
#else
	valid_lx += leftx;
	/// denoise
	gray_img filter_img(img);
	{
		static constexpr int extent_pixel = 5;
		const int roi_x = std::max(valid_lx - extent_pixel, 0);
		const int roi_y = std::max(ri[roi_x - leftx].tri - extent_pixel, 0);
		const int roi_w = std::min(valid_lx + cont_cnt + extent_pixel, img.width) - roi_x;
		const int roi_h = std::min(ri[roi_x - leftx].bri + extent_pixel, img.height) - roi_y;
		median_blur(filter_img, img, roi_x, roi_y, roi_w, roi_h);
	}

	/// 4. further check if core is valid
	std::vector< int > col_max_y(cont_cnt);
	std::vector< int > col_max_gray(cont_cnt);
	const bool is_core_valid = [leftx, cont_cnt, valid_lx, &ri, &filter_img, &col_max_y, &col_max_gray]() -> bool
	{
		for (int i = 0; i < cont_cnt; ++i) {
			const int x = valid_lx + i;
			const auto & tmp = ri[x - leftx];
			int mp_y = tmp.tri;
			int mp_g = filter_img.at(x, mp_y);
			for (int y = tmp.tri + 1; y <= tmp.bri; ++y) {
				const int tmp_g = static_cast<int>(filter_img.at(x, y));
				if (mp_g < tmp_g) {
					mp_y = y;
					mp_g = tmp_g;
				}
			}

			if (std::abs(tmp.core_center() - mp_y) > 1) {
				return false;
			}

			col_max_gray[i] = mp_g;
			col_max_y[i] = mp_y;
		}

		int min_cg = col_max_gray[0];
		int max_cg = col_max_gray[0];
		for (int i = 1; i < cont_cnt; ++i) {
			update_minmax(min_cg, max_cg, col_max_gray[i]);
		}
		if (std::abs(max_cg - min_cg) > PRECISE_CALIBRATING_GRAY_DIFF_THRESHOLD) {
			return false;
		}

		return true;
	}();

	if (!is_core_valid) {
		/// 5. use cladding internal edge
		std::vector<int> adv_col_data(height);
		for (int j = 0; j < height; ++j) {
			const int y = trow + j;
			int row_sum = 0;
			for (int i = 0; i < cont_cnt; ++i) {
				const int x = valid_lx + i;
				row_sum += filter_img.at(x, y);
			}
			adv_col_data[j] = row_sum;
		}
		const row_info<std::size_t> fr = get_row_info(adv_col_data, spliter * cont_cnt);
		const row_info<double> frr = promote_row_info(img, adv_col_data, spliter * cont_cnt, fr);
		return frr.core_center() + trow;
	}

	/// 6. use real core
	double sum_center = 0.0;
	for (int i = 0; i < cont_cnt; ++i) {
		const int x = valid_lx + i;
		const auto & tmp = ri[x - leftx];
		const int valley_up_threshold = col_max_gray[i] - PRECISE_CALIBRATING_GRAY_DIFF_THRESHOLD;

		int lvalley = col_max_y[i];
		while (lvalley > tmp.tri
			&& filter_img.at(x, lvalley) > valley_up_threshold) {
			--lvalley;
		}
		while (lvalley > tmp.tri
			&& filter_img.at(x, lvalley) > filter_img.at(x, lvalley - 1)) {
			--lvalley;
		}

		int rvalley = col_max_y[i];
		while (rvalley < tmp.bri
			&& filter_img.at(x, rvalley) > valley_up_threshold) {
			++rvalley;
		}
		while (rvalley < tmp.bri
			&& filter_img.at(x, rvalley) > filter_img.at(x, rvalley + 1)) {
			++rvalley;
		}

		const int col_spliter = (std::max(filter_img.at(x, lvalley), filter_img.at(x, rvalley))
				+ col_max_gray[i]) / 2;
		double lret = lvalley;
		for (int y = lvalley; y <= col_max_y[i]; ++y) {
			const int beg = filter_img.at(x, y);
			if (beg >= col_spliter) {
				lret = y - (beg - col_spliter) / (beg - filter_img.at(x, y-1));
				break;
			}
		}
		double rret = rvalley;
		for (int y = rvalley; y >= col_max_y[i]; --y) {
			const int beg = filter_img.at(x, y);
			if (beg >= col_spliter) {
				rret = y + (beg - col_spliter) / (beg - filter_img.at(x, y+1));
				break;
			}
		}

		sum_center += (lret + rret) / 2;
	}

	return (sum_center / cont_cnt);
#endif
}

double xy_precise_dist_core(
	const gray_img & img, const ipp_t & ipp,
	int base, int off, const int ref_spliter)
{
	const auto labs = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM);
	const auto rabs = get_abstract(img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM);
	if (!labs.has_fiber() || !rabs.has_fiber()) {
		log_err("%s %s no fiber", __func__, labs.has_fiber() ? "right" : "left");
		return 0;
	}

	int spliter = (labs.split_gray() + rabs.split_gray()) / 2;
	if (ref_spliter) {
		if (std::abs(ref_spliter - spliter) <= SPLITER_DIFF) {
			spliter = ref_spliter;
		}
		else {
			log_err("spliter error: ref(%d), real(%d)", ref_spliter, spliter);
		}
	}

	const int rough_dist = rabs.cladding_center() - labs.cladding_center();
	if (std::abs(rough_dist) > DIST_THRESHOLD) {
		return rough_dist;
	}

	double lc = bmp_calc_core_y(img, spliter,
			labs.ext_top_row(0), labs.ext_bottom_row(img.height - 1),
			0, base - off,
			true);
	double rc = bmp_calc_core_y(img, spliter,
			rabs.ext_top_row(0), rabs.ext_bottom_row(img.height - 1),
			base + off, img.width - (base + off),
			false);
	return rc - lc;
}

double xy_precise_dist_cladding(
	const gray_img & img, const ipp_t & ipp,
	int base, int off, const int ref_spliter)
{
	const auto labs = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM);
	const auto rabs = get_abstract(img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM);
	if (!labs.has_fiber() || !rabs.has_fiber()) {
		log_err("%s %s no fiber", __func__, labs.has_fiber() ? "right" : "left");
		return 0;
	}

	int spliter = (labs.split_gray() + rabs.split_gray()) / 2;
	if (ref_spliter) {
		if (std::abs(ref_spliter - spliter) <= SPLITER_DIFF) {
			spliter = ref_spliter;
		}
		else {
			log_err("spliter error: ref(%d), real(%d)", ref_spliter, spliter);
		}
	}

	const int rough_dist = rabs.cladding_center() - labs.cladding_center();
	if (std::abs(rough_dist) > DIST_THRESHOLD) {
		return rough_dist;
	}

	double lc = bmp_calc_cladding_y(img, spliter,
			labs.ext_top_row(0), labs.ext_bottom_row(img.height - 1),
			0, base - off,
			true);
	double rc = bmp_calc_cladding_y(img, spliter,
			rabs.ext_top_row(0), rabs.ext_bottom_row(img.height - 1),
			base + off, img.width - (base + off),
			false);
	return rc - lc;
}

double xy_dist(const gray_img & img, const ipp_t & ipp)
{
	const auto labs = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM, true);
	const auto rabs = get_abstract(img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM, true);
	if (!labs.has_fiber() || !rabs.has_fiber()) {
		log_debug("%s %s no fiber", __func__, labs.has_fiber() ? "right" : "left");
		throw img_process_error(img, "no fiber", iae_t::no_fiber);
		return 0;
	}

	return rabs.cladding_center() - labs.cladding_center();
}

} /* namespace img_process */
