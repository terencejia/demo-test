#include <opencv2/imgproc/imgproc.hpp>

#include <fs_log.hpp>

#include "img_process.hpp"
#include "fiber_analyzer_cmm.hpp"
#include "math_ext.hpp"

namespace img_process {

static void detect_defect(const gray_img & img, const ipp_t & ipp,
		   const img_analyze_param_t & iap,
		   ifd_line_t & ld,
		   gray_img & result)
{
	ld.dbmp = 0;
	/// #. get abstract
	const auto abstract = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM);
	if (!abstract.has_fiber()) {
		ld.dbmp |= ifd_cant_identify;
		return;
	}

	const int trow = abstract.ext_top_row(0);
	const int brow = abstract.ext_bottom_row(img.height - 1);
	const int spliter = abstract.split_gray();

	/// #. get vertex x
	const int rvx = get_fiber_vertex(img, abstract, true);
	if (rvx <= 0 || img.width <= rvx) {
		/// can't find vertex
		ld.dbmp |= ifd_cant_identify;
		return;
	}

	if (iap.check_vertex && (img.width - 1) == rvx) {
		ld.dbmp |= ifd_cant_identify;
		return;
	}

	/// #. get row info
	std::vector< row_info<int> > row_infos(rvx + 1);
	std::vector< std::size_t > cladding_widths(brow - trow + 2);
	std::vector< std::size_t > core_widths(brow - trow + 2);
	for (int i = 0; i <= rvx; ++i) {
		row_infos[i] = get_row_info(img, trow, brow, i, spliter);
		if (row_infos[i].is_valid()) {
			++cladding_widths[row_infos[i].cladding_width()];
			++core_widths[row_infos[i].core_width()];
		}
	}

	/// #. get cladding/core width
	const int cladding_median_width = static_cast<int>(get_median_pos(cladding_widths));
	const int core_median_width = static_cast<int>(get_median_pos(core_widths));

	ld.wrap_diameter = cladding_median_width + 1; /// include both edge

	/// #. check if col is valid
	std::vector< bool > col_valid(row_infos.size());
	std::vector< bool > col_core_valid(row_infos.size());
	std::size_t valid_cnt = 0;
	for (int i = 0; i <= rvx; ++i) {
		const auto & tmp = row_infos[i];
		if (tmp.is_valid()
		    && std::abs(tmp.cladding_width() - cladding_median_width) <= CLADDING_WIDTH_DIFF_THRESHOLD) {
			col_valid[i] = true;
			++valid_cnt;
		}
		else {
			col_valid[i] = false;
		}

		if (tmp.is_valid()
		    && std::abs(tmp.core_width() - core_median_width) <= CORE_WIDTH_DIFF_THRESHOLD) {
			col_core_valid[i] = true;
		}
		else {
			col_core_valid[i] = false;
		}
	}
	if (static_cast<double>(valid_cnt) / col_valid.size() < 0.5) {
		/// width / core problem
		ld.dbmp |= ifd_cant_identify;
		return;
	}

	/// #. end crude detect
#if 0
	int invalid_leftest_x = rvx;
	for (int i = rvx; i >= 0; --i) {
		const auto & tmp = row_infos[i];
		if (std::abs(tmp.cladding_width() - cladding_median_width) <= CLADDING_WIDTH_DIFF_THRESHOLD) {
			invalid_leftest_x = i+1;
			break;
		}
	}
	const int left_bound = invalid_leftest_x;
	const int right_bound = rvx;
#else
	int left_bound = 0;
	bool find_valid_fiber = find_cont_valid_cols(col_valid, true, SAMPLE_POINT_NUM, left_bound);
	if (!find_valid_fiber) {
		left_bound = 0;
	}
	int invalid_leftest_x = rvx;
	for (int i = left_bound; i < static_cast<int>(col_valid.size()); ++i) {

		if (row_infos[i].cladding_width() < cladding_median_width) {
			invalid_leftest_x = i;
			break;
		}
	}
	const int right_bound = rvx;
#endif
	int up_bound = img.height - 1;
	int bottom_bound = 0;
	for (int x = left_bound; x <= right_bound; ++x) {
		const auto & tmp = row_infos[x];
		update_min(up_bound, tmp.tro);
		update_max(bottom_bound, tmp.bro);
	}
	std::vector<int> len(img.height, 0);
	for (int y = up_bound; y <= bottom_bound; ++y) {
		int x = right_bound;
		while (x >= invalid_leftest_x) {
			if (img.at(x, y) < spliter) {
				break;
			}
			--x;
		}
		len[y] = right_bound - x;
	}

	/// \note the len must dec first, and then inc, and then end.
	/// \todo check the trend of inc/dec value.
	int x_up = up_bound;
	while (x_up < bottom_bound && (len[x_up + 1] - 1) <= len[x_up]) {
		++x_up;
	}

	int x_down = bottom_bound;
	while (up_bound < x_down && (len[x_down] + 1) >= len[x_down - 1]) {
		--x_down;
	}
	if (x_down > x_up
	    || rvx - invalid_leftest_x > END_CRUDE_THRESHOLD) {
		log_warning("img_process: end crude - up(%d), x_down(%d), width(%d)", x_up, x_down, (rvx - invalid_leftest_x));
		/// \todo enable end crude
		//ld.dbmp |= ifd_end_crude;
		for (int y = up_bound; y <= bottom_bound; ++y) {
			for (int x = right_bound - len[y] + 1; x <= right_bound; ++x) {
				result.at(x, y) = pixel_max;
			}
		}
	}

	/// #. hangle
	int lx = 0;
	int rx = rvx;
	double dy = 0;
{
	bool find_lx_result = false;
	bool find_rx_result = false;

	/// \note first check core, if fail then check cladding
	find_lx_result = find_cont_valid_cols(col_core_valid, false, SAMPLE_POINT_NUM, lx);
	find_rx_result = find_cont_valid_cols(col_core_valid, true, SAMPLE_POINT_NUM, rx);
	if (find_lx_result && find_rx_result) {
		row_info<double> lx_sub_rinfo = promote_row_info(img, spliter, lx, row_infos[lx]);
		row_info<double> rx_sub_rinfo = promote_row_info(img, spliter, rx, row_infos[rx]);
		dy = rx_sub_rinfo.core_center() - lx_sub_rinfo.core_center();
	}
	else {
		find_lx_result = find_cont_valid_cols(col_valid, false, SAMPLE_POINT_NUM, lx);
		find_rx_result = find_cont_valid_cols(col_valid, true, SAMPLE_POINT_NUM, rx);
		if (find_lx_result && find_rx_result) {
			row_info<double> lx_sub_rinfo = promote_row_info(img, spliter, lx, row_infos[lx]);
			row_info<double> rx_sub_rinfo = promote_row_info(img, spliter, rx, row_infos[rx]);
			dy = rx_sub_rinfo.cladding_center() - lx_sub_rinfo.cladding_center();
		}
		else {
			throw img_process_error(img, "can't calc hangle", iae_t::calc_horizontal_angle);
		}
	}
}
	const double h_rad = std::atan(dy / (rx - lx));
	ld.h_angle = rad2deg(h_rad);
	if (std::abs(ld.h_angle) > iap.fiber_angle_limit) {
		ld.dbmp |= ifd_horizontal_angle;
	}

	/// #. get top/bottom corner y
	int tp_y = trow;
	int bp_y = brow;
{
	bool find_rx_result = false;

	/// \note check cladding
	find_rx_result = find_cont_valid_cols(col_valid, true, SAMPLE_POINT_NUM, rx);
	if (find_rx_result) {
		tp_y = row_infos[rx].tro;
		bp_y = row_infos[rx].bro;
	}
	else {
		throw img_process_error(img, "can't find valid corner", iae_t::calc_vertical_angle);
	}
}

	tp_y += 1;
	bp_y -= 1;
	if (bp_y < tp_y) {
		ld.dbmp |= ifd_cant_identify;
		return;
	}

	gray_img denoise_img(img);
	median_blur(denoise_img, img, rx, tp_y - 2, rvx - rx + 3, bp_y - tp_y + 5);

	/// #. find top/bottom corner x
	double tp_x = rvx;
	for (int i = rvx; i > 0; --i) {
		const auto lg = denoise_img.at(i, tp_y);
		if (lg < spliter) {
			if (i == (denoise_img.width - 1)) {
				break;
			}
			const auto beg = denoise_img.at(i + 1, tp_y);
			tp_x = i + static_cast<double>(spliter - lg) / (beg - lg);
			break;
		}
	}
	double bp_x = rvx;
	for (int i = rvx; i > 0; --i) {
		const auto lg = denoise_img.at(i, bp_y);
		if (lg < spliter) {
			if (i == (denoise_img.width - 1)) {
				break;
			}
			const auto beg = denoise_img.at(i + 1, bp_y);
			bp_x = i + static_cast<double>(spliter - lg) / (beg - lg);
			break;
		}
	}

	/// #. find vertex y
	double vp_y = trow;
	double vp_x = rvx;
	for (int i = trow; i <= brow; ++i) {
		const auto lg = denoise_img.at(rvx, i);
		if (lg < spliter) {
			if (rvx == (denoise_img.width - 1)) {
				vp_y = i;
			}
			else {
				const auto beg = denoise_img.at(rvx + 1, i);
				double tmp = rvx + static_cast<double>(spliter - lg) / (beg - lg);
				if (tmp > vp_x) {
					vp_x = tmp;
					vp_y = i;
				}
			}
		}
	}

	/// #. calc v angle
	/// vp_x, vp_y    tp_x, tp_y     bp_x, bp_y
	const double tp_rad = std::atan(tp_y/tp_x) - h_rad;
	const double tp_len = std::sqrt(std::pow(tp_y, 2) + std::pow(tp_x, 2));
	const double tpx = tp_len * std::cos(tp_rad);
	const double tpy = tp_len * std::sin(tp_rad);
	const double vp_rad = std::atan(vp_y/vp_x) - h_rad;
	const double vp_len = std::sqrt(std::pow(vp_y, 2) + std::pow(vp_x, 2));
	const double vpx = vp_len * std::cos(vp_rad);
	const double vpy = vp_len * std::sin(vp_rad);
	const double bp_rad = std::atan(bp_y/bp_x) - h_rad;
	const double bp_len = std::sqrt(std::pow(bp_y, 2) + std::pow(bp_x, 2));
	const double bpx = bp_len * std::cos(bp_rad);
	const double bpy = bp_len * std::sin(bp_rad);

	const double x_dist = vpx - (tpx + bpx) / 2;
	const double y_dist = (std::max({ tpy, vpy, bpy, }) - std::min({ tpy, vpy, bpy, })) / 2;
	ld.v_angle = rad2deg(std::atan(x_dist/y_dist));
	if (std::abs(ld.v_angle) > iap.cut_angle_limit) {
		ld.dbmp |= ifd_vertical_angle;
	}
}

static void flip_img_horizontal(gray_img & img)
{
	const cv::Mat src(img.height, img.width, CV_8UC1, (void *)img.data());
	cv::flip(src, src, 1);
}

void detect_defect(const fs_imgs & imgs, const ipp_t & ipp, const img_analyze_param_t & param, img_defects_t & ret)
{
	/// xz
	{
		gray_img xz_result(imgs.xz_img.width, imgs.xz_img.height);
		gray_img xz(imgs.xz_img.width, imgs.xz_img.height);
		img_process::bilateral_filter(xz, imgs.xz_img);

		detect_defect(xz, ipp, param, ret.xzl, xz_result);

		flip_img_horizontal(xz);
		flip_img_horizontal(xz_result);
		detect_defect(xz, ipp, param, ret.xzr, xz_result);

		ret.xz_hangle_intersect = ret.xzl.h_angle + ret.xzr.h_angle;
		if (std::abs(ret.xz_hangle_intersect) > std::abs(param.fiber_angle_limit)) {
			ret.xzl.dbmp |= ifd_horizontal_angle;
			ret.xzr.dbmp |= ifd_horizontal_angle;
		}

		flip_img_horizontal(xz_result);

		static const char * const xz_defects_path = "/tmp/x_defects.png";
		save_defects(xz_defects_path, xz_result);
		ret.xz_img = xz_defects_path;
	}

	/// yz
	{
		gray_img yz_result(imgs.yz_img.width, imgs.yz_img.height);
		gray_img yz(imgs.yz_img.width, imgs.yz_img.height);
		img_process::bilateral_filter(yz, imgs.yz_img);

		detect_defect(yz, ipp, param, ret.yzl, yz_result);

		flip_img_horizontal(yz);
		flip_img_horizontal(yz_result);
		detect_defect(yz, ipp, param, ret.yzr, yz_result);

		ret.yz_hangle_intersect = ret.yzl.h_angle + ret.yzr.h_angle;
		if (std::abs(ret.yz_hangle_intersect) > std::abs(param.fiber_angle_limit)) {
			ret.yzl.dbmp |= ifd_horizontal_angle;
			ret.yzr.dbmp |= ifd_horizontal_angle;
		}

		flip_img_horizontal(yz_result);

		static const char * const yz_defects_path = "/tmp/y_defects.png";
		save_defects(yz_defects_path, yz_result);
		ret.yz_img = yz_defects_path;
	}

	return;
}

} /* namespace img_process */

