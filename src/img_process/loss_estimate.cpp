#include <fs_log.hpp>

#include "img_process.hpp"

#include "fiber_analyzer_cmm.hpp"

namespace img_process {

static bool img_preprocess(const gray_img & img, const ipp_t & ipp, const int leftx, const int width,
			   int & spliter,
			   std::vector< row_info<int> > & row_infos,
			   std::vector< row_info<double> > & row_info_detail)
{
	const int rightx = leftx + width;
	if (leftx < 0 || width < 1
	    || img.width < rightx) {
		log_err("loss_estimate: %d %d", leftx, width);
		throw img_process_error(img, "loss_estimate_base param error", iae_t::param_error);
	}

	const auto left_abstract = get_abstract(img, ipp, leftx, SAMPLE_POINT_NUM);
	const auto right_abstract = get_abstract(img, ipp, rightx - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM);
	if (!left_abstract.has_fiber()
	    || !right_abstract.has_fiber()) {
		log_err("loss_estimate: can't find fiber");
		throw img_process_error(img, "loss_estimate_base param error", iae_t::no_fiber);
	}

	const int trow = std::min(left_abstract.ext_top_row(0),
			    right_abstract.ext_top_row(0));
	const int brow = std::max(left_abstract.ext_bottom_row(img.height - 1),
			    right_abstract.ext_bottom_row(img.height - 1));

	spliter = (left_abstract.split_gray() + right_abstract.split_gray()) / 2;

	row_infos.resize(width);
	for (int i = 0; i < width; ++i) {
		row_infos[i] = get_row_info(img, trow, brow, leftx + i, spliter);
		if (!row_infos[i].is_valid()) {
			return false;
		}
	}

	row_info_detail.resize(4);
	row_info_detail[0] = promote_row_info(img, spliter, leftx + (width/2 - width/40 * 2), row_infos[(width/2 - width/40 * 2)]);
	row_info_detail[1] = promote_row_info(img, spliter, leftx + (width/2 - width/40 * 1), row_infos[(width/2 - width/40 * 1)]);
	row_info_detail[2] = promote_row_info(img, spliter, leftx + (width/2 + width/40 * 1), row_infos[(width/2 + width/40 * 1)]);
	row_info_detail[3] = promote_row_info(img, spliter, leftx + (width/2 + width/40 * 2), row_infos[(width/2 + width/40 * 2)]);

	return true;
}

static double loss_estimate_base(const gray_img & img, const std::vector< row_info<int> > & row_infos)
{
	if (row_infos.empty()) {
		throw img_process_error(img, "loss_estimate_base param empty", iae_t::param_error);
	}

	row_info<int> rimin = row_infos[0];
	row_info<int> rimax = row_infos[0];
	int width_min = row_infos[0].cladding_width();
	for (const auto & tmp : row_infos) {
		update_minmax(rimin.tro, rimax.tro, tmp.tro);
		update_minmax(rimin.tri, rimax.tri, tmp.tri);
		update_minmax(rimin.bri, rimax.bri, tmp.bri);
		update_minmax(rimin.bro, rimax.bro, tmp.bro);
		update_min(width_min, tmp.cladding_width());
	}

	int fiber_width = std::min(
		(*row_infos.begin()).cladding_width(),
		(*row_infos.rbegin()).cladding_width());

	double ret = 0.0;
	ret += 0.1  * std::max(0, fiber_width - width_min - 4);	/// sunken
	ret += 0.01 * std::pow(std::max(0, rimax.tro - rimin.tro - 2), 2);
	ret += 0.05 * std::pow(std::max(0, rimax.tri - rimin.tri - 2), 2);
	ret += 0.05 * std::pow(std::max(0, rimax.bri - rimin.bri - 2), 2);
	ret += 0.01 * std::pow(std::max(0, rimax.bro - rimin.bro - 2), 2);

	return ret;
}

static double fiber_dist(const std::vector< row_info<double> > & xinfo,
			 const std::vector< row_info<double> > & yinfo,
			 bool is_core)
{
	std::size_t size = std::min(xinfo.size(), yinfo.size());
	double ret = 0.0;

	if (is_core) {
		double x_center = xinfo[0].core_center();
		double y_center = yinfo[0].core_center();

		for (std::size_t i = 1; i < size; ++i) {
			double tmp = std::sqrt(
				std::pow((xinfo[i].core_center() - x_center), 2)
				+ std::pow((yinfo[i].core_center() - y_center), 2));
			update_max(ret, tmp);
		}
	}
	else {
		double x_center = xinfo[0].cladding_center();
		double y_center = yinfo[0].cladding_center();

		for (std::size_t i = 1; i < size; ++i) {
			double tmp = std::sqrt(
				std::pow((xinfo[i].cladding_center() - x_center), 2)
				+ std::pow((yinfo[i].cladding_center() - y_center), 2));
			update_max(ret, tmp);
		}
	}

	return ret;
}

static double accurate_process(const gray_img & img,
			       const int leftx,
			       const int topy,
			       const int width,
			       const int height)
{
	const int rightx = leftx + width;
	const int bottomy = topy + height;
	if (leftx < 0 || topy < 0
	    || width < 1 || height < 1
	    || img.width < rightx || img.height < bottomy) {
		log_err("accurate_process: x%d y%d w%d h%d", leftx, topy, width, height);
		throw img_process_error(img, "accurate_process param error", iae_t::param_error);
	}

	double ret = 0.0;

	int min = img.at(leftx, topy);
	int max = min;
	for (int y = topy; y < bottomy; ++y) {
		int col_min = img.at(leftx, y);
		int col_max = col_min;
		for (int x = leftx; x < (leftx + width/4); ++x) {
			update_minmax(col_min, col_max, static_cast<int>(img.at(x, y)));
		}
		for (int x = (rightx - width/4); x < rightx; ++x) {
			update_minmax(col_min, col_max, static_cast<int>(img.at(x, y)));
		}
		for (int x = (leftx + width/4); x < (rightx - width/4); ++x) {
			int tmp = img.at(x, y);
			if (tmp < col_min) {
				ret += col_min - tmp - 1;
			}
			else if (col_max < tmp) {
				ret += tmp - col_max - 1;
			}
		}
		update_min(min, col_min);
		update_max(max, col_max);
	}

	if (max <= 0) {
		throw img_process_error(img, "accurate_process no fiber", iae_t::no_fiber);
	}

	ret *= width / static_cast<double>(max);

	return ret;
}

double loss_estimate(const fs_imgs & imgs, const ipp_t & ipp, const loss_estimate_param_t & lep)
{
	if (lep.leftx < 0 || lep.width < 1
	    || imgs.width() < (lep.leftx + lep.width)) {
		log_err("loss_estimate: %d %d", lep.leftx, lep.width);
		throw img_process_error(imgs.xz_img, "loss_estimate param error", iae_t::param_error);
	}

	gray_img xz_img(imgs.xz_img.width, imgs.xz_img.height);
	img_process::bilateral_filter(xz_img, imgs.xz_img);
	gray_img yz_img(imgs.yz_img.width, imgs.yz_img.height);
	img_process::bilateral_filter(yz_img, imgs.yz_img);

	int x_spliter = 0;
	std::vector< row_info<int> > x_rinfo;
	std::vector< row_info<double> > x_rinfo_detail;
	bool x_pp_result = img_preprocess(xz_img, ipp, lep.leftx, lep.width, x_spliter, x_rinfo, x_rinfo_detail);
	if (!x_pp_result) {
		return -1;
	}

	int y_spliter = 0;
	std::vector< row_info<int> > y_rinfo;
	std::vector< row_info<double> > y_rinfo_detail;
	bool y_pp_result = img_preprocess(yz_img, ipp, lep.leftx, lep.width, y_spliter, y_rinfo, y_rinfo_detail);
	if (!y_pp_result) {
		return -1;
	}

	double ret = 0.0;
	ret += loss_estimate_base(imgs.yz_img, y_rinfo);
	ret += loss_estimate_base(imgs.xz_img, x_rinfo);
	ret += 0.003 * std::max(lep.left_cut_angle, lep.right_cut_angle);

	if (lep.left_mfd < 0.1 || lep.right_mfd < 0.1) {
		return -1;
	}

	switch(lep.mode) {
	case loss_estimate_mode_t::core:
		ret += 0.1 * lep.core_diff_pre;
		ret += 25  * fiber_dist(x_rinfo_detail, y_rinfo_detail, true) * 4 / (M_PI * 5 * (lep.left_mfd + lep.right_mfd));
		break;
	case loss_estimate_mode_t::cladding:
		ret += 0.05 * lep.cladding_diff_pre;
		ret += 4    * fiber_dist(x_rinfo_detail, y_rinfo_detail, false) * 4 / (M_PI * 5 * (lep.left_mfd + lep.right_mfd));
		break;
	case loss_estimate_mode_t::accurate:
		ret += 0.1 * lep.core_diff_pre;
		ret += 10  * fiber_dist(x_rinfo_detail, y_rinfo_detail, true) * 4 / (M_PI * 5 * (lep.left_mfd + lep.right_mfd));
		{
			/// use the 1/4 pixels in the center of dst region.
			const int leftx_ap = lep.leftx + lep.width * 3 / 8;
			const int width_ap = lep.width / 4;
			const auto y_abstract = get_abstract(yz_img, ipp, leftx_ap, width_ap);
			const auto x_abstract = get_abstract(xz_img, ipp, leftx_ap, width_ap);
			if (!x_abstract.has_fiber()
			|| !y_abstract.has_fiber()) {
				log_err("loss_estimate accurate : can't find fiber");
				return -1;
			}
			const double x_ap_result = lep.final_factor * accurate_process(xz_img,
						leftx_ap, x_abstract.top_row,
						width_ap, (x_abstract.bottom_row - x_abstract.top_row + 1));
			const double y_ap_result = lep.final_factor * accurate_process(yz_img,
						leftx_ap, y_abstract.top_row,
						width_ap, (y_abstract.bottom_row - y_abstract.top_row + 1));

			ret += 0.05 * (x_ap_result + y_ap_result)
				* (lep.vertex_intersect_angle > 90.0 ? lep.syntropy_bending_coefficient : lep.reverse_bending_coefficient);
		}
		break;
	default:
		break;
	}

	ret += 0.5;

	if (lep.mfd_mismatch_coefficient >= 0.01) {
		const double tmp = std::abs(
			std::pow(lep.left_mfd, 2) - std::pow(lep.right_mfd, 2));
		ret += (tmp / lep.left_mfd + tmp / lep.right_mfd)
			* lep.mfd_mismatch_coefficient
			/ 4;
	}

	/// \note pre unit is 0.01db
	ret /= 100;
	/// \note post unit is 1db
	ret += lep.loss_min;

	return ret;
}

}
