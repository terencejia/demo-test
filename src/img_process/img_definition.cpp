#include <numeric>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>

#include <fs_log.hpp>

#include "img_process.hpp"
#include "fiber_analyzer_cmm.hpp"

namespace img_process {

static bool confirm(const gray_img & img, const std::vector<int> & tmp,
	     const std::vector<int> & adj,
	     int tri_adv, int bri_adv, int center_min_idx)
{
	if (bri_adv <= tri_adv
		|| tri_adv <= 0
		|| (int)tmp.size() <= bri_adv
		|| tmp.size() != adj.size()
		|| center_min_idx <= tri_adv
		|| bri_adv <= center_min_idx) {
		log_err("confirm error arg");
		throw img_process_error(img, "confirm param error", iae_t::img_definition);
	}

	/// triangle: tri_adv -- center_min_idx -- bri_adv
	int low_cnt = 0;
	int high_cnt = 0;
	/// top half triangle
	const int tri_lb_idx = tri_adv + std::max(1, (int)((bri_adv - tri_adv) * 0.15));
	const int tri_rb_idx = bri_adv - std::max(1, (int)((bri_adv - tri_adv) * 0.15));
	/// \note (tri_lb_idx <= center_min_idx && center_min_idx <= tri_rb_idx) test may fail
	if (tri_lb_idx >= tri_rb_idx) {
		log_err("confirm error tri t(%d) b(%d) (%d) (%d) c(%d)", tri_adv, bri_adv, tri_lb_idx, tri_rb_idx, center_min_idx);
		throw img_process_error(img, "confirm error", iae_t::img_definition);
	}
	for (int i = tri_adv + 1; i < center_min_idx; ++i) {
		const int tri_lb_gray = std::max(tmp[tri_adv], tmp[bri_adv]);
		const double ratio = (i - tri_lb_idx) / (double)(center_min_idx - tri_lb_idx);
		double dst_gray = tri_lb_gray + ratio * (tmp[center_min_idx] - tri_lb_gray);
		if ((tmp[i] + 0.5) < dst_gray) {
			++low_cnt;
		}
		if (dst_gray < (tmp[i] - 0.5)) {
			++high_cnt;
		}
	}
	/// bottom half triangle
	for (int i = bri_adv - 1; i > center_min_idx; --i) {
		const int tri_rb_gray = std::max(tmp[tri_adv], tmp[bri_adv]);
		const double ratio = (i - tri_rb_idx) / (double)(center_min_idx - tri_rb_idx);
		double dst_gray = tri_rb_gray + ratio * (tmp[center_min_idx] - tri_rb_gray);
		if ((tmp[i] + 0.5) < dst_gray) {
			++low_cnt;
		}
		if (dst_gray < (tmp[i] - 0.5)) {
			++high_cnt;
		}
	}

	return (high_cnt > 0);
}

int definition_estimate(const gray_img & img, std::vector<int> & wave, bool is_left)
{
	const int xstart = is_left ? 0 : img.width - SAMPLE_POINT_NUM;
	const int xstop = xstart + SAMPLE_POINT_NUM;

	std::vector<int> & tmp = wave;
	tmp.resize(0);
	tmp.resize(img.height, 0);
	//tmp.fill(0);
	for (int y = 0; y < img.height; ++y) {
		for (int x = xstart; x < xstop; ++x) {
			tmp[y] += img.at(x, y);
		}
		tmp[y] = (int)((tmp[y] + 0.5) / SAMPLE_POINT_NUM);
	}
	std::vector<int> adj(img.height);
	std::adjacent_difference(tmp.begin(),tmp.end(),adj.begin());
	adj[0] = 0;

	/// get_abstract
	const ipp_t ipp = { 0, };
	const auto abstract = get_abstract(img, ipp, xstart, SAMPLE_POINT_NUM);
	if (!abstract.has_fiber()) {
		log_debug("bg is %d but no fiber", abstract.background_gray);
		throw img_process_error(img, "no fiber", iae_t::img_definition);
	}
	const int bg = abstract.background_gray;
	if ((double)bg / tmp[0] < 0.8) {
		log_debug("bg is %d but indeed %d", bg, tmp[0]);
		throw img_process_error(img, "error bg color", iae_t::img_definition);
	}

	/// rough judge for ranges without obvious characteristic
	const row_info<std::size_t> ri = get_row_info(tmp, abstract.cladding_gray + (abstract.background_gray - abstract.cladding_gray) * 3 / 6);
	if (!ri.is_valid()) {
		return fc_lvl_4;
	}

	{
		const row_info<std::size_t> ris = get_row_info(tmp, abstract.cladding_gray + (abstract.background_gray - abstract.cladding_gray) * 4 / 6);
		if (!ris.is_valid()) {
			return fc_lvl_3;
		}
	}
	{
		const row_info<std::size_t> ris = get_row_info(tmp, abstract.cladding_gray + (abstract.background_gray - abstract.cladding_gray) * 5 / 6);
		if (!ris.is_valid()) {
			return fc_lvl_2;
		}
	}

	/// \note the tri/bri in above ri is not valid, so we use lower spliter
	/// to recalc them.
	int tri_adv = ri.tri;
	int bri_adv = ri.bri;
	while (adj[tri_adv + 1] >= 0 && tri_adv <= bri_adv) {
		++tri_adv;
	}
	while (adj[bri_adv] <= 0 && tri_adv <= bri_adv) {
		--bri_adv;
	}

	if (tri_adv >= bri_adv) {
		return fc_lvl_0_3;
	}

	/// find the most black point in core
	const auto center_min_el = std::min_element(tmp.begin() + tri_adv, tmp.begin() + bri_adv + 1);
	const int center_min_idx = std::distance(tmp.begin(), center_min_el);
	if (!(tri_adv <= center_min_idx && center_min_idx <= bri_adv)) {
		log_err("no center min");
		throw img_process_error(img, "no center min", iae_t::img_definition);
	}
	if (!(tmp[center_min_idx] < std::min(tmp[bri_adv], tmp[bri_adv]))) {
		log_err("center min too big");
		throw img_process_error(img, "center min too big", iae_t::img_definition);
	}

	/// \note we can do a simple check for sagger.
	bool is_sag = [tri_adv, center_min_idx, bri_adv, &adj](void) -> bool {
		for (int i = tri_adv + 1; i < center_min_idx; ++i) {
			if (adj[i] > 0) {
				return false;
			}
		}
		for (int i = bri_adv; i > center_min_idx; --i) {
			if (adj[i] < 0) {
				return false;
			}
		}
		return true;
	}();
	if (is_sag) {
		/// the brightest image may have small sag at center
		if (tmp[tri_adv] >= abstract.background_gray
			&& tmp[center_min_idx] >= abstract.background_gray
			&& tmp[bri_adv] >= abstract.background_gray) {
			return fc_lvl_0_3;
		}
		const int ref_idx = ri.core_center();
		if (ref_idx <= tri_adv || bri_adv <= ref_idx) {
			return fc_lvl_0_2;
		}

		if (ref_idx < ((tri_adv + center_min_idx) / 2)
			|| ref_idx > ((center_min_idx + bri_adv) / 2)) {
			return fc_lvl_0_2;
		}
		return fc_lvl_0_4;
	}

	/// \note test two black line in core
	int trii = tri_adv + 1;
	int brii = bri_adv - 1;
	while (adj[trii + 1] <= 0 && trii < brii) {
		++trii;
	}
	while (adj[brii] >= 0 && trii < brii) {
		--brii;
	}
	if (brii <= trii) {
		log_err("already sag");
		throw img_process_error(img, "already sag", iae_t::img_definition);
	}

	const int bwidth = std::min(trii - tri_adv, bri_adv - brii);

	while (adj[trii + 1] >= 0 && trii < brii) {
		++trii;
	}
	while (adj[brii] <= 0 && trii < brii) {
		--brii;
	}
	if (brii <= trii) {
		/// \note although we find the two black line in core, but
		/// need further confirm
		if (confirm(img, tmp, adj, tri_adv, bri_adv, center_min_idx)) {
			if (bwidth <= 2) {
				return fc_lvl_0_1;
			}
			return fc_lvl_0;
		}
		return fc_lvl_1;
	}
	return fc_lvl_1;
}

int definition_estimate(const gray_img & img, bool is_left)
{
	std::vector<int> wave(0);
	return definition_estimate(img, wave, is_left);
}

} /* namespace img_process */
