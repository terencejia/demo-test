#include <cstdlib>

#include <opencv2/imgproc/imgproc.hpp>

#include <fs_log.hpp>

#include "img_process.hpp"

#include "fiber_analyzer_cmm.hpp"

namespace img_process {

static pixel_t get_median_gray(const gray_img & img, const std::vector<int> & wave)
{
	/// record the appear times for every gray value
	std::array<int, pixel_lvl> cnt;
	cnt.fill(0);
	for (auto i : wave) {
		if (0 <= i && i <= pixel_max) {
			++cnt[i];
		}
		else {
			throw img_process_error(img, "invalid gray value", iae_t::get_median_gray);
		}
	}

	const int half_total_points = wave.size() / 2;
	int sum = 0;
	int i = 0;
	while (i < pixel_lvl) {
		sum += cnt[i];
		if (sum >= half_total_points) {
			break;
		}
		++i;
	}
	return static_cast<pixel_t>(i);
}

static bool has_darker_point(const gray_img & img,
		      const int trow,
		      const int brow,
		      const int x,
		      const int spliter)
{
	for (int y = trow; y <= brow; ++y) {
		if (img.at(x, y) < spliter) {
			return true;
		}
	}

	return false;
}

static bool all_darker(const gray_img & img,
		      const int lcol,
		      const int rcol,
		      const int y,
		      const int spliter)
{
	for (int x = lcol; x <= rcol; ++x) {
		if (img.at(x, y) >= spliter) {
			return false;
		}
	}

	return true;
}

int get_fiber_vertex(const gray_img & img,
		     const img_abstract_t abstract,
		     const bool is_left,
		     const int ref_spliter)
{
	const int trow = abstract.ext_top_row(0);
	const int brow = abstract.ext_bottom_row(img.height - 1);
	int spliter = abstract.split_gray();
	if (ref_spliter) {
		if (std::abs(ref_spliter - spliter) <= SPLITER_DIFF) {
			spliter = ref_spliter;
		}
		else {
			log_err("get_fiber_vertex spliter error: ref(%d), real(%d)", ref_spliter, spliter);
		}
	}

	if (is_left) {
		for (int x = 0; x < img.width; ++x) {
			if (!has_darker_point(img, trow, brow, x, spliter)) {
				return (x - 1);
			}
		}
		return (img.width - 1);
	}
	else {
		for (int x = img.width - 1; x >= 0; --x) {
			if (!has_darker_point(img, trow, brow, x, spliter)) {
				return (x + 1);
			}
		}
		return 0;
	}
}

static double promote_fiber_vertex(const gray_img & img,
		     const img_abstract_t abstract,
		     const int vertex,
		     const bool is_left,
		     const int ref_spliter = 0)
{
	const int ref_x = (is_left ? (vertex + 1) : (vertex - 1));
	if (ref_x < 0 || img.width <= ref_x) {
		return vertex;
	}

	const int trow = abstract.ext_top_row(0);
	const int brow = abstract.ext_bottom_row(img.height - 1);
	int spliter = abstract.split_gray();
	if (ref_spliter) {
		if (std::abs(ref_spliter - spliter) <= SPLITER_DIFF) {
			spliter = ref_spliter;
		}
		else {
			log_err("promote_fiber_vertex spliter error: ref(%d), real(%d)", ref_spliter, spliter);
		}
	}

	static constexpr int extend_pixel = 3;
	const int roi_x = std::max(vertex - extend_pixel, 0);
	const int roi_y = trow;
	const int roi_w = extend_pixel * 2 + 1;
	const int roi_h = brow - trow + 1;
	gray_img filter_img(img.width, img.height);
	img_process::bilateral_filter(filter_img, img, roi_x, roi_y, roi_w, roi_h);

	double max_offset = 0;
	for (int i = roi_y; i < (roi_y + roi_h); ++i) {
		const auto lg = filter_img.at(vertex, i);
		if (lg < spliter) {
			const auto beg = filter_img.at(ref_x, i);
			if (beg >= spliter) {
				const double new_offset = static_cast<double>(spliter - lg) / (beg - lg);
				update_max(max_offset, new_offset);
			}
		}
	}
	return (is_left ? (vertex + max_offset) : (vertex - max_offset));
}

double left_vertex_fine_pos(const gray_img & img, const ipp_t & ipp, const int ref_spliter)
{
	const auto abstract = get_left_abstract(img, ipp);
	return left_vertex_fine_pos(img, abstract, ref_spliter);
}

double right_vertex_fine_pos(const gray_img & img, const ipp_t & ipp, const int ref_spliter)
{
	auto abstract = get_right_abstract(img, ipp);
	return right_vertex_fine_pos(img, abstract, ref_spliter);
}

double left_vertex_fine_pos(const gray_img & img, const img_abstract_t & img_abs, const int ref_spliter)
{
	if (!img_abs.has_fiber()) {
		return 0;
	}

	const int vertex = get_fiber_vertex(img, img_abs, true, ref_spliter);

	return promote_fiber_vertex(img, img_abs, vertex, true, ref_spliter);
}

double right_vertex_fine_pos(const gray_img & img, const img_abstract_t & img_abs, const int ref_spliter)
{
	if (!img_abs.has_fiber()) {
		return (img.width - 1);
	}

	const int vertex = get_fiber_vertex(img, img_abs, false, ref_spliter);

	return promote_fiber_vertex(img, img_abs, vertex, false, ref_spliter);
}

int left_hcenter(const gray_img & img, const ipp_t & ipp)
{
	auto abstract = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM);
	if (!abstract.has_fiber()) {
		return (img.height / 2);
	}

	return abstract.cladding_center();
}

int right_hcenter(const gray_img & img, const ipp_t & ipp)
{
	auto abstract = get_abstract(img, ipp,
				img.width - SAMPLE_POINT_NUM,
				SAMPLE_POINT_NUM);
	if (!abstract.has_fiber()) {
		return (img.height / 2);
	}

	return abstract.cladding_center();
}

bool is_lf_in(const gray_img & img, const ipp_t & ipp)
{
	auto abstract = get_abstract(img, ipp, 0, JUDGE_FIBER_IN_POINT_NUM, true);
	return abstract.has_fiber();
}

bool is_rf_in(const gray_img & img, const ipp_t & ipp)
{
	auto abstract = get_abstract(img, ipp, img.width - JUDGE_FIBER_IN_POINT_NUM, JUDGE_FIBER_IN_POINT_NUM, true);
	return abstract.has_fiber();
}

int left_vertex_pos(const gray_img & img, const ipp_t & ipp, bool omit_width)
{
	const auto abstract = get_left_abstract(img, ipp, omit_width);
	if (!abstract.has_fiber()) {
		return 0;
	}

	return get_fiber_vertex(img, abstract, true);
}

int right_vertex_pos(const gray_img & img, const ipp_t & ipp, bool omit_width)
{
	auto abstract = get_right_abstract(img, ipp, omit_width);
	if (!abstract.has_fiber()) {
		return (img.width - 1);
	}
	return get_fiber_vertex(img, abstract, false);
}

img_abstract_t get_abstract(const gray_img & img, const ipp_t & ipp, int leftx, int width, bool omit_width)
{
	const int rightx = leftx + width;
	if (leftx < 0 || width < 1 || img.width < rightx) {
		log_err("get_abstract: %d %d %d", leftx, width, img.width);
		throw img_process_error(img, "get_abstract param error", iae_t::param_error);
	}

	const int ref_bg = static_cast<int>(ipp.bg * pixel_max);
	img_abstract_t ret;

	/// calc wave
	std::vector<int> tmp;
	tmp.resize(0);
	tmp.resize(img.height, 0);

	std::vector<int> line_tmp;
	line_tmp.resize(0);
	line_tmp.resize(width, 0);
	for (int y = 0; y < img.height; ++y) {
		for (int x = 0; x < width; ++x) {
			line_tmp[x] = img.at(x + leftx, y);
		}
		std::nth_element(line_tmp.begin(), line_tmp.begin() + line_tmp.size()/2, line_tmp.end());
		tmp[y] = line_tmp[line_tmp.size()/2];
	}

	/// find background gray
	int trow = 0;
	const int tul = tmp[0] * 2;
	const int tll = tmp[0] / 2;
	while (trow < static_cast<int>(tmp.size() - 1)) {
		if (tmp[trow] >= tul || tmp[trow] <= tll) {
			break;
		}
		++trow;
	}

	int brow = tmp.size() - 1;
	const int bul = tmp[img.height - 1] * 2;
	const int bll = tmp[img.height - 1] / 2;
	while (brow > 0) {
		if (tmp[brow] >= bul || tmp[brow] <= bll) {
			break;
		}
		--brow;
	}

	///\todo better way to judge too low brightness
	if (tmp[0] <= 30 && tmp[img.height - 1] <= 30) {
		/// no fiber, low led brightness
		ret.background_gray = get_median_gray(img, tmp);
		ret.cladding_gray = ret.background_gray;
		return ret;
	}

	if (brow < img.height / 2) {	/// \note bottom half is background
		if (tmp[brow] >= bul) {	/// \note ensure bottom half is background
			throw img_process_error(img, "bottom half is not background", iae_t::get_abstract_info);
		}
		const int brow_diff = img.height/2 - (brow + 1);
		const int bstart = brow + 1;
		const int bend = (brow_diff > 10 ? (img.height/2 + brow_diff) : (bstart + 2 * 10));
		const std::vector<int> bgv_bot(tmp.begin() + bstart, tmp.begin() + bend);
		ret.background_gray = get_median_gray(img, bgv_bot);
	}
	else if (trow > img.height / 2) {	/// \note top half is background
		if (tmp[trow] >= tul) {		/// \note ensure top half is background
			throw img_process_error(img, "top half is not background", iae_t::get_abstract_info);
		}
		const int trow_diff = trow - img.height/2;
		const int tend = trow;
		const int tstart = (trow_diff > 10 ? (img.height/2 - trow_diff) : (tend - 2 * 10));
		const std::vector<int> bgv_top(tmp.begin() + tstart, tmp.begin() + tend);
		ret.background_gray = get_median_gray(img, bgv_top);
	}
	else {	/// \note top and bottom are all background
		static_assert(FIBER_MAX_WIDTH_DIV_IMG_HEIGHT < 0.5, "fiber max width must less than 0.5 * image_height");
		if (!omit_width) {
			if (tmp[trow] >= tul || tmp[brow] >= bul	/// \note ensure top and bottom are all background
			|| (brow - trow + 1) > (img.height * FIBER_MAX_WIDTH_DIV_IMG_HEIGHT)) {
				throw img_process_error(img, "fiber too wide", iae_t::get_abstract_info);
			}
		}
		const std::vector<int> bgv_bot(tmp.begin() + brow + 1, tmp.end());
		const pixel_t mg_bot = get_median_gray(img, bgv_bot);
		const std::vector<int> bgv_top(tmp.begin(), tmp.begin() + trow);
		const pixel_t mg_top = get_median_gray(img, bgv_top);
		ret.background_gray = (mg_bot * (img.height - trow) + mg_top * (brow + 1)) / (brow + img.height - trow + 1);
	}

	/// fix bg gray
	if (ref_bg) {
		if (std::abs((ret.background_gray - ref_bg) / static_cast<double>(ref_bg)) < BG_GRAY_DIFF_THRESHOLD) {
			ret.background_gray = ref_bg;
		}
		else {
			log_warning("%s: x(%d+%d) refbg(%d) realbg(%d)", __func__, leftx, width, ref_bg, ret.background_gray);
		}
	}

	/// find top/bottom row
	/// reinit trow/brow, \note we can optimize it according the relation of tlen and blen
	trow = 0;
	brow = tmp.size() - 1;
	const int spliter = static_cast<int>(ret.background_gray * (1 - 2 * BG_GRAY_DIFF_THRESHOLD));
	while ((trow < img.height - 1)
	    && !all_darker(img, leftx, rightx - 1, trow, spliter)) {
		++trow;
	}
	while ((brow > trow)
	    && !all_darker(img, leftx, rightx - 1, brow, spliter)) {
		--brow;
	}

	if (brow <= trow) {
		/// no fiber
		ret.cladding_gray = ret.background_gray;
	}
	else {
		ret.top_row = trow;
		ret.bottom_row = brow;
		/// \note for fpga bug, using tearing image,
		/// we can not ensure the cladding_gray is always correct.
		const auto mine = std::min_element(tmp.begin() + trow, tmp.begin() + brow + 1);
		ret.cladding_gray = *mine;
	}

	return ret;
}

img_abstract_t get_left_abstract(const gray_img & img, const ipp_t & ipp, bool omit_width)
{
	return get_abstract(img, ipp, 0, SAMPLE_POINT_NUM, omit_width);
}

img_abstract_t get_right_abstract(const gray_img & img, const ipp_t & ipp, bool omit_width)
{
	return get_abstract(img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM, omit_width);
}

row_info<int> get_row_info(const gray_img & img, int y0, int y1, int x, int spliter)
{
	/// cladding
	int tro = y0;
	while (tro <= y1) {
		if (img.at(x, tro) < spliter) {
			break;
		}
		++tro;
	}

	int bro = y1;
	while (bro >= tro) {
		if (img.at(x, bro) < spliter) {
			break;
		}
		--bro;
	}

	/// core
	int tri = tro;
	while (tri < tro) {
		if (img.at(x, tri + 1) >= spliter) {
			break;
		}
		++tri;
	}

	int bri = bro;
	while (bri > tri) {
		if (img.at(x, bri - 1) >= spliter) {
			break;
		}
		--bri;
	}

	return { tro, tri, bri, bro, };
}

row_info<std::size_t> get_row_info(
	const std::vector<int> & data,
	const int spliter)
{
	const std::size_t y0 = 0;
	const std::size_t y1 = data.size() - 1;

	std::size_t tro = y0;
	std::size_t tri = y0;
	std::size_t bri = y1;
	std::size_t bro = y1;

	for (std::size_t y = y0; y <= y1; ++y) {
		if (data[y] < spliter) {
			tro = y;
			break;
		}
	}

	for (std::size_t y = tro; y <= y1; ++y) {
		if (data[y] >= spliter) {
			tri = y - 1;
			break;
		}
	}

	for (std::size_t y = y1; y >= tri; --y) {
		if (data[y] < spliter) {
			bro = y;
			break;
		}
	}

	for (std::size_t y = bro; y >= tri; --y) {
		if (data[y] >= spliter) {
			bri = y + 1;
			break;
		}
	}

	return { tro, tri, bri, bro, };
}

row_info<double> promote_row_info(
	const gray_img & img,
	const std::vector<int> & data,
	const int spliter,
	const row_info<std::size_t> & src)
{
	if (data.size() <= src.bro) {
		throw img_process_error(img, "promote_row_info param error", iae_t::param_error);
	}

	double tro = src.tro;
	double tri = src.tri;
	double bri = src.bri;
	double bro = src.bro;

	/// tro
	if (0 < src.tro) {
		int bes = data[src.tro - 1];
		int ls = data[src.tro];
		if (ls < spliter && spliter <= bes) {
			tro -= (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	/// tri
	if (src.tri < (data.size() - 1))	{
		int bes = data[src.tri + 1];
		int ls = data[src.tri];
		if (ls < spliter && spliter <= bes) {
			tri += (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	/// bri
	if (src.bri > 0) {
		int bes = data[src.bri - 1];
		int ls = data[src.bri];
		if (ls < spliter && spliter <= bes) {
			bri -= (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	/// bro
	if (src.bro < (data.size() - 1)) {
		int bes = data[src.bro + 1];
		int ls = data[src.bro];
		if (ls < spliter && spliter <= bes) {
			bro += (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	return { tro, tri, bri, bro, };
}

row_info<double> promote_row_info(const gray_img & img,
			const int spliter,
			const int x,
			const row_info<int>  & src)
{
	if (src.tro < 0 || img.height <= src.bro) {
		throw img_process_error(img, "promote_row_info param error", iae_t::param_error);
	}

	double tro = src.tro;
	double tri = src.tri;
	double bri = src.bri;
	double bro = src.bro;

	/// tro
	if (0 < src.tro) {
		int bes = img.at(x, src.tro - 1);
		int ls = img.at(x, src.tro);
		if (ls < spliter && spliter <= bes) {
			tro -= (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	/// tri
	if (src.tri < (img.height - 1))	{
		int bes = img.at(x, src.tri + 1);
		int ls = img.at(x, src.tri);
		if (ls < spliter && spliter <= bes) {
			tri += (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	/// bri
	if (src.bri > 0) {
		int bes = img.at(x, src.bri - 1);
		int ls = img.at(x, src.bri);
		if (ls < spliter && spliter <= bes) {
			bri -= (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	/// bro
	if (src.bro < (img.height - 1)) {
		int bes = img.at(x, src.bro + 1);
		int ls = img.at(x, src.bro);
		if (ls < spliter && spliter <= bes) {
			bro += (spliter - ls) / static_cast<double>(bes - ls);
		}
	}

	return { tro, tri, bri, bro, };
}

int get_spliter(const gray_img & img, const ipp_t & ipp)
{
	const auto labs = get_abstract(img, ipp, 0, SAMPLE_POINT_NUM);
	const auto rabs = get_abstract(img, ipp, img.width - SAMPLE_POINT_NUM, SAMPLE_POINT_NUM);
	if (labs.has_fiber()) {
		if (rabs.has_fiber()) {
			const int lspliter = labs.split_gray();
			const int rspliter = rabs.split_gray();
			if (std::abs(lspliter - rspliter) <= SPLITER_DIFF) {
				return (lspliter + rspliter) / 2;
			}
			else {
				log_err("%s left(%d), right(%d) diff too big", __func__, lspliter, rspliter);
				return 0;
			}
		}
		else {
			return labs.split_gray();
		}
	}
	else if (rabs.has_fiber()) {
		return rabs.split_gray();
	}
	else {
		log_err("%s no fiber", __func__);
		return 0;
	}

	return labs.split_gray();
}

int part_gray_scale(const gray_img & img,
		int x, int y, int w, int h)
{
	const int xmin = std::max(0, x);
	const int ymin = std::max(0, y);
	const int xmax = std::min(x + w, img.width);
	const int ymax = std::min(y + h, img.height);

	std::vector<std::size_t> cnt;
	cnt.resize(pixel_lvl, 0);

	for (int y = ymin; y < ymax; ++y) {
		for (int x = xmin; x < xmax; ++x) {
			++cnt[img.at(x,y)];
		}
	}

	return static_cast<int>(get_median_pos(cnt));
}

} /* namespace img_process */
