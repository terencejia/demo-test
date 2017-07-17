#include "math_ext.hpp"
#include "fs_log.hpp"
#include "img_process.hpp"

static constexpr int vratio = 6;
static constexpr int hratio = 6;

static constexpr int vr1 = 9;
static constexpr int vr2 = 3;

namespace img_process {

region_t get_left_fiber_region_for_identify(const gray_img & img)
{
	const ipp_t ipp = {0};
	region_t ret = { 0, 0, 0, 0, };
	const auto abstract = img_process::get_left_abstract(img, ipp);
	if (!abstract.has_fiber()) {
		return ret;
	}

	const auto vertex = left_vertex_pos(img, ipp);

	const auto cladding_width = abstract.cladding_width();
	const auto vreserve = cladding_width / vratio;
	const auto hreserve = vertex / hratio;

	ret.y = abstract.top_row + vreserve;
	ret.h = abstract.bottom_row - vreserve - ret.y;
	ret.x = hreserve;
	ret.w = vertex - hreserve - ret.x;

	return ret;
}

region_t get_right_fiber_region_for_identify(const gray_img & img)
{
	const ipp_t ipp = {0};
	region_t ret = { 0, 0, 0, 0, };
	const auto abstract = img_process::get_right_abstract(img, ipp);
	if (!abstract.has_fiber()) {
		return ret;
	}

	const auto vertex = right_vertex_pos(img, ipp);

	const auto cladding_width = abstract.cladding_width();
	const auto vreserve = cladding_width / vratio;
	const auto hreserve = (img.width - vertex) / hratio;

	ret.y = abstract.top_row + vreserve;
	ret.h = abstract.bottom_row - vreserve - ret.y;
	ret.x = vertex + hreserve;
	ret.w = img.width - hreserve - ret.x;

	return ret;
}

fiber_model identify_left_fiber_model(const gray_img & img, const region_t & region)
{
	int cnt_gezero = 0;
	int cnt_valid = 0;
	for (int i = 0; i < region.w; ++i) {
		std::vector<mathext::point_t> points;
		for (int j = 0; j < region.h; ++j) {
			points.push_back({ (double)j, (double)img.at(region.x+i, region.y+j), });
		}

		std::vector<double> coefficient;
		mathext::multifit_robust(points, 2, coefficient);
		if (coefficient.size() > 2) {
			++cnt_valid;
			if (coefficient[2] > 0) {
				++cnt_gezero;
			}
		}
	}

	log_info("left fiber: %d/%d", cnt_gezero, cnt_valid);

	/// \todo return value according cnt;
	return fiber_model::G651;
}

fiber_model identify_right_fiber_model(const gray_img & img, const region_t & region)
{
	int cnt_gezero = 0;
	int cnt_valid = 0;
	for (int i = 0; i < region.w; ++i) {
		std::vector<mathext::point_t> points;
		for (int j = 0; j < region.h; ++j) {
			points.push_back({ (double)j, (double)img.at(region.x+i, region.y+j), });
		}

		std::vector<double> coefficient;
		mathext::multifit_robust(points, 2, coefficient);
		if (coefficient.size() > 2) {
			++cnt_valid;
			if (coefficient[2] > 0) {
				++cnt_gezero;
			}
		}
	}

	log_info("right fiber: %d/%d", cnt_gezero, cnt_valid);

	/// \todo return value according cnt;
	return fiber_model::G651;
}

} /* namespace img_process */
