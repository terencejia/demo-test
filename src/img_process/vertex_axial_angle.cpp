#include "img_process.hpp"
#include "fiber_analyzer_cmm.hpp"

#include "math_ext.hpp"

namespace img_process {

/**
 * \brief get the vertex of left/right fiber's offset to core center
 * \return unit: pixel
 */
static double vertex_off(const gray_img & img, const ipp_t & ipp, const bool is_left)
{
	const auto abstract = get_abstract(img, ipp,
					   (is_left ? 0 : (img.width - SAMPLE_POINT_NUM)),
					   SAMPLE_POINT_NUM);
	if (!abstract.has_fiber()) {
		return 0;
	}

	const int trow = abstract.ext_top_row(0);
	const int brow = abstract.ext_bottom_row(img.height - 1);
	const int spliter = abstract.split_gray();
	const int lgx = get_fiber_vertex(img, abstract, is_left);
	const int bgx = lgx + (is_left ? 1 : -1);

	if (lgx < 0 || img.width <= lgx
	    || bgx < 0 || img.width <= bgx) {
		return 0;
	}

	/// find the y of vertex
	int vy = abstract.cladding_center();
	double x_off = -1;
	for (int y = trow; y <= brow; ++y) {
		const int lg = img.at(lgx, y);
		if (lg < spliter) {
			const int bg = img.at(bgx, y);
			double tmp = static_cast<double>(spliter - lg) / (bg - lg);
			if (tmp > x_off) {
				x_off = tmp;
				vy = y;
			}
		}
	}

	/// find the core center
	const int leftx = (is_left ? 0 : lgx);
	const int width = (is_left ? lgx : (img.width - lgx));
	const double core_center = bmp_calc_core_y(img, spliter, trow, brow, leftx, width, is_left);

	return (vy - core_center);
}

double vertex_axial_angle(const fs_imgs & imgs, const ipp_t & ipp)
{
	const double xl = vertex_off(imgs.xz_img, ipp, true);
	const double yl = vertex_off(imgs.yz_img, ipp, true);
	const double xr = vertex_off(imgs.xz_img, ipp, false);
	const double yr = vertex_off(imgs.yz_img, ipp, false);

	if ((xl <= 2.0 && yl <= 2.0)
	    || (xr <= 2.0 && yr <= 2.0)) {
		return 0.0;
	}

	const double ldeg = rad2deg(std::atan2(yl, xl));
	const double rdeg = rad2deg(std::atan2(yr, xr));

	const double diff_deg = std::abs(ldeg - rdeg);

	return (diff_deg <= 180.0 ? diff_deg : (360.0 - diff_deg));
}

}
