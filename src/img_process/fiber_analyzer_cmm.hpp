#pragma once

#include <cmath>

#include "img_buf.hpp"
#include "img_process_cmm.hpp"

#define SAMPLE_POINT_NUM 5
#define JUDGE_FIBER_IN_POINT_NUM (5)

/// \note the fiber width must less than \em half of img height
#define FIBER_MAX_WIDTH_DIV_IMG_HEIGHT (0.49)

#define CLADDING_WIDTH_DIFF_THRESHOLD 4
#define CORE_WIDTH_DIFF_THRESHOLD 2
#define END_CRUDE_THRESHOLD 3
#define WRAP_GRAY_DIFF_THRESHOLD 10

#define BG_GRAY_DIFF_THRESHOLD (0.125)

#define FIND_ARC_GRAY_DIFF_THRESHOLD 30
#define PRECISE_CALIBRATING_GRAY_DIFF_THRESHOLD 10
#define SPLITER_DIFF 10
#define DIST_THRESHOLD 20

namespace img_process {

/**
 * \note the caller should initialize min <= max.
 */
template< typename _T >
static inline void update_minmax(_T & min, _T & max, const _T tmp)
{
	if (tmp < min) {
		min = tmp;
	}
	else if (max < tmp) {
		max = tmp;
	}
}

/**
 * \note the caller should initialize min.
 */
template< typename _T >
static inline void update_min(_T & min, const _T tmp)
{
	if (tmp < min) {
		min = tmp;
	}
}

/**
 * \note the caller should initialize min.
 */
template< typename _T >
static inline void update_max(_T & max, const _T tmp)
{
	if (max < tmp) {
		max = tmp;
	}
}

template< typename _T >
struct coord_pair_t final {
	_T min;
	_T max;
public:
	_T center()
	{
		return (min + max) / 2;
	}

	_T diff()
	{
		return (max - min);
	}
};

/**
 * \note don't use uninitialized 'row_info'
 * \note don't use big value of _T, it will overflow when calc center.
 */
template< typename _T >
struct row_info final {
	bool is_cladding_valid() const
	{
		return (tro < bro);
	}

	bool is_valid() const
	{
		return (tro <= tri && tri < bri && bri <= bro);
	}

	_T cladding_width() const
	{
		return (bro - tro);
	}

	_T cladding_center() const
	{
		return (bro + tro) / 2;
	}

	_T core_width() const
	{
		return (bri - tri);
	}

	_T core_center() const
	{
		return (bri + tri) / 2;
	}

	_T tro;
	_T tri;
	_T bri;
	_T bro;
};

/**
 * \section interfaces
 */
bool is_fiber_in(const gray_img & img, bool is_left);

int get_fiber_vertex(
	const gray_img & img,
	const img_abstract_t abstract,
	bool is_left,
	const int ref_spliter = 0);

double bmp_calc_core_y(
	const gray_img & img,
	const int spliter,
	const int trow, const int brow,
	const int leftx, const int width,
	const bool is_left);

/**
 * \brief row_info serial interfaces
 * \note will not validate the parameters, caller should check by itself.
 * \note will try their best, e.g. it will calc cladding info even core is invalid.
 */
row_info<int> get_row_info(
	const gray_img & img,
	int y0, int y1,
	int x, int spliter);
row_info<double> promote_row_info(
	const gray_img & img,
	const int spliter,
	const int x,
	const row_info<int>  & src);

row_info<std::size_t> get_row_info(
	const std::vector<int> & data,
	const int spliter);
row_info<double> promote_row_info(
	const gray_img & img,
	const std::vector<int> & data,
	const int spliter,
	const row_info<std::size_t>  & src);

/**
 * \note the caller should ensure value in 'data' is small enough,
 * and sum of all values in 'data' will not overflow.
 * generally, the values in 'data' represents 'times', it means something
 * appears 'value' times; and the index of 'data' is the 'something'.
 */
inline std::size_t get_median_pos(const std::vector<std::size_t> & data)
{
	std::size_t sum = 0;
	for (auto i : data) {
		sum += i;
	}

	std::size_t half_sum = sum / 2;
	std::size_t tmp = 0;
	for (std::size_t i = 0; i < data.size(); ++i) {
		tmp += data[i];
		if (tmp >= half_sum) {
			return i;
		}
	}

	throw std::range_error("get_median_pos");
}

/**
 * \brief get image gray scale of specific part
 */
int part_gray_scale(const gray_img & img,
		int x, int y, int w, int h);

/**
 * \note find continuous cnt * 2 columns, but only use cnt columns.
 * \param is_left  true: find from right to left
 *                 false: find from left to right
 * \param cnt      the number of continuous valid points
 * \param valid_lx the most left valid point
 */
bool find_cont_valid_cols(const std::vector<bool> & col_valid, const bool is_left, const int cnt, int & valid_lx);

} /* namesp  img_process */
