#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>

#include "img_process.hpp"

namespace img_process {

void median_blur(gray_img & dst, const gray_img & src,
		      int x, int y, int w, int h)
{
	const cv::Mat dst_mat(dst.height, dst.width, CV_8UC1, (void *)dst.data());
	const cv::Mat src_mat(src.height, src.width, CV_8UC1, (void *)src.data());

	if (x < 0) {
		w += x;
		x = 0;
	}
	if ( y < 0) {
		h += y;
		y = 0;
	}

	if (src.width < x + w) {
		w = src.width - x;
	}
	if (src.height < y + h) {
		h = src.height - y;
	}

	if (w > 0 && h > 0) {
		const cv::Rect roi(x, y, w, h);
		const cv::Mat dst_roi = dst_mat(roi);
		const cv::Mat src_roi = src_mat(roi);

		cv::medianBlur(src_roi, dst_roi, 5);
	}
	else {
		cv::medianBlur(src_mat, dst_mat, 5);
	}
}

void bilateral_filter(gray_img & dst, const gray_img & src,
		      int x, int y, int w, int h)
{
	const cv::Mat dst_mat(dst.height, dst.width, CV_8UC1, (void *)dst.data());
	const cv::Mat src_mat(src.height, src.width, CV_8UC1, (void *)src.data());

	if (w != 0 && h != 0) {
		const cv::Rect roi(x, y, w, h);
		const cv::Mat dst_roi = dst_mat(roi);
		const cv::Mat src_roi = src_mat(roi);

		cv::bilateralFilter(src_roi, dst_roi, 3, 10, 2);
	}
	else {
		cv::bilateralFilter(src_mat, dst_mat, 3, 10, 2);
	}
}

void img_denoise(gray_img & dst, const std::vector<gray_img> & src)
{
	if (src.empty()) {
		throw std::invalid_argument("no source img used for denoise");
	}

	std::vector<int> tmp(src[0].psize());
	//std::unique_ptr<int[]> tmp(new int[src[0].size]());

	for (auto & si : src) {
		const pixel_t * sp = si.data();
		int * dp = tmp.data();
		int * dp_end = dp + tmp.size();
		while (dp < dp_end) {
			*dp += *sp;
			++dp;
			++sp;
		}
	}

	dst.reset(src[0].width, src[0].height);
	{
		pixel_t * dp = dst.data();
		pixel_t * dp_end = dp + dst.psize();
		const int * sp = tmp.data();
		while (dp < dp_end) {
			*dp = static_cast<pixel_t>((*sp) / src.size());
			++dp;
			++sp;
		}
	}
}

void copy_img(const gray_img & src, gray_img & dst, int x, int y)
{
	if (dst.width <= x || dst.height <= y) {
		throw std::invalid_argument("invalid position for copy_img");
	}

	cv::Mat dst_mat(dst.height, dst.width, CV_8UC1, (void *)dst.data());
	const cv::Mat src_mat(src.height, src.width, CV_8UC1, (void *)src.data());

	const int w = std::min(src.width, dst.width - x);
	const int h = std::min(src.height, dst.height - y);
	const cv::Rect dst_roi(x, y, w, h);
	const cv::Rect src_roi(0, 0, w, h);

	src_mat(src_roi).copyTo(dst_mat(dst_roi));
}

}

