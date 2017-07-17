#include <opencv2/imgproc/imgproc.hpp>

#include "img_process.hpp"
#include "fiber_analyzer_cmm.hpp"

namespace img_process {

void check_dust(const gray_img & img, double th0, double th1, gray_img & result)
{
	gray_img & dst = result;
	dst.reset(img.width, img.height);

	const cv::Mat gray(img.height, img.width, CV_8UC1, (void *)img.data());
	cv::Mat edge(img.height, img.width, CV_8UC1, (void *)dst.data());
	cv::blur(gray, edge, cv::Size(3,3));
	Canny(edge, edge, th0, th1, 3);

	return;
}

///\brief we think the img is dirty even only one dirty point
bool is_img_clean(const gray_img & img)
{
	const pixel_t * pdata = img.buf.data();

	for (int i = 0; i < img.width * img.height; ++i) {
		if (pdata[i]) {
			return false;
		}
	}

	return true;
}

}
