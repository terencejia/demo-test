#include "img_process.hpp"

namespace img_process {

void to_wave_form(const gray_img & img, std::vector<uint8_t> & wave_form, img_dir flag)
{
	wave_form.resize(img.height);

	uint32_t cnt;

	if (flag == img_dir::y_img)
	{
		for (decltype(img.height) l = 0; l < img.height; ++l) {
			cnt = 0;
			for (int c = 0; c < 10; ++c) {
				cnt += img.at(c, l);
			}
			wave_form[l] = static_cast<uint8_t>(cnt / 10);
		}
	}
	else if (flag == img_dir::x_img)
	{
		for (decltype(img.height) l = 0; l < img.height; ++l) {
			cnt = 0;
			for (int c = img.width-10; c < img.width; ++c) {
				cnt += img.at(c, l);
			}
			wave_form[l] = static_cast<uint8_t>(cnt / 10);
		}
	}
}

}
