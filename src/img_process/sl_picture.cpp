#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <png.h>

#include <string>
#include <fstream>

#include "fsconf.h"
#include "img_process.hpp"

namespace img_process {

void save_pgm(const std::string & path, const gray_img & src)
{
	save_pgm(path.c_str(), src);
}

void save_pgm(const char * path, const gray_img & src)
{
	std::string full_path;
#ifdef CFG_DEBUG_FILE_DIR
	if (path[0] != '/') {
		full_path.append(CFG_DEBUG_FILE_DIR);
	}
#endif
	full_path.append(path);

	const mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	int fd = open(full_path.c_str(), O_WRONLY | O_CREAT, mode);
	if (fd <= 0) {
		throw "save pgm";
	}

	char hbuffer[30] = "";
	int hs = std::snprintf(hbuffer, sizeof(hbuffer),
			       "P5\n%d %d\n%d\n", src.width, src.height, pixel_max);

	auto ret = ::pwrite(fd, hbuffer, hs, 0);
	if (ret != hs) {
		throw "hello";
	}
	ret = ::pwrite(fd, src.data(), src.bsize(), hs);
	if (ret != (decltype(ret))src.bsize()) {
		throw;
	}
	close(fd);
}

void load_pgm(const std::string & path, gray_img & dst)
{
	load_pgm(path.c_str(), dst);
}

void load_pgm(const char * path, gray_img & dst)
{
	/// \todo use custom width/height
	dst.reset(320, 240);

	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		throw "load_pgm";
	}

	auto ret = ::pread(fd, dst.data(), dst.bsize(), 15);
	if (ret != (decltype(ret))dst.bsize()) {
		throw;
	}
	close(fd);
}

void save_dust(const std::string & path, const gray_img & dust_data)
{
	save_dust(path.c_str(), dust_data);
}

void save_dust(const char * path, const gray_img & dust_data)
{
	std::string full_path;
#ifdef CFG_DEBUG_FILE_DIR
	if (path[0] != '/') {
		full_path.append(CFG_DEBUG_FILE_DIR);
	}
#endif
	full_path.append(path);

        FILE *fp = fopen(full_path.c_str(), "wb");
        if (!fp) {
                throw path;
	}

        /* initialize stuff */
        png_structp write_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (!write_ptr) {
		throw;
	}

        png_infop write_info_ptr = png_create_info_struct(write_ptr);
        if (!write_info_ptr) {
		throw;
	}

	png_infop write_end_info_ptr = png_create_info_struct(write_ptr);
	if (!write_end_info_ptr) {
		throw;
	}

        png_init_io(write_ptr, fp);

        /// write header info
	const png_uint_32 width = dust_data.width;
	const png_uint_32 height = dust_data.height;
	const png_byte color_type = PNG_COLOR_TYPE_PALETTE;
	const png_byte bit_depth = 1;	/// \note for indexed color png, it is the bit depth of indies.
        png_set_IHDR(write_ptr, write_info_ptr, width, height,
                     bit_depth, color_type, PNG_INTERLACE_NONE,
                     PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	const png_color palette[] = {
		{ 0, 0, 0, },
		{ 255, 0, 0, },
	};
	png_set_PLTE(write_ptr, write_info_ptr, palette, sizeof(palette) / sizeof(palette[0]));

	const png_byte trans_alpha[] = {
		0, 255,
	};
	png_set_tRNS(write_ptr, write_info_ptr,
		trans_alpha, sizeof(trans_alpha) / sizeof(trans_alpha[0]),
		nullptr);

	png_write_info(write_ptr, write_info_ptr);

	/// write content
	png_bytep row_buf = (png_bytep)png_malloc(write_ptr,
			png_get_rowbytes(write_ptr, write_info_ptr));
	for (int y = 0; y < (int)height; ++y)
	{
		png_bytep tmp = row_buf;
		for (int x = 0; x < (int)(width/8); ++x) {
			*tmp = 0x0;
#define XXX(x, y, off) \
do { \
	if (dust_data.at(x * 8 + 7 - off, y)) { \
		*tmp |= (1 << off); \
	} \
} while (0)
			XXX(x, y, 0);
			XXX(x, y, 1);
			XXX(x, y, 2);
			XXX(x, y, 3);
			XXX(x, y, 4);
			XXX(x, y, 5);
			XXX(x, y, 6);
			XXX(x, y, 7);
#undef XXX

			++tmp;
		}
		png_write_rows(write_ptr, &row_buf, 1);
	}
	png_free(write_ptr, row_buf);

	/// clear
	png_write_end(write_ptr, write_end_info_ptr);
	png_destroy_info_struct(write_ptr, &write_end_info_ptr);
	png_destroy_write_struct(&write_ptr, &write_info_ptr);

        fclose(fp);
}

void save_defects(const std::string & path, const gray_img & defects)
{
	save_defects(path.c_str(), defects);
}
void save_defects(const char * path, const gray_img & defects)
{
	/// TODO: maybe we should improve it, but not use save_dust directly.
	save_dust(path, defects);
}

void save_wave(const std::string & path, const std::vector<uint8_t> & wave, int width, img_dir dir)
{
	save_wave(path.c_str(), wave, width, dir);
}
void save_wave(const char * path, const std::vector<uint8_t> & wave, int width, img_dir dir)
{
	std::string full_path;
#ifdef CFG_DEBUG_FILE_DIR
	if (path[0] != '/') {
		full_path.append(CFG_DEBUG_FILE_DIR);
	}
#endif
	full_path.append(path);

	std::ofstream svg;
	svg.open(full_path);

	/// header
	svg << R"XXX(<?xml version="1.0" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN"
"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">

<svg width=")XXX";
	svg << width;
	svg << R"XXX(%" height=")XXX";
	svg << wave.size();
	svg << R"XXX(%" version="1.1"
xmlns="http://www.w3.org/2000/svg">
)XXX";

	svg << R"XXX(<polyline points=")XXX";
	for (int i = 0; i < (int)wave.size(); ++i) {
		switch (dir) {
		case img_dir::y_img:
			svg << std::to_string(wave[i]) << "," << std::to_string(i) << " ";
			break;
		case img_dir::x_img:
			svg << std::to_string(width - wave[i]) << "," << std::to_string(i) << " ";
			break;
		}

	}
	/// tail
	svg << R"XXX("
style="stroke:green;stroke-width:2"/>
)XXX";
	svg << R"XXX(
</svg>
)XXX";
	svg.close();
}

}
