#include "fsconf.h"

#ifdef CFG_RECORD_VIDEO

/// file oper
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>

#include <fs_log.hpp>

#include "gray_video.hpp"

#define IVF_FILE_HDR_SZ  (32)
#define IVF_FRAME_HDR_SZ (12)

static void mem_put_le16(uint8_t *mem, uint16_t val) {
    mem[0] = (uint8_t)(val & 0xFF);
    mem[1] = (uint8_t)(val >> 8);
}

static void mem_put_le32(uint8_t *mem, uint32_t val) {
    mem[0] = (uint8_t)(val);
    mem[1] = (uint8_t)(val>>8);
    mem[2] = (uint8_t)(val>>16);
    mem[3] = (uint8_t)(val>>24);
}

static void write_ivf_file_header(int outfile,
			const vpx_codec_enc_cfg_t *cfg,
			int frame_cnt)
{
	if (cfg->g_pass != VPX_RC_ONE_PASS
		&& cfg->g_pass != VPX_RC_LAST_PASS) {
		return;
	}

	constexpr uint32_t fourcc  = 0x30385056;	/// VP80
	uint8_t header[32];
	header[0] = 'D';
	header[1] = 'K';
	header[2] = 'I';
	header[3] = 'F';
	mem_put_le16(header+4,  0);                   /* version */
	mem_put_le16(header+6,  32);                  /* headersize */
	mem_put_le32(header+8,  fourcc);              /* headersize */
	mem_put_le16(header+12, (uint16_t)cfg->g_w);            /* width */
	mem_put_le16(header+14, (uint16_t)cfg->g_h);            /* height */
	mem_put_le32(header+16, (uint32_t)cfg->g_timebase.den); /* rate */
	mem_put_le32(header+20, (uint32_t)cfg->g_timebase.num); /* scale */
	mem_put_le32(header+24, (uint32_t)frame_cnt);           /* length */
	mem_put_le32(header+28, (uint32_t)0);                   /* unused */

	auto ret = ::pwrite(outfile, header, sizeof(header), 0);
	if (ret != sizeof(header)) {
		throw "write video file header fail";
	}
}

static void write_ivf_frame(int outfile,
				const vpx_codec_cx_pkt_t *pkt)
{
	if (pkt->kind != VPX_CODEC_CX_FRAME_PKT) {
		return;
	}

	const vpx_codec_pts_t pts = pkt->data.frame.pts;
	uint8_t header[12];
	mem_put_le32(header, (uint32_t)pkt->data.frame.sz);
	mem_put_le32(header+4, (uint32_t)(pts & 0xFFFFFFFF));
	mem_put_le32(header+8, (uint32_t)(pts >> 32));

	auto ret = ::write(outfile, header, sizeof(header));
	if (ret < 0 || ret != sizeof(header)) {
		throw "write video file frame header fail";
	}

	ret = ::write(outfile, pkt->data.frame.buf, pkt->data.frame.sz);
	if (ret < 0 || (size_t)ret != pkt->data.frame.sz) {
		throw "write video file frame content fail";
	}
}

gray_video::gray_video(
	const char * path,
	const int width,
	const int height)
: m_iface(vpx_codec_vp8_cx())
, m_frame_cnt(0)
{
	/// Populate encoder configuration
	vpx_codec_err_t res = vpx_codec_enc_config_default(m_iface, &m_cfg, 0);
	if (res) {
		throw vpx_codec_err_to_string(res);
	}
	/// Update the default configuration with our settings
	log_debug("vp8 encode - default bitrate %d", m_cfg.rc_target_bitrate);
	m_cfg.rc_target_bitrate = width * height * m_cfg.rc_target_bitrate / m_cfg.g_w / m_cfg.g_h;
	m_cfg.g_w = width;
	m_cfg.g_h = height;
	m_cfg.g_timebase = { 1, 30, };

	/// Initialize codec
	if (vpx_codec_enc_init(&m_ctx, m_iface, &m_cfg, 0)) {
		const char *detail = vpx_codec_error_detail(&m_ctx);
		if(detail) {
			throw detail;
		}
		else {
			throw vpx_codec_error(&m_ctx);
		}
	}

	auto ret_alloc = vpx_img_alloc(&m_raw, VPX_IMG_FMT_I420, width, height, 1);
	if (!ret_alloc) {
		throw "Faile to allocate image";
	}
	/// \note for gray image, the u/v are always 128
	std::memset(m_raw.planes[0] + width * height, 0x80, width * height / 2);

	/// open out file
	m_fd = ::open(path,
				(O_WRONLY | O_CREAT | O_TRUNC),
				(S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH));
	if (!m_fd) {
		printf("can't open %s\n", path);
		throw path;
	}

	/// skip header for frames
	off_t pos = ::lseek(m_fd, 32, SEEK_SET);
	if (pos < 0 || (pos != 32)) {
		throw "seek video file error";
	}
}

gray_video::~gray_video()
{
	vpx_img_free(&m_raw);
	vpx_codec_destroy(&m_ctx);

	/// write file header
	if (m_frame_cnt > 0) {
		write_ivf_file_header(m_fd, &m_cfg, (m_frame_cnt-1));
	}
	::close(m_fd);
}

void gray_video::write(const fs_imgs & img)
{
	const int flags = 0;

	const int buf_size = m_cfg.g_w * m_cfg.g_h;
	const int simg_size = img.xz_img.size;
	if (buf_size != simg_size) {
		throw "invalid frame img";
	}

	const int cpy_size = simg_size / 2;
	const int start_offset = simg_size / 4;
	uint8_t * pDst = m_raw.planes[0];
	std::memcpy(pDst, img.xz_img.buf.get() + start_offset, cpy_size);
	std::memcpy(pDst + cpy_size, img.yz_img.buf.get() + start_offset, cpy_size);

	auto ret_encode = vpx_codec_encode(&m_ctx, &m_raw, m_frame_cnt, 1, flags, VPX_DL_REALTIME);
	if (ret_encode) {
		const char *detail = vpx_codec_error_detail(&m_ctx);
		if(detail) {
			throw detail;
		}
		else {
			throw vpx_codec_err_to_string(ret_encode);
		}
	}

	vpx_codec_iter_t iter = NULL;
	const vpx_codec_cx_pkt_t *pkt;
	while ((pkt = vpx_codec_get_cx_data(&m_ctx, &iter))) {
		switch(pkt->kind) {
		case VPX_CODEC_CX_FRAME_PKT:
			write_ivf_frame(m_fd, pkt);
			break;
		default:
			break;
		}
	}

	++m_frame_cnt;
}

#endif
