#include <sys/mman.h>
#include <linux/videodev2.h>

#include <plf_drv/cameractl_dev.h>

#include <algorithm>
#include <limits>

#include "fs_log.hpp"
#include "fs_spec.hpp"

#include "device/camera.hpp"

static constexpr uint32_t merge_x_y(int32_t x, int32_t y)
{
	return ((x << 16) | (y & 0xFFFF));
}

static constexpr int32_t extrac_x(uint32_t pos)
{
	return (pos >> 16);
}

static constexpr int32_t extrac_y(uint32_t pos)
{
	return (pos & 0xFFFF);
}

struct camera::img_waiter_info final {
	explicit img_waiter_info(waiter_t _cb)
	: time_stamp(exemodel::monotonic_clock_info::get_time())
	, callback(_cb)
	{
	}

	explicit img_waiter_info(waiter_t _cb, uint32_t ms)
	: time_stamp(exemodel::timespec_add(exemodel::monotonic_clock_info::get_time(), exemodel::ms_to_timespec(ms)))
	, callback(_cb)
	{
	}

	exemodel::timespec_t const time_stamp;
	waiter_t const callback;
};

camera::camera(const hw_info & hwinfo)
: m_cam(::access("/dev/video0", F_OK) == 0 ? new exemodel::devicee("/dev/video0") : nullptr)
, m_width(0)
, m_height(0)
, m_img_size(0)
, m_buf{{ nullptr, nullptr, nullptr, }}
, m_imgs()
, m_disposing(false)
, m_waiting_list()
, m_disposing_list()
, m_waiting_video_list()
, m_left_max(hwinfo.cmos_full_width - hwinfo.cmos_win_width)
, m_top_max(hwinfo.cmos_full_height - hwinfo.cmos_win_height)
, m_cameractl(hwinfo.cam_ctl_path)
, m_x_win_pos(::access(hwinfo.x_cmos_win_pos_path, F_OK) == 0 ? new exemodel::dev_attr_rw<uint32_t>(hwinfo.x_cmos_win_pos_path) : nullptr)
, m_y_win_pos(::access(hwinfo.y_cmos_win_pos_path, F_OK) == 0 ? new exemodel::dev_attr_rw<uint32_t>(hwinfo.y_cmos_win_pos_path) : nullptr)
{
	__init_video(hwinfo.cmos_win_width, hwinfo.cmos_win_height);
}

camera::~camera()
{
}

void camera::set_window_pos_x(int32_t c, int32_t r)
{
	if (c < 0 || m_left_max < c
		|| r < 0 || m_top_max < r) {
		throw std::range_error("camera's row or col was out of range!");
	}

	if (m_x_win_pos) {
		m_x_win_pos->write(merge_x_y(c, r));
	}
	else {
		imaq_pos_t pos = {0, static_cast<decltype(imaq_pos_t::row)>(r)
				, static_cast<decltype(imaq_pos_t::column)>(c)};
		int ret = m_cameractl.io_ctl(CAMERACTL_IOC_SET_IMAQ_POS, &pos);
		exemodel::validate_ret(ret, "camera set imaq pos x");
	}
}

void camera::set_window_pos_y(int32_t c, int32_t r)
{
	if (c < 0 || m_left_max < c
		|| r < 0 || m_top_max < r) {
		throw std::range_error("camera's row or col was out of range!");
	}

	if (m_y_win_pos) {
		m_y_win_pos->write(merge_x_y(c, r));
	}
	else {
		imaq_pos_t pos = {1, static_cast<decltype(imaq_pos_t::row)>(r)
				, static_cast<decltype(imaq_pos_t::column)>(c)};
		int ret = m_cameractl.io_ctl(CAMERACTL_IOC_SET_IMAQ_POS, &pos);
		exemodel::validate_ret(ret, "camera set imaq pos y");
	}
}

void camera::get_window_pos_x(int32_t & c, int32_t & r)
{
	if (m_x_win_pos) {
		uint32_t pos = m_x_win_pos->read();

		c = extrac_x(pos);
		r = extrac_y(pos);
	}
	else {
		imaq_pos_t pos = { 0, 0, 0, };
		int ret = m_cameractl.io_ctl(CAMERACTL_IOC_GET_IMAQ_POS, &pos);
		exemodel::validate_ret(ret, "camera get imaq pos x");
		r = pos.row;
		c = pos.column;
	}
}

void camera::get_window_pos_y(int32_t & c, int32_t & r)
{
	if (m_y_win_pos) {
		uint32_t pos = m_y_win_pos->read();

		c = extrac_x(pos);
		r = extrac_y(pos);
	}
	else {
		imaq_pos_t pos = { 1, 0, 0, };
		int ret = m_cameractl.io_ctl(CAMERACTL_IOC_GET_IMAQ_POS, &pos);
		exemodel::validate_ret(ret, "camera get imaq pos y");
		r = pos.row;
		c = pos.column;
	}
}

void camera::move_window_pos_x(int32_t c, int32_t r)
{
	int32_t c_ori;
	int32_t r_ori;
	if (m_x_win_pos) {
		uint32_t pos = m_x_win_pos->read_soft();

		c_ori = extrac_x(pos);
		r_ori = extrac_y(pos);
	}
	else {
		this->get_window_pos_x(c_ori, r_ori);
	}

	c_ori += c;
	r_ori += r;
	this->set_window_pos_x(c_ori, r_ori);
}
void camera::move_window_pos_y(int32_t c, int32_t r)
{
	int32_t c_ori;
	int32_t r_ori;
	if (m_y_win_pos) {
		uint32_t pos = m_y_win_pos->read_soft();

		c_ori = extrac_x(pos);
		r_ori = extrac_y(pos);
	}
	else {
		this->get_window_pos_y(c_ori, r_ori);
	}

	c_ori += c;
	r_ori += r;
	this->set_window_pos_y(c_ori, r_ori);
}

void camera::set_window_size(int32_t w, int32_t h)
{
	if (std::numeric_limits<decltype(imaq_size_t::width)>::min() <= w
	    && w <= std::numeric_limits<decltype(imaq_size_t::width)>::max()
	    && std::numeric_limits<decltype(imaq_size_t::length)>::min() <= h
	    && h <= std::numeric_limits<decltype(imaq_size_t::length)>::max()) {

		imaq_size_t size = {static_cast<decltype(imaq_size_t::width)>(w)
				   , static_cast<decltype(imaq_size_t::length)>(h)};

		int ret = m_cameractl.io_ctl(CAMERACTL_IOC_SET_IMAQ_SIZE, &size);
		exemodel::validate_ret(ret, "set_window_size ");
	}
	else {
		log_err("width: %d, height: %d", w, h);
		throw std::range_error("camera's width or height was out of range!");
	}
}

void camera::__init_video(uint32_t width, uint32_t height)
{
	if (!m_cam)
		return;

	int ret;

	set_window_size((int32_t)width, (int32_t)height);

	/// query capabilities
	struct v4l2_capability cap;
	ret = m_cam->io_ctl(VIDIOC_QUERYCAP, &cap);
	exemodel::validate_ret(ret, "VIDIOC_QUERYCAP");
	static constexpr uint32_t expect_cap = (
		  V4L2_CAP_VIDEO_CAPTURE
		| V4L2_CAP_STREAMING);
	if ((cap.capabilities & expect_cap) != expect_cap) {
		throw std::range_error("camera capabilities can't satisfy expected");
	}

	/// set format
	struct v4l2_format fmt;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

	ret = m_cam->io_ctl(VIDIOC_S_FMT, &fmt);
	exemodel::validate_ret(ret, "VIDIOC_S_FMT");

	/// save image related info
	m_width = fmt.fmt.pix.width;
	m_height = fmt.fmt.pix.height;
	m_img_size = fmt.fmt.pix.sizeimage;

	__req_buf();

	/// reset buffer
	m_imgs.reset(m_width, m_height);

	/// regist callback
	m_cam->connect([this](exemodel::poller & p, uint32_t evts) {
			this->__dispose(p, evts);
	});
}

void camera::push_waiter(camera::waiter_t waiter, uint32_t delay_ms)
{
	bool old_empty = this->__empty();
	m_waiting_list.push_back(img_waiter_info(waiter, delay_ms));
	bool new_empty = this->__empty();

	if (!m_disposing && old_empty && !new_empty) {
		this->__start();
	}
}

void camera::pop_waiter(camera::waiter_t waiter)
{
	bool old_empty = this->__empty();

	/// \note we must search it in two list!
	const std::type_info & exp_type = waiter.target_type();
	m_waiting_list.remove_if([&exp_type](const img_waiter_info & info) {
		if (info.callback.target_type() == exp_type) {
			return true;
		}
		return false;
	});
	m_disposing_list.remove_if([&exp_type](const img_waiter_info & info) {
		if (info.callback.target_type() == exp_type) {
			return true;
		}
		return false;
	});

	bool new_empty = this->__empty();
	if (!m_disposing && !old_empty && new_empty) {
		this->__stop();
	}
}

void camera::push_waiter_video(waiter_t waiter)
{
	bool old_empty = this->__empty();
	m_waiting_video_list.push_back(waiter);
	bool new_empty = this->__empty();

	if (!m_disposing && old_empty && !new_empty) {
		this->__start();
	}
}

void camera::pop_waiter_video(waiter_t waiter)
{
	bool old_empty = this->__empty();

	const std::type_info & exp_type = waiter.target_type();
	m_waiting_video_list.remove_if([&exp_type](const waiter_t & info) {
		if (info.target_type() == exp_type) {
			return true;
		}
		return false;
	});

	bool new_empty = this->__empty();
	if (!m_disposing && !old_empty && new_empty) {
		this->__stop();
	}
}

void camera::__start()
{
	__map_buf();
	__queue_buf();
	this->__streamon();

	exemodel::poller & p = *this;
	p.add(*m_cam);
}

void camera::__stop()
{
	exemodel::poller & p = *this;
	p.del(*m_cam);

	__unmap_buf();
	this->__streamoff();
}

void camera::__streamoff(void)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = m_cam->io_ctl(VIDIOC_STREAMOFF, &type);
	exemodel::validate_ret(ret, "VIDIOC_STREAMOFF");
}

void camera::__streamon(void)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = m_cam->io_ctl(VIDIOC_STREAMON, &type);
	exemodel::validate_ret(ret, "VIDIOC_STREAMON");
}

void camera::__req_buf(void)
{
	struct v4l2_requestbuffers req;
	req.count = m_buf.size();
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	int ret = m_cam->io_ctl(VIDIOC_REQBUFS, &req);
	exemodel::validate_ret(ret, "VIDIOC_REQBUFS");
	if (req.count != m_buf.size()) {
		throw std::system_error(ENOMEM, std::system_category(), "Insufficient buffer memory");
	}
}

void camera::__map_buf(void)
{
	for (uint32_t i = 0; i < m_buf.size(); ++i) {
		struct v4l2_buffer buf;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		int ret = m_cam->io_ctl(VIDIOC_QUERYBUF, &buf);
		exemodel::validate_ret(ret, "VIDIOC_QUERYBUF");
		if (buf.length != m_img_size) {
			throw std::length_error("buf length error");
		}
		m_buf[i] = ::mmap(NULL,
				buf.length,
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				m_cam->fd(),
				buf.m.offset);
		if (MAP_FAILED == m_buf[i]) {
			throw std::system_error(errno,
						std::system_category(),
						"mmap for video");
		}
	}
}

void camera::__unmap_buf(void)
{
	for (auto & buf : m_buf) {
		int ret = munmap(buf, m_img_size);
		if (ret < 0) {
			throw std::system_error(errno,
						std::system_category(),
						"munmap for video");
		}
		buf = nullptr;
	}
}

void camera::__queue_buf(void)
{
	for (uint32_t i = 0; i < m_buf.size(); ++i) {
		struct v4l2_buffer buf;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		int ret = m_cam->io_ctl(VIDIOC_QBUF, &buf);
		exemodel::validate_ret(ret, "VIDIOC_QBUF");
	}
}

void camera::__dispose(exemodel::poller &, uint32_t)
{
	int ret;

	if (this->__empty()) {
		log_err("camera new frame, but no waiter");
		//return;
	}

	/// dequeue
	struct v4l2_buffer vlb;
	vlb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vlb.memory = V4L2_MEMORY_MMAP;
	ret = m_cam->io_ctl(VIDIOC_DQBUF, &vlb);
	exemodel::validate_ret(ret, "VIDIOC_DQBUF");
	if (m_buf.size() <= vlb.index) {
		throw std::out_of_range("VIDIOC_DQBUF");
	}

	/// convert img
	this->__convert_img(m_buf[vlb.index], m_img_size);

	/// queue
	ret = m_cam->io_ctl(VIDIOC_QBUF, &vlb);
	exemodel::validate_ret(ret, "VIDIOC_QBUF");

	/// \note please don't return before reset to false.
	m_disposing = true;
	/// exe
	/// 1. if we don't use 'tmp_list', the callback may push/pop waiter, result problem.
	/// 2. then we use 'tmp_list', but, if we reserved some waiter, and the
	///    next waiter callback want pop it, then... so we must repush it at once.
	/// 3. when callbacking, want pop others, which list?
	/// 4. if we use empty/pop method, when end?
	m_disposing_list.swap(m_waiting_list);
	while (!m_disposing_list.empty()) {
		std::list<img_waiter_info> tmp;
		tmp.splice(tmp.begin(), m_disposing_list, m_disposing_list.begin());

		img_waiter_info & info = *tmp.begin();
		/// \note the time_stamp in vlb maybe zero
		if (exemodel::timespec_compare(exemodel::monotonic_clock_info::get_time(), info.time_stamp) < 0) {
			m_waiting_list.splice(m_waiting_list.begin(), tmp, tmp.begin());
		}
		else {
			info.callback(m_imgs);
		}
	};

	/// \note the video list has lowest priority
	for (auto & i : m_waiting_video_list) {
		i(m_imgs);
	}
	/// \note must reset to false
	m_disposing = false;

	/// check
	if (this->__empty()) {
		this->__stop();
	}
}

void camera::__convert_img(void * buf, uint32_t size)
{
	uint8_t * ori_buf = static_cast<uint8_t *>(buf);
	uint8_t * yz_buf = (uint8_t *)m_imgs.yz_img.data();
	uint8_t * xz_buf = (uint8_t *)m_imgs.xz_img.data();

	/// xz img
	std::copy(ori_buf, ori_buf + size/2, xz_buf);

	/// yz img
	std::copy(ori_buf + size/2, ori_buf + size, yz_buf);
}

bool camera::__empty() const
{
	return (m_waiting_list.empty() && m_waiting_video_list.empty());
}

void camera::dump()
{
	log_info("waiting list size : %d", m_waiting_list.size());
	for (auto & i : m_waiting_list) {
		log_info("\twaiter type: %s", i.callback.target_type().name());
	}
}

