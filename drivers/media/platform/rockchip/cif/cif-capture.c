// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip CIF Camera Interface Driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 * Copyright (C) 2020 Maxime Chevallier <maxime.chevallier@bootlin.com>
 * Copyright (C) 2023 Mehdi Djait <mehdi.djait@bootlin.com>
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <media/v4l2-common.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "cif-capture.h"
#include "cif-common.h"
#include "cif-regs.h"

#define CIF_REQ_BUFS_MIN	2
#define CIF_MIN_WIDTH		64
#define CIF_MIN_HEIGHT		64
#define CIF_MAX_WIDTH		8192
#define CIF_MAX_HEIGHT		8192

#define CIF_PLANE_Y		0
#define CIF_PLANE_UV		1

static struct cif_output_fmt out_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.fmt_val = CIF_FORMAT_YUV_OUTPUT_422 |
			   CIF_FORMAT_UV_STORAGE_ORDER_UVUV,
		.cplanes = 2,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.fmt_val = CIF_FORMAT_YUV_OUTPUT_422 |
			   CIF_FORMAT_UV_STORAGE_ORDER_VUVU,
		.cplanes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.fmt_val = CIF_FORMAT_YUV_OUTPUT_420 |
			   CIF_FORMAT_UV_STORAGE_ORDER_UVUV,
		.cplanes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.fmt_val = CIF_FORMAT_YUV_OUTPUT_420 |
			   CIF_FORMAT_UV_STORAGE_ORDER_VUVU,
		.cplanes = 2,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB24,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR666,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB10,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG10,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG10,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR10,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB12,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG12,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG12,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR12,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR16,
		.cplanes = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_Y16,
		.cplanes = 1,
	}
};

static const struct cif_input_fmt *get_input_fmt(struct cif_device *cif_dev,
						 struct v4l2_subdev *sd)
{
	const struct cif_input_fmt *in_fmts = cif_dev->match_data->in_fmts;
	struct v4l2_subdev_format fmt;
	u32 i;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = 0;
	v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);

	for (i = 0; i < cif_dev->match_data->in_fmts_num; i++)
		if (fmt.format.code == in_fmts[i].mbus_code &&
		    fmt.format.field == in_fmts[i].field)
			return &in_fmts[i];

	v4l2_err(sd->v4l2_dev, "remote's mbus code not supported\n");
	return NULL;
}

static struct cif_output_fmt *find_output_fmt(struct cif_stream *stream,
					      u32 pixelfmt)
{
	struct cif_output_fmt *fmt;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(out_fmts); i++) {
		fmt = &out_fmts[i];
		if (fmt->fourcc == pixelfmt)
			return fmt;
	}

	return NULL;
}

static struct cif_buffer *cif_get_buffer(struct cif_stream *stream)
{
	struct cif_buffer *buff;

	lockdep_assert_held(&stream->vbq_lock);

	if (list_empty(&stream->buf_head))
		return NULL;

	buff = list_first_entry(&stream->buf_head, struct cif_buffer, queue);
	list_del(&buff->queue);

	return buff;
}

static int cif_init_buffers(struct cif_stream *stream)
{
	struct cif_device *cif_dev = stream->cifdev;
	unsigned long lock_flags;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);

	stream->buffs[0] = cif_get_buffer(stream);
	stream->buffs[1] = cif_get_buffer(stream);

	if (!(stream->buffs[0]) || !(stream->buffs[1])) {
		spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
		return -EINVAL;
	}

	stream->drop_frame = false;

	cif_write(cif_dev, CIF_FRM0_ADDR_Y,
		  stream->buffs[0]->buff_addr[CIF_PLANE_Y]);
	cif_write(cif_dev, CIF_FRM0_ADDR_UV,
		  stream->buffs[0]->buff_addr[CIF_PLANE_UV]);

	cif_write(cif_dev, CIF_FRM1_ADDR_Y,
		  stream->buffs[1]->buff_addr[CIF_PLANE_Y]);
	cif_write(cif_dev, CIF_FRM1_ADDR_UV,
		  stream->buffs[1]->buff_addr[CIF_PLANE_UV]);

	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);

	return 0;
}

static void cif_assign_new_buffer_pingpong(struct cif_stream *stream)
{
	struct cif_device *cif_dev = stream->cifdev;
	struct cif_buffer *buffer = NULL;
	u32 frm_addr_y, frm_addr_uv;
	unsigned long lock_flags;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);

	buffer = cif_get_buffer(stream);

	/*
	 * In Pingpong mode:
	 * After one frame0 captured, CIF will start to capture the next frame1
	 * automatically.
	 *
	 * If there is no buffer:
	 * 1. Make the next frame0 write to the buffer of frame1.
	 *
	 * 2. Drop the frame1: Don't return it to user-space, as it will be
	 *    overwritten by the next frame0.
	 */
	if (!buffer) {
		stream->drop_frame = true;
		buffer = stream->buffs[1 - stream->frame_phase];
	} else {
		stream->drop_frame = false;
	}

	stream->buffs[stream->frame_phase] = buffer;
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);

	frm_addr_y = stream->frame_phase ? CIF_FRM1_ADDR_Y : CIF_FRM0_ADDR_Y;
	frm_addr_uv = stream->frame_phase ? CIF_FRM1_ADDR_UV : CIF_FRM0_ADDR_UV;

	cif_write(cif_dev, frm_addr_y, buffer->buff_addr[CIF_PLANE_Y]);
	cif_write(cif_dev, frm_addr_uv, buffer->buff_addr[CIF_PLANE_UV]);
}

static void cif_stream_stop(struct cif_stream *stream)
{
	struct cif_device *cif_dev = stream->cifdev;
	u32 val;

	val = cif_read(cif_dev, CIF_CTRL);
	cif_write(cif_dev, CIF_CTRL, val & (~CIF_CTRL_ENABLE_CAPTURE));
	cif_write(cif_dev, CIF_INTEN, 0x0);
	cif_write(cif_dev, CIF_INTSTAT, 0x3ff);
	cif_write(cif_dev, CIF_FRAME_STATUS, 0x0);

	stream->stopping = false;
}

static int cif_queue_setup(struct vb2_queue *queue,
			   unsigned int *num_buffers,
			   unsigned int *num_planes,
			   unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct cif_stream *stream = queue->drv_priv;

	if (*num_planes)
		return sizes[0] < stream->pix.sizeimage ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = stream->pix.sizeimage;

	return 0;
}

static void cif_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cif_buffer *cifbuf = to_cif_buffer(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct cif_stream *stream = queue->drv_priv;
	struct v4l2_pix_format *pix = &stream->pix;
	unsigned long lock_flags;
	int i;

	struct cif_output_fmt *fmt = stream->cif_fmt_out;

	memset(cifbuf->buff_addr, 0, sizeof(cifbuf->buff_addr));

	cifbuf->buff_addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);

	for (i = 0; i < fmt->cplanes - 1; i++)
		cifbuf->buff_addr[i + 1] = cifbuf->buff_addr[i] +
			pix->bytesperline * pix->height;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	list_add_tail(&cifbuf->queue, &stream->buf_head);
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
}

static void cif_return_all_buffers(struct cif_stream *stream,
				   enum vb2_buffer_state state)
{
	struct cif_buffer *buf;
	unsigned long lock_flags;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);

	if (stream->buffs[0]) {
		vb2_buffer_done(&stream->buffs[0]->vb.vb2_buf, state);
		stream->buffs[0] = NULL;
	}

	if (stream->buffs[1]) {
		if (!stream->drop_frame)
			vb2_buffer_done(&stream->buffs[1]->vb.vb2_buf, state);

		stream->buffs[1] = NULL;
	}

	while (!list_empty(&stream->buf_head)) {
		buf = cif_get_buffer(stream);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}

	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
}

static void cif_stop_streaming(struct vb2_queue *queue)
{
	struct cif_stream *stream = queue->drv_priv;
	struct cif_device *cif_dev = stream->cifdev;
	struct v4l2_subdev *sd = cif_dev->remote.sd;
	int ret;

	v4l2_subdev_call(sd, video, s_stream, 0);

	stream->stopping = true;
	ret = wait_event_timeout(stream->wq_stopped,
				 !stream->stopping,
				 msecs_to_jiffies(1000));
	if (!ret)
		cif_stream_stop(stream);

	pm_runtime_put(cif_dev->dev);

	cif_return_all_buffers(stream, VB2_BUF_STATE_ERROR);
}

static int cif_stream_start(struct cif_stream *stream)
{
	u32 val, fmt_type, xfer_mode = 0;
	struct cif_device *cif_dev = stream->cifdev;
	struct cif_remote *remote_info = &cif_dev->remote;
	struct v4l2_mbus_config_parallel *parallel;
	int ret;
	u32 input_mode;

	stream->frame_idx = 0;
	stream->frame_phase = 0;

	fmt_type = stream->cif_fmt_in->fmt_type;
	input_mode = (remote_info->std == V4L2_STD_NTSC) ?
		      CIF_FORMAT_INPUT_MODE_NTSC :
		      CIF_FORMAT_INPUT_MODE_PAL;

	parallel = &cif_dev->vep.bus.parallel;
	if ((parallel->bus_width == 16) &&
	    (parallel->flags & V4L2_MBUS_PCLK_SAMPLE_DUALEDGE))
		xfer_mode |= CIF_FORMAT_BT1120_CLOCK_DOUBLE_EDGES;

	val = input_mode | stream->cif_fmt_out->fmt_val |
	      stream->cif_fmt_in->dvp_fmt_val | xfer_mode;
	cif_write(cif_dev, CIF_FOR, val);

	val = stream->pix.width;
	if (stream->cif_fmt_in->fmt_type == CIF_FMT_TYPE_RAW)
		val = stream->pix.width * 2;

	cif_write(cif_dev, CIF_VIR_LINE_WIDTH, val);
	cif_write(cif_dev, CIF_SET_SIZE,
		  stream->pix.width | (stream->pix.height << 16));

	cif_write(cif_dev, CIF_FRAME_STATUS, CIF_FRAME_STAT_CLS);
	cif_write(cif_dev, CIF_INTSTAT, CIF_INTSTAT_CLS);
	if (cif_dev->match_data->has_scaler)
		cif_write(cif_dev, CIF_SCL_CTRL,
			  (fmt_type == CIF_FMT_TYPE_YUV) ?
				  CIF_SCL_CTRL_ENABLE_YUV_16BIT_BYPASS :
				  CIF_SCL_CTRL_ENABLE_RAW_16BIT_BYPASS);

	ret = cif_init_buffers(stream);
	if (ret)
		return ret;

	cif_write(cif_dev, CIF_INTEN, CIF_INTEN_FRAME_END_EN |
				      CIF_INTEN_LINE_ERR_EN |
				      CIF_INTEN_PST_INF_FRAME_END_EN);

	cif_write(cif_dev, CIF_CTRL, CIF_CTRL_AXI_BURST_16 |
				     CIF_CTRL_MODE_PINGPONG |
				     CIF_CTRL_ENABLE_CAPTURE);

	return 0;
}

static int cif_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct cif_stream *stream = queue->drv_priv;
	struct cif_device *cif_dev = stream->cifdev;
	struct v4l2_device *v4l2_dev = &cif_dev->v4l2_dev;
	struct v4l2_subdev *sd;
	int ret;

	if (!cif_dev->remote.sd) {
		ret = -ENODEV;
		v4l2_err(v4l2_dev, "No remote subdev detected\n");
		goto destroy_buf;
	}

	ret = pm_runtime_resume_and_get(cif_dev->dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to get runtime pm, %d\n", ret);
		goto destroy_buf;
	}

	sd = cif_dev->remote.sd;

	stream->cif_fmt_in = get_input_fmt(cif_dev, sd);
	if (!stream->cif_fmt_in)
		goto runtime_put;

	ret = cif_stream_start(stream);
	if (ret < 0)
		goto stop_stream;

	ret = v4l2_subdev_call(sd, video, s_stream, 1);
	if (ret < 0)
		goto stop_stream;

	return 0;

stop_stream:
	cif_stream_stop(stream);
runtime_put:
	pm_runtime_put(cif_dev->dev);
destroy_buf:
	cif_return_all_buffers(stream, VB2_BUF_STATE_QUEUED);

	return ret;
}

static const struct vb2_ops cif_vb2_ops = {
	.queue_setup = cif_queue_setup,
	.buf_queue = cif_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.stop_streaming = cif_stop_streaming,
	.start_streaming = cif_start_streaming,
};

static int cif_init_vb2_queue(struct vb2_queue *q,
			      struct cif_stream *stream)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = stream;
	q->ops = &cif_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct cif_buffer);
	q->min_buffers_needed = CIF_REQ_BUFS_MIN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &stream->vlock;
	q->dev = stream->cifdev->dev;

	return vb2_queue_init(q);
}

static void cif_update_pix(struct cif_stream *stream,
			   struct cif_output_fmt *fmt,
			   struct v4l2_pix_format *pix)
{
	struct cif_remote *remote_info = &stream->cifdev->remote;
	struct v4l2_subdev_format sd_fmt;
	u32 width, height;

	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;
	v4l2_subdev_call(remote_info->sd, pad, get_fmt, NULL, &sd_fmt);

	width = clamp_t(u32, sd_fmt.format.width,
			CIF_MIN_WIDTH, CIF_MAX_WIDTH);
	height = clamp_t(u32, sd_fmt.format.height,
			 CIF_MIN_HEIGHT, CIF_MAX_HEIGHT);

	pix->width = width;
	pix->height = height;
	pix->field = sd_fmt.format.field;
	pix->colorspace = sd_fmt.format.colorspace;
	pix->ycbcr_enc = sd_fmt.format.ycbcr_enc;
	pix->quantization = sd_fmt.format.quantization;
	pix->xfer_func = sd_fmt.format.xfer_func;

	v4l2_fill_pixfmt(pix, fmt->fourcc, pix->width, pix->height);
}

static int cif_set_fmt(struct cif_stream *stream,
		       struct v4l2_pix_format *pix)
{
	struct cif_device *cif_dev = stream->cifdev;
	struct v4l2_subdev_format sd_fmt;
	struct cif_output_fmt *fmt;
	int ret;

	if (vb2_is_streaming(&stream->buf_queue))
		return -EBUSY;

	fmt = find_output_fmt(stream, pix->pixelformat);
	if (!fmt)
		fmt = &out_fmts[0];

	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;
	sd_fmt.format.width = pix->width;
	sd_fmt.format.height = pix->height;

	ret = v4l2_subdev_call(cif_dev->remote.sd, pad, set_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	cif_update_pix(stream, fmt, pix);
	stream->pix = *pix;
	stream->cif_fmt_out = fmt;

	return 0;
}

void cif_set_default_format(struct cif_device *cif_dev)
{
	struct cif_stream *stream = &cif_dev->stream;
	struct v4l2_pix_format pix;

	cif_dev->remote.std = V4L2_STD_NTSC;

	pix.pixelformat = V4L2_PIX_FMT_NV12;
	pix.width = CIF_DEFAULT_WIDTH;
	pix.height = CIF_DEFAULT_HEIGHT;

	cif_set_fmt(stream, &pix);
}

void cif_stream_init(struct cif_device *cif_dev)
{
	struct cif_stream *stream = &cif_dev->stream;
	struct v4l2_pix_format pix;

	memset(stream, 0, sizeof(*stream));
	memset(&pix, 0, sizeof(pix));
	stream->cifdev = cif_dev;

	INIT_LIST_HEAD(&stream->buf_head);
	spin_lock_init(&stream->vbq_lock);
	init_waitqueue_head(&stream->wq_stopped);
}

static const struct v4l2_file_operations cif_fops = {
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int cif_enum_input(struct file *file, void *priv,
			  struct v4l2_input *input)
{
	struct cif_stream *stream = video_drvdata(file);
	struct v4l2_subdev *sd = stream->cifdev->remote.sd;
	int ret;

	if (input->index > 0)
		return -EINVAL;

	ret = v4l2_subdev_call(sd, video, g_input_status, &input->status);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	strscpy(input->name, "Camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = stream->vdev.tvnorms;
	input->capabilities = V4L2_IN_CAP_STD;

	return 0;
}

static int cif_g_std(struct file *file, void *fh, v4l2_std_id *norm)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_remote *remote_info = &stream->cifdev->remote;

	*norm = remote_info->std;

	return 0;
}

static int cif_s_std(struct file *file, void *fh, v4l2_std_id norm)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_remote *remote_info = &stream->cifdev->remote;
	int ret;

	if (norm == remote_info->std)
		return 0;

	if (vb2_is_busy(&stream->buf_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(remote_info->sd, video, s_std, norm);
	if (ret)
		return ret;

	remote_info->std = norm;

	/* S_STD will update the format since that depends on the standard. */
	cif_update_pix(stream, stream->cif_fmt_out, &stream->pix);

	return 0;
}

static int cif_querystd(struct file *file, void *fh, v4l2_std_id *a)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_remote *remote_info = &stream->cifdev->remote;

	*a = V4L2_STD_UNKNOWN;

	return v4l2_subdev_call(remote_info->sd, video, querystd, a);
}

static int cif_enum_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	struct cif_output_fmt *fmt = NULL;

	if (f->index >= ARRAY_SIZE(out_fmts))
		return -EINVAL;

	fmt = &out_fmts[f->index];
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int cif_try_fmt_vid_cap(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_output_fmt *fmt;

	fmt = find_output_fmt(stream, f->fmt.pix.pixelformat);
	if (!fmt)
		fmt = &out_fmts[0];

	cif_update_pix(stream, fmt, &f->fmt.pix);

	return 0;
}

static int cif_s_fmt_vid_cap(struct file *file,
			     void *priv, struct v4l2_format *f)
{
	struct cif_stream *stream = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&stream->buf_queue))
		return -EBUSY;

	ret = cif_set_fmt(stream, &f->fmt.pix);

	return ret;
}

static int cif_g_fmt_vid_cap(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct cif_stream *stream = video_drvdata(file);

	f->fmt.pix = stream->pix;

	return 0;
}

static int cif_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct cif_stream *stream = video_drvdata(file);
	struct device *dev = stream->cifdev->dev;

	strscpy(cap->driver, dev->driver->name, sizeof(cap->driver));
	strscpy(cap->card, dev->driver->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", dev_name(dev));

	return 0;
}

static int cif_enum_framesizes(struct file *file, void *fh,
			       struct v4l2_frmsizeenum *fsize)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_device *cif_dev = stream->cifdev;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct cif_output_fmt *fmt;
	int ret;

	if (!cif_dev->remote.sd)
		return -ENODEV;

	fmt = find_output_fmt(stream, fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	fse.code = fmt->mbus;

	ret = v4l2_subdev_call(cif_dev->remote.sd, pad, enum_frame_size,
			       NULL, &fse);
	if (ret)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

static int cif_enum_frameintervals(struct file *file, void *fh,
				   struct v4l2_frmivalenum *fival)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_device *cif_dev = stream->cifdev;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = fival->index,
		.width = fival->width,
		.height = fival->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct cif_output_fmt *fmt;
	int ret;

	if (!cif_dev->remote.sd)
		return -ENODEV;

	fmt = find_output_fmt(stream, fival->pixel_format);
	if (!fmt)
		return -EINVAL;

	fie.code = fmt->mbus;

	ret = v4l2_subdev_call(cif_dev->remote.sd, pad, enum_frame_interval,
			       NULL, &fie);
	if (ret)
		return ret;

	fival->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return 0;
}

static int cif_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int cif_s_input(struct file *file, void *fh, unsigned int i)
{
	if (i)
		return -EINVAL;

	return 0;
}

static int cif_g_parm(struct file *file, void *priv, struct v4l2_streamparm *p)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_device *cif_dev = stream->cifdev;

	if (!cif_dev->remote.sd)
		return -ENODEV;

	return v4l2_g_parm_cap(video_devdata(file), cif_dev->remote.sd, p);
}

static int cif_s_parm(struct file *file, void *priv, struct v4l2_streamparm *p)
{
	struct cif_stream *stream = video_drvdata(file);
	struct cif_device *cif_dev = stream->cifdev;

	if (!cif_dev->remote.sd)
		return -ENODEV;

	return v4l2_s_parm_cap(video_devdata(file), cif_dev->remote.sd, p);
}

static const struct v4l2_ioctl_ops cif_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_g_std = cif_g_std,
	.vidioc_s_std = cif_s_std,
	.vidioc_querystd = cif_querystd,

	.vidioc_enum_fmt_vid_cap = cif_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = cif_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = cif_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = cif_g_fmt_vid_cap,
	.vidioc_querycap = cif_querycap,
	.vidioc_enum_framesizes = cif_enum_framesizes,
	.vidioc_enum_frameintervals = cif_enum_frameintervals,

	.vidioc_enum_input = cif_enum_input,
	.vidioc_g_input = cif_g_input,
	.vidioc_s_input = cif_s_input,

	.vidioc_g_parm = cif_g_parm,
	.vidioc_s_parm = cif_s_parm,
};

void cif_unregister_stream_vdev(struct cif_device *cif_dev)
{
	struct cif_stream *stream = &cif_dev->stream;

	media_entity_cleanup(&stream->vdev.entity);
	video_unregister_device(&stream->vdev);
}

int cif_register_stream_vdev(struct cif_device *cif_dev)
{
	struct cif_stream *stream = &cif_dev->stream;
	struct v4l2_device *v4l2_dev = &cif_dev->v4l2_dev;
	struct video_device *vdev = &stream->vdev;
	int ret;

	strscpy(vdev->name, CIF_DRIVER_NAME, sizeof(vdev->name));
	mutex_init(&stream->vlock);

	vdev->ioctl_ops = &cif_v4l2_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->fops = &cif_fops;
	vdev->minor = -1;
	vdev->v4l2_dev = v4l2_dev;
	vdev->lock = &stream->vlock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_STREAMING;
	vdev->tvnorms = V4L2_STD_NTSC | V4L2_STD_PAL;
	video_set_drvdata(vdev, stream);
	vdev->vfl_dir = VFL_DIR_RX;
	stream->pad.flags = MEDIA_PAD_FL_SINK;

	cif_init_vb2_queue(&stream->buf_queue, stream);

	vdev->queue = &stream->buf_queue;
	strscpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));

	ret = media_entity_pads_init(&vdev->entity, 1, &stream->pad);
	if (ret < 0)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0)
		v4l2_err(v4l2_dev,
			 "video_register_device failed with error %d\n", ret);

	return ret;
}

static void cif_vb_done(struct cif_stream *stream,
			struct vb2_v4l2_buffer *vb_done)
{
	vb2_set_plane_payload(&vb_done->vb2_buf, 0,
			      stream->pix.sizeimage);
	vb_done->vb2_buf.timestamp = ktime_get_ns();
	vb_done->sequence = stream->frame_idx;
	vb2_buffer_done(&vb_done->vb2_buf, VB2_BUF_STATE_DONE);
}

static void cif_reset_stream(struct cif_device *cif_dev)
{
	u32 ctl = cif_read(cif_dev, CIF_CTRL);

	cif_write(cif_dev, CIF_CTRL, ctl & (~CIF_CTRL_ENABLE_CAPTURE));
	cif_write(cif_dev, CIF_CTRL, ctl | CIF_CTRL_ENABLE_CAPTURE);
}

irqreturn_t cif_irq_pingpong(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct cif_device *cif_dev = dev_get_drvdata(dev);
	struct cif_stream *stream = &cif_dev->stream;
	unsigned int intstat;
	u32 lastline, lastpix, ctl, cif_frmst;

	intstat = cif_read(cif_dev, CIF_INTSTAT);
	cif_frmst = cif_read(cif_dev, CIF_FRAME_STATUS);
	lastline = CIF_FETCH_Y_LAST_LINE(cif_read(cif_dev, CIF_LAST_LINE));
	lastpix =  CIF_FETCH_Y_LAST_LINE(cif_read(cif_dev, CIF_LAST_PIX));
	ctl = cif_read(cif_dev, CIF_CTRL);

	/*
	 * There are two irqs enabled:
	 *  - PST_INF_FRAME_END: cif FIFO is ready,
	 *    this is prior to FRAME_END
	 *  - FRAME_END: cif has saved frame to memory,
	 *    a frame ready
	 */

	if (intstat & CIF_INTSTAT_PST_INF_FRAME_END) {
		cif_write(cif_dev, CIF_INTSTAT,
			  CIF_INTSTAT_PST_INF_FRAME_END_CLR);

		if (stream->stopping)
			/* To stop CIF ASAP, before FRAME_END irq. */
			cif_write(cif_dev, CIF_CTRL,
				  ctl & (~CIF_CTRL_ENABLE_CAPTURE));
	}

	if (intstat & CIF_INTSTAT_PRE_INF_FRAME_END)
		cif_write(cif_dev, CIF_INTSTAT, CIF_INTSTAT_PRE_INF_FRAME_END);

	if (intstat & (CIF_INTSTAT_LINE_ERR | CIF_INTSTAT_PIX_ERR)) {
		v4l2_err(&cif_dev->v4l2_dev,
			 "LINE_ERR OR PIX_ERR detected, stream will be reset");
		cif_write(cif_dev, CIF_INTSTAT, CIF_INTSTAT_LINE_ERR |
						CIF_INTSTAT_PIX_ERR);
		cif_reset_stream(cif_dev);
	}

	if (intstat & CIF_INTSTAT_FRAME_END) {
		struct vb2_v4l2_buffer *vb_done = NULL;

		cif_write(cif_dev, CIF_INTSTAT, CIF_INTSTAT_FRAME_END_CLR |
						CIF_INTSTAT_LINE_END_CLR);

		if (stream->stopping) {
			cif_stream_stop(stream);
			wake_up(&stream->wq_stopped);
			return IRQ_HANDLED;
		}

		if (lastline != stream->pix.height) {
			v4l2_err(&cif_dev->v4l2_dev,
				 "Bad frame, irq:%#x frmst:%#x size:%dx%d\n",
				 intstat, cif_frmst, lastpix, lastline);

			cif_reset_stream(cif_dev);
		}

		if (cif_frmst & CIF_INTSTAT_F0_READY)
			stream->frame_phase = 0;
		else if (cif_frmst & CIF_INTSTAT_F1_READY)
			stream->frame_phase = 1;
		else
			return IRQ_HANDLED;

		vb_done = &stream->buffs[stream->frame_phase]->vb;
		if (!stream->drop_frame) {
			cif_vb_done(stream, vb_done);
			stream->frame_idx++;
		}

		cif_assign_new_buffer_pingpong(stream);
	}

	return IRQ_HANDLED;
}
