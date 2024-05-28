/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip CIF Driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 * Copyright (C) 2023 Mehdi Djait <mehdi.djait@bootlin.com>
 */

#ifndef _CIF_COMMON_H
#define _CIF_COMMON_H

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

#include "cif-regs.h"

#define CIF_DRIVER_NAME		"rockchip-cif"

#define CIF_MAX_BUS_CLK		8
#define CIF_MAX_SENSOR		1
#define CIF_MAX_RESET		5

#define CIF_DEFAULT_WIDTH	640
#define CIF_DEFAULT_HEIGHT	480

struct cif_buffer {
	struct vb2_v4l2_buffer	vb;
	struct list_head	queue;
	u32			buff_addr[VIDEO_MAX_PLANES];
};

static inline struct cif_buffer *to_cif_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct cif_buffer, vb);
}

struct cif_remote {
	struct v4l2_subdev	*sd;
	int			pad;
	int			lanes;
	v4l2_std_id		std;
};

struct cif_output_fmt {
	u32	fourcc;
	u32	mbus;
	u32	fmt_val;
	u8	cplanes;
};

enum cif_fmt_type {
	CIF_FMT_TYPE_YUV = 0,
	CIF_FMT_TYPE_RAW,
};

struct cif_input_fmt {
	u32			mbus_code;
	u32			dvp_fmt_val;
	enum cif_fmt_type	fmt_type;
	enum v4l2_field		field;
};

struct cif_stream {
	struct cif_device		*cifdev;
	bool				stopping;
	wait_queue_head_t		wq_stopped;
	int				frame_idx;
	int				frame_phase;
	bool				drop_frame;

	/* Lock between irq and buf_queue, buffs. */
	spinlock_t			vbq_lock;
	struct vb2_queue		buf_queue;
	struct list_head		buf_head;
	struct cif_buffer		*buffs[2];

	/* Lock used by the V4L core. */
	struct mutex			vlock;
	struct video_device		vdev;
	struct media_pad		pad;

	struct cif_output_fmt		*cif_fmt_out;
	const struct cif_input_fmt	*cif_fmt_in;
	struct v4l2_pix_format		pix;
};

static inline struct cif_stream *to_cif_stream(struct video_device *vdev)
{
	return container_of(vdev, struct cif_stream, vdev);
}

struct cif_match_data {
	struct clk_bulk_data *clks;
	int clks_num;
	const struct cif_input_fmt *in_fmts;
	int in_fmts_num;
	void (*grf_dvp_setup)(struct cif_device *cif_dev);
	bool has_scaler;
	unsigned int regs[CIF_REGISTERS_MAX];
};

struct cif_device {
	struct device			*dev;
	int				irq;
	void __iomem			*base_addr;
	struct reset_control		*cif_rst;
	struct regmap			*grf;

	struct v4l2_device		v4l2_dev;
	struct media_device		media_dev;
	struct v4l2_async_notifier	notifier;
	struct v4l2_async_connection	asd;
	struct v4l2_fwnode_endpoint	vep;
	struct cif_remote		remote;

	struct cif_stream		stream;
	const struct cif_match_data	*match_data;
};

static inline unsigned int cif_get_addr(struct cif_device *cif_device,
					unsigned int index)
{
	if (index >= CIF_REGISTERS_MAX)
		return CIF_REGISTERS_INVALID;

	return cif_device->match_data->regs[index];
}

static inline void cif_write(struct cif_device *cif_dev, unsigned int index,
			     u32 val)
{
	unsigned int addr = cif_get_addr(cif_dev, index);

	if (addr == CIF_REGISTERS_INVALID)
		return;

	writel(val, cif_dev->base_addr + addr);
}

static inline u32 cif_read(struct cif_device *cif_dev, unsigned int index)
{
	unsigned int addr = cif_get_addr(cif_dev, index);

	if (addr == CIF_REGISTERS_INVALID)
		return 0;

	return readl(cif_dev->base_addr + addr);
}

#endif
