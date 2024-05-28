// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip CIF Camera Interface Driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 * Copyright (C) 2020 Maxime Chevallier <maxime.chevallier@bootlin.com>
 * Copyright (C) 2023 Mehdi Djait <mehdi.djait@bootlin.com>
 */

#include <dt-bindings/media/rockchip-cif.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-fwnode.h>

#include "cif-capture.h"
#include "cif-common.h"
#include "cif-regs.h"

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct cif_device *cif_dev;
	struct v4l2_subdev *sd;
	int ret;

	cif_dev = container_of(notifier, struct cif_device, notifier);
	sd = cif_dev->remote.sd;

	mutex_lock(&cif_dev->media_dev.graph_mutex);

	ret = v4l2_device_register_subdev_nodes(&cif_dev->v4l2_dev);
	if (ret < 0)
		goto unlock;

	ret = media_create_pad_link(&sd->entity, 0,
				    &cif_dev->stream.vdev.entity, 0,
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		dev_err(cif_dev->dev, "failed to create link");

unlock:
	mutex_unlock(&cif_dev->media_dev.graph_mutex);
	return ret;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_connection *asd)
{
	struct cif_device *cif_dev = container_of(notifier,
						  struct cif_device, notifier);
	int pad;

	cif_dev->remote.sd = subdev;
	pad = media_entity_get_fwnode_pad(&subdev->entity, subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;

	cif_dev->remote.pad = pad;

	return 0;
}

static const struct v4l2_async_notifier_operations subdev_notifier_ops = {
	.bound = subdev_notifier_bound,
	.complete = subdev_notifier_complete,
};

static int cif_subdev_notifier(struct cif_device *cif_dev)
{
	struct v4l2_async_notifier *ntf = &cif_dev->notifier;
	struct device *dev = cif_dev->dev;
	struct v4l2_async_connection *asd;
	struct v4l2_fwnode_endpoint *vep = &cif_dev->vep;
	struct fwnode_handle *ep;
	int ret;

	v4l2_async_nf_init(ntf, &cif_dev->v4l2_dev);

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
					     MEDIA_ROCKCHIP_CIF_DVP, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return -ENODEV;

	vep->bus_type = V4L2_MBUS_UNKNOWN;
	ret = v4l2_fwnode_endpoint_parse(ep, vep);
	if (ret)
		goto complete;

	if (vep->bus_type != V4L2_MBUS_BT656 &&
	    vep->bus_type != V4L2_MBUS_PARALLEL) {
		v4l2_err(&cif_dev->v4l2_dev, "unsupported bus type\n");
		goto complete;
	}

	if (cif_dev->match_data->grf_dvp_setup)
		cif_dev->match_data->grf_dvp_setup(cif_dev);

	asd = v4l2_async_nf_add_fwnode_remote(ntf, ep,
					      struct v4l2_async_connection);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto complete;
	}

	ntf->ops = &subdev_notifier_ops;

	ret = v4l2_async_nf_register(ntf);
	if (ret)
		v4l2_async_nf_cleanup(ntf);

complete:
	fwnode_handle_put(ep);

	return ret;
}

static struct clk_bulk_data px30_cif_clks[] = {
	{ .id = "aclk", },
	{ .id = "hclk", },
	{ .id = "pclk", },
};

static const struct cif_input_fmt px30_in_fmts[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_UYVY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_UYVY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_VYUY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_VYUY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}
};

static const struct cif_match_data px30_cif_match_data = {
	.clks = px30_cif_clks,
	.clks_num = ARRAY_SIZE(px30_cif_clks),
	.in_fmts = px30_in_fmts,
	.in_fmts_num = ARRAY_SIZE(px30_in_fmts),
	.has_scaler = true,
	.regs = {
		[CIF_CTRL] = 0x00,
		[CIF_INTEN] = 0x04,
		[CIF_INTSTAT] = 0x08,
		[CIF_FOR] = 0x0c,
		[CIF_LINE_NUM_ADDR] = 0x10,
		[CIF_FRM0_ADDR_Y] = 0x14,
		[CIF_FRM0_ADDR_UV] = 0x18,
		[CIF_FRM1_ADDR_Y] = 0x1c,
		[CIF_FRM1_ADDR_UV] = 0x20,
		[CIF_VIR_LINE_WIDTH] = 0x24,
		[CIF_SET_SIZE] = 0x28,
		[CIF_SCL_CTRL] = 0x48,
		[CIF_FRAME_STATUS] = 0x60,
		[CIF_LAST_LINE] = 0x68,
		[CIF_LAST_PIX] = 0x6c,
	},
};

static const struct cif_input_fmt rk3568_in_fmts[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_UYVY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_UYVY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_VYUY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_VYUY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV |
				  CIF_FORMAT_INPUT_MODE_BT1120 |
				  CIF_FORMAT_BT1120_TRANSMIT_PROGRESS,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV |
				  CIF_FORMAT_INPUT_MODE_BT1120,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU |
				  CIF_FORMAT_INPUT_MODE_BT1120 |
				  CIF_FORMAT_BT1120_TRANSMIT_PROGRESS,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU |
				  CIF_FORMAT_INPUT_MODE_BT1120,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV |
				  CIF_FORMAT_INPUT_MODE_BT1120 |
				  CIF_FORMAT_BT1120_YC_SWAP |
				  CIF_FORMAT_BT1120_TRANSMIT_PROGRESS,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YUYV |
				  CIF_FORMAT_BT1120_YC_SWAP |
				  CIF_FORMAT_INPUT_MODE_BT1120,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU |
				  CIF_FORMAT_INPUT_MODE_BT1120 |
				  CIF_FORMAT_BT1120_YC_SWAP |
				  CIF_FORMAT_BT1120_TRANSMIT_PROGRESS,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_1X16,
		.dvp_fmt_val	= CIF_FORMAT_YUV_INPUT_422 |
				  CIF_FORMAT_YUV_INPUT_ORDER_YVYU |
				  CIF_FORMAT_BT1120_YC_SWAP |
				  CIF_FORMAT_INPUT_MODE_BT1120,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
		.dvp_fmt_val	= CIF_FORMAT_INPUT_MODE_RAW |
				  CIF_FORMAT_RAW_DATA_WIDTH_12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	},
};

static struct clk_bulk_data rk3568_cif_clks[] = {
	{ .id = "aclk", },
	{ .id = "hclk", },
	{ .id = "dclk", },
	{ .id = "iclk", },
};

static void rk3568_grf_dvp_setup(struct cif_device *cif_dev)
{
	u32 con1 = RK3568_GRF_WRITE_ENABLE(RK3568_GRF_VI_CON1_CIF_DATAPATH);

	if (cif_dev->vep.bus.parallel.flags & V4L2_MBUS_PCLK_SAMPLE_DUALEDGE)
		con1 |= RK3568_GRF_VI_CON1_CIF_DATAPATH;

	regmap_write(cif_dev->grf, RK3568_GRF_VI_CON1, con1);
}

static const struct cif_match_data rk3568_cif_match_data = {
	.clks = rk3568_cif_clks,
	.clks_num = ARRAY_SIZE(rk3568_cif_clks),
	.grf_dvp_setup = rk3568_grf_dvp_setup,
	.in_fmts = rk3568_in_fmts,
	.in_fmts_num = ARRAY_SIZE(rk3568_in_fmts),
	.has_scaler = false,
	.regs = {
		[CIF_CTRL] = 0x00,
		[CIF_INTEN] = 0x04,
		[CIF_INTSTAT] = 0x08,
		[CIF_FOR] = 0x0c,
		[CIF_LINE_NUM_ADDR] = 0x2c,
		[CIF_FRM0_ADDR_Y] = 0x14,
		[CIF_FRM0_ADDR_UV] = 0x18,
		[CIF_FRM1_ADDR_Y] = 0x1c,
		[CIF_FRM1_ADDR_UV] = 0x20,
		[CIF_VIR_LINE_WIDTH] = 0x24,
		[CIF_SET_SIZE] = 0x28,
		[CIF_FRAME_STATUS] = 0x3c,
		[CIF_LAST_LINE] = 0x44,
		[CIF_LAST_PIX] = 0x48,
	},
};

static const struct of_device_id cif_plat_of_match[] = {
	{
		.compatible = "rockchip,px30-vip",
		.data = &px30_cif_match_data,
	},
	{
		.compatible = "rockchip,rk3568-vicap",
		.data = &rk3568_cif_match_data,
	},
	{},
};

static int cif_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct cif_device *cif_dev;
	struct resource *res;
	int ret, irq;

	cif_dev = devm_kzalloc(dev, sizeof(*cif_dev), GFP_KERNEL);
	if (!cif_dev)
		return -ENOMEM;

	cif_dev->match_data = of_device_get_match_data(dev);
	if (!cif_dev->match_data)
		return -ENODEV;

	platform_set_drvdata(pdev, cif_dev);
	cif_dev->dev = dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, cif_irq_pingpong, IRQF_SHARED,
			       dev_driver_string(dev), dev);
	if (ret)
		return dev_err_probe(dev, ret, "request irq failed\n");

	cif_dev->irq = irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to allocate resources for device\n");
		return -ENODEV;
	}

	cif_dev->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cif_dev->base_addr))
		return PTR_ERR(cif_dev->base_addr);

	ret = devm_clk_bulk_get(dev, cif_dev->match_data->clks_num,
				cif_dev->match_data->clks);
	if (ret)
		return ret;

	cif_dev->cif_rst = devm_reset_control_array_get(dev, false, false);
	if (IS_ERR(cif_dev->cif_rst))
		return PTR_ERR(cif_dev->cif_rst);

	cif_dev->grf = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "rockchip,grf");

	cif_stream_init(cif_dev);
	strscpy(cif_dev->media_dev.model, "cif",
		sizeof(cif_dev->media_dev.model));
	cif_dev->media_dev.dev = &pdev->dev;
	v4l2_dev = &cif_dev->v4l2_dev;
	v4l2_dev->mdev = &cif_dev->media_dev;
	strscpy(v4l2_dev->name, "rockchip-cif", sizeof(v4l2_dev->name));

	ret = v4l2_device_register(cif_dev->dev, &cif_dev->v4l2_dev);
	if (ret < 0)
		return ret;

	media_device_init(&cif_dev->media_dev);

	ret = media_device_register(&cif_dev->media_dev);
	if (ret < 0)
		goto err_unreg_v4l2_dev;

	/* Create & register platform subdev. */
	ret = cif_register_stream_vdev(cif_dev);
	if (ret < 0)
		goto err_unreg_media_dev;

	ret = cif_subdev_notifier(cif_dev);
	if (ret < 0) {
		v4l2_err(&cif_dev->v4l2_dev,
			 "Failed to register subdev notifier(%d)\n", ret);
		goto err_unreg_stream_vdev;
	}

	cif_set_default_format(cif_dev);

	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

err_unreg_stream_vdev:
	cif_unregister_stream_vdev(cif_dev);
err_unreg_media_dev:
	media_device_unregister(&cif_dev->media_dev);
err_unreg_v4l2_dev:
	v4l2_device_unregister(&cif_dev->v4l2_dev);
	return ret;
}

static int cif_plat_remove(struct platform_device *pdev)
{
	struct cif_device *cif_dev = platform_get_drvdata(pdev);

	media_device_unregister(&cif_dev->media_dev);
	v4l2_device_unregister(&cif_dev->v4l2_dev);
	cif_unregister_stream_vdev(cif_dev);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int cif_runtime_suspend(struct device *dev)
{
	struct cif_device *cif_dev = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(cif_dev->match_data->clks_num,
				   cif_dev->match_data->clks);

	reset_control_assert(cif_dev->cif_rst);

	return 0;
}

static int cif_runtime_resume(struct device *dev)
{
	struct cif_device *cif_dev = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(cif_dev->cif_rst);
	if (ret) {
		dev_err(dev, "failed to deassert reset\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(cif_dev->match_data->clks_num,
				      cif_dev->match_data->clks);
	if (ret) {
		dev_err(dev, "failed to enable module clock\n");
		goto err_reset;
	}

	return 0;

err_reset:
	reset_control_assert(cif_dev->cif_rst);

	return ret;
}

static const struct dev_pm_ops cif_plat_pm_ops = {
	.runtime_suspend = cif_runtime_suspend,
	.runtime_resume	= cif_runtime_resume,
};

static struct platform_driver cif_plat_drv = {
	.driver = {
		   .name = CIF_DRIVER_NAME,
		   .of_match_table = cif_plat_of_match,
		   .pm = &cif_plat_pm_ops,
	},
	.probe = cif_plat_probe,
	.remove = cif_plat_remove,
};
module_platform_driver(cif_plat_drv);

MODULE_AUTHOR("Rockchip Camera/ISP team");
MODULE_DESCRIPTION("Rockchip CIF platform driver");
MODULE_LICENSE("GPL");
