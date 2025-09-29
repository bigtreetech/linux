// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2025 Jernej Skrabec <jernej.skrabec@gmail.com> */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include "sun50i_planes.h"
#include "sun8i_ui_layer.h"
#include "sun8i_vi_layer.h"

static bool sun50i_planes_node_is_planes(struct device_node *node)
{
	return !!of_match_node(sun50i_planes_of_table, node);
}

struct drm_plane **
sun50i_planes_setup(struct device *dev, struct drm_device *drm,
		    unsigned int mixer)
{
	struct sun50i_planes *planes = dev_get_drvdata(dev);
	const struct sun50i_planes_quirks *quirks;
	struct drm_plane **drm_planes;
	const struct default_map *map;
	unsigned int i;

	if (!sun50i_planes_node_is_planes(dev->of_node)) {
		dev_err(dev, "Device is not planes driver!\n");
		return NULL;
	}

	if (!planes) {
		dev_err(dev, "Planes driver is not loaded yet!\n");
		return NULL;
	}

	if (mixer > 1) {
		dev_err(dev, "Mixer index is too high!\n");
		return NULL;
	}

	quirks = planes->quirks;
	map = &quirks->def_map[mixer];

	drm_planes = devm_kcalloc(drm->dev, map->num_ch + 1,
				  sizeof(*drm_planes), GFP_KERNEL);
	if (!drm_planes)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < map->num_ch; i++) {
		unsigned int phy_ch = map->map[i];
		struct sun8i_layer *layer;
		enum drm_plane_type type;

		if ((i == 0 && map->num_ch == 1) || i == 1)
			type = DRM_PLANE_TYPE_PRIMARY;
		else
			type = DRM_PLANE_TYPE_OVERLAY;

		if (phy_ch < UI_PLANE_OFFSET)
			layer = sun8i_vi_layer_init_one(drm, type, planes->regs,
							i, phy_ch, map->num_ch,
							&quirks->cfg);
		else
			layer = sun8i_ui_layer_init_one(drm, type, planes->regs,
							i, phy_ch, map->num_ch,
							&quirks->cfg);

		if (IS_ERR(layer)) {
			dev_err(drm->dev,
				"Couldn't initialize DRM plane\n");
			return ERR_CAST(layer);
		}

		drm_planes[i] = &layer->plane;
	}

	return drm_planes;
}
EXPORT_SYMBOL(sun50i_planes_setup);

static void sun50i_planes_init_mapping(struct sun50i_planes *planes)
{
	const struct sun50i_planes_quirks *quirks = planes->quirks;
	unsigned int i, j;
	u32 mapping;

	mapping = 0;
	for (j = 0; j < MAX_DISP; j++)
		for (i = 0; i < quirks->def_map[j].num_ch; i++) {
			unsigned int ch = quirks->def_map[j].map[i];

			if (ch < UI_PLANE_OFFSET)
				mapping |= j << (ch * 2);
			else
				mapping |= j << ((ch - UI_PLANE_OFFSET) * 2 + 16);
		}
	regmap_write(planes->mapping, SUNXI_DE33_DE_CHN2CORE_MUX_REG, mapping);

	for (j = 0; j < MAX_DISP; j++) {
		mapping = 0;
		for (i = 0; i < quirks->def_map[j].num_ch; i++) {
			unsigned int ch = quirks->def_map[j].map[i];

			if (ch >= UI_PLANE_OFFSET)
				ch += 2;

			mapping |= ch << (i * 4);
		}
		regmap_write(planes->mapping, SUNXI_DE33_DE_PORT02CHN_MUX_REG + j * 4, mapping);
	}
}

static const struct regmap_config sun50i_planes_regmap_config = {
	.name		= "planes",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x17fffc,
};

static int sun50i_planes_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sun50i_planes *planes;
	void __iomem *regs;

	planes = devm_kzalloc(dev, sizeof(*planes), GFP_KERNEL);
	if (!planes)
		return -ENOMEM;

	planes->quirks = of_device_get_match_data(&pdev->dev);
	if (!planes->quirks)
		return dev_err_probe(dev, -EINVAL, "Unable to get quirks\n");

	planes->mapping = syscon_regmap_lookup_by_phandle(dev->of_node,
							  "allwinner,plane-mapping");
	if (IS_ERR(planes->mapping))
		return dev_err_probe(dev, PTR_ERR(planes->mapping),
				     "Unable to get mapping\n");

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	planes->regs = devm_regmap_init_mmio(dev, regs, &sun50i_planes_regmap_config);
	if (IS_ERR(planes->regs))
		return PTR_ERR(planes->regs);

	dev_set_drvdata(dev, planes);

	sun50i_planes_init_mapping(planes);

	return 0;
}

static const struct sun50i_planes_quirks sun50i_h616_planes_quirks = {
	.def_map = {
		{
			.map = {0, 6, 7},
			.num_ch = 3,
		},
		{
			.map = {1, 2, 8},
			.num_ch = 3,
		},
	},
	.cfg = {
		.de_type	= SUN8I_MIXER_DE33,
		/*
		 * TODO: All planes support scaling, but driver needs
		 * improvements to properly support it.
		 */
		.scaler_mask    = 0,
		.scanline_yuv	= 4096,
	},
};

/* sun4i_drv uses this list to check if a device node is a plane */
const struct of_device_id sun50i_planes_of_table[] = {
	{
		.compatible = "allwinner,sun50i-h616-de33-planes",
		.data = &sun50i_h616_planes_quirks
	},
	{ }
};
MODULE_DEVICE_TABLE(of, sun50i_planes_of_table);
EXPORT_SYMBOL(sun50i_planes_of_table);

static struct platform_driver sun50i_planes_platform_driver = {
	.probe		= sun50i_planes_probe,
	.driver		= {
		.name		= "sun50i-planes",
		.of_match_table	= sun50i_planes_of_table,
	},
};
module_platform_driver(sun50i_planes_platform_driver);

MODULE_AUTHOR("Jernej Skrabec <jernej.skrabec@gmail.com>");
MODULE_DESCRIPTION("Allwinner DE33 planes driver");
MODULE_LICENSE("GPL");
