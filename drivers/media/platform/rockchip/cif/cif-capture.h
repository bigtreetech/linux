/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip CIF Driver
 *
 * Copyright (C) 2023 Mehdi Djait <mehdi.djait@bootlin.com>
 */

#ifndef _CIF_CAPTURE_H
#define _CIF_CAPTURE_H

struct cif_device;

void cif_unregister_stream_vdev(struct cif_device *dev);
int cif_register_stream_vdev(struct cif_device *dev);
void cif_stream_init(struct cif_device *dev);
void cif_set_default_format(struct cif_device *dev);

irqreturn_t cif_irq_pingpong(int irq, void *ctx);

#endif
