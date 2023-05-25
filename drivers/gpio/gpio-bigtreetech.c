/*
 * Driver for pwm demo on Bigtreetech board.
 *
 * Copyright (C) 2016, Zhongshan T-chip Intelligent Technology Co.,ltd.
 * Copyright 2006  JC.Lin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

struct bigtreetech_gpio_info {
	int bq_otg_pwrio;
    int otg_pwrio_enable_value;
	int bq_usbhost3_pwrio;
	int usbhost3_enable_value; 
};

static int bigtreetech_gpio_probe(struct platform_device *pdev)
{
    int gpio;
    enum of_gpio_flags flag;
    struct bigtreetech_gpio_info *gpio_info;
    struct device_node *bigtreetech_gpio_node = pdev->dev.of_node;

    dev_err(&pdev->dev,"bigtreetech GPIO Test Program Probe exec now\n");
    gpio_info = devm_kzalloc(&pdev->dev, sizeof(struct bigtreetech_gpio_info), GFP_KERNEL);
    if (!gpio_info) {
        dev_err(&pdev->dev, "devm_kzalloc failed!\n");
        return -ENOMEM;
    }

    gpio = of_get_named_gpio_flags(bigtreetech_gpio_node, "bq-usbhost3-pwrio", 0, &flag);
    if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "bq-usbhost3-pwrio: %d is invalid\n", gpio);
        return -ENODEV;
    }
	
    if (gpio_request(gpio, "bq-usbhost3-pwrio")) {
        dev_err(&pdev->dev, "biqu-gpio: %d request failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }
	
    gpio_info->bq_usbhost3_pwrio = gpio;
    gpio_info->usbhost3_enable_value = (flag == OF_GPIO_ACTIVE_LOW) ? 0:1;
    gpio_direction_output(gpio_info->bq_usbhost3_pwrio, gpio_info->usbhost3_enable_value);
    printk("Bigtreetech gpio putout\n");
	gpio_set_value(gpio_info->bq_usbhost3_pwrio, 1);
	printk("Bigtreetech bq_usbhost3_pwrio high\n");
		
	gpio = of_get_named_gpio_flags(bigtreetech_gpio_node, "bq-usbotg-pwrio", 0, &flag);
    if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "bq-usbotg-pwrio: %d is invalid\n", gpio);
        return -ENODEV;
    }
	
    if (gpio_request(gpio, "bq-usbotg-pwrio")) {
        dev_err(&pdev->dev, "bq-usbotg-pwrio: %d request failed!\n", gpio);
        gpio_free(gpio);
        return -ENODEV;
    }
	
    gpio_info->bq_otg_pwrio = gpio;
    gpio_info->otg_pwrio_enable_value = (flag == OF_GPIO_ACTIVE_LOW) ? 0:1;
    gpio_direction_output(gpio_info->bq_otg_pwrio, gpio_info->otg_pwrio_enable_value);
    printk("Bigtreetech gpio putout\n");
	gpio_set_value(gpio_info->bq_otg_pwrio, 1);
	printk("Bigtreetech bq_otg_pwrio high\n");
	
    return 0;
}

static struct of_device_id bigtreetech_match_table[] = {
        { .compatible = "rk356x,bigtreetech-usb_pwrio",},
        {},
};

static struct platform_driver bigtreetech_gpio_driver = {
        .driver = {
                .name = "bigtreetech-gpio",
                .owner = THIS_MODULE,
                .of_match_table = bigtreetech_match_table,
        },
        .probe = bigtreetech_gpio_probe,
};

static int bigtreetech_gpio_init(void)
{
        return platform_driver_register(&bigtreetech_gpio_driver);
}
module_init(bigtreetech_gpio_init);

static void bigtreetech_gpio_exit(void)
{
        platform_driver_unregister(&bigtreetech_gpio_driver);
}
module_exit(bigtreetech_gpio_exit);

MODULE_AUTHOR("falcon <service@bigtree-tech.com>");
MODULE_DESCRIPTION("bigtree GPIO driver");
MODULE_ALIAS("platform:bigtreetech-gpio");
MODULE_LICENSE("GPL");
