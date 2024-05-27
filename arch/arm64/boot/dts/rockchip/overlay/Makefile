# SPDX-License-Identifier: GPL-2.0
dtbo-$(CONFIG_ARCH_ROCKCHIP) += \
	rk3566-rtc-rk808.dtbo\
	rk3566-rtc-pcf8563.dtbo\
	rk3566-mcp2515.dtbo

scr-$(CONFIG_ARCH_ROCKCHIP) += \
	rk3566-fixup.scr

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays

targets += $(dtbo-y) $(scr-y) $(dtbotxt-y)

always			:= $(dtbo-y) $(scr-y) $(dtbotxt-y)
clean-files		:= *.dtbo *.scr
