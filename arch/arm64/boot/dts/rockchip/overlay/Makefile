# SPDX-License-Identifier: GPL-2.0
dtbo-$(CONFIG_ARCH_ROCKCHIP) += \
	rk3566-hdmi.dtbo \
	rk3566-hdmi_sound.dtbo \
	rk3566-dsi.dtbo \
	rk3566-i2c1.dtbo \
	rk3566-i2c3.dtbo \
	rk3566-i2c4.dtbo \
	rk3566-mcp2515.dtbo \
	rk3566-pwm0.dtbo \
	rk3566-pwm1.dtbo \
	rk3566-pwm2.dtbo \
	rk3566-pwm4.dtbo \
	rk3566-pwm5.dtbo \
	rk3566-pwm12.dtbo \
	rk3566-pwm13.dtbo \
	rk3566-pwm14.dtbo \
	rk3566-pwm15.dtbo \
	rk3566-spi1.dtbo \
	rk3566-spi3.dtbo \
	rk3566-tft_35.dtbo \
	rk3566-uart0.dtbo \
	rk3566-uart1.dtbo \
	rk3566-uart3.dtbo \
	rk3566-uart5.dtbo \
	rk3566-uart7.dtbo \
	rk3566-uart9.dtbo

scr-$(CONFIG_ARCH_ROCKCHIP) += \
	rk3566-fixup.scr

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays

targets += $(dtbo-y) $(scr-y) $(dtbotxt-y)

always			:= $(dtbo-y) $(scr-y) $(dtbotxt-y)
clean-files		:= *.dtbo *.scr

