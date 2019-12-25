/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mmc.h>
#include <asm/arch/periph.h>
#include <asm/arch/pinmux.h>
#include <usb.h>
#include <usb/dwc2_udc.h>
#include <linux/printk.h>

// #define DEBUG
#ifdef  DEBUG
#undef  debug
#define debug(fmt, args...) debug_cond(true, fmt, ##args)
#endif

DECLARE_GLOBAL_DATA_PTR;

u32 get_board_rev(void)
{
	return 0;
}

static void board_gpio_init(void)
{
#ifdef CONFIG_CMD_USB
    /* USB3503A Connect */
    gpio_request(EXYNOS4X12_GPIO_M33, "USB3503A Connect");

    /* USB3503A Reset */
    gpio_request(EXYNOS4X12_GPIO_M24, "USB3503A Reset");

    /* Red LED2 Light On */
    gpio_request(EXYNOS4X12_GPIO_L20, "Red LED2");
    gpio_direction_output(EXYNOS4X12_GPIO_L20, 1);
#endif
}

int exynos_init(void)
{
    debug("---> ready to call board_gpio_init()!\n");
    board_gpio_init();

    /* FIXME: should be not called in here */
    board_usb_init(0, USB_INIT_DEVICE);

	return 0;
}

#ifdef CONFIG_USB_GADGET
static int s5pc210_phy_control(int on)
{
    /* FIXME: need to set power? */
#if 0
    struct udevice *dev;
    int ret;

    ret = regulator_get_by_platname("VDD_UOTG_3.0V", &dev);
    if (ret) {
        pr_err("Regulator get error: %d", ret);
        return ret;
    }

    if (on)
        return regulator_set_mode(dev, OPMODE_ON);
    else
        return regulator_set_mode(dev, OPMODE_LPM);
#else
    return 0;
#endif
}

struct dwc2_plat_otg_data s5pc210_otg_data = {
    .phy_control    = s5pc210_phy_control,
    .regs_phy   = EXYNOS4X12_USBPHY_BASE,
    .regs_otg   = EXYNOS4X12_USBOTG_BASE,
    .usb_phy_ctrl   = EXYNOS4X12_USBPHY_CONTROL,
    .usb_flags  = PHY0_SLEEP,
};
#endif

#if defined(CONFIG_USB_GADGET) || defined(CONFIG_CMD_USB)

int board_usb_init(int index, enum usb_init_type init)
{
#ifdef CONFIG_CMD_USB
    debug("---> ready to init usb3503\n");

    /* USB3503A Disconnect, Reset, Connect */
    gpio_direction_output(EXYNOS4X12_GPIO_M33, 0);
    gpio_direction_output(EXYNOS4X12_GPIO_M24, 0);
    gpio_direction_output(EXYNOS4X12_GPIO_M24, 1);
    gpio_direction_output(EXYNOS4X12_GPIO_M33, 1);

#endif
    debug("USB_udc_probe\n");
	return dwc2_udc_probe(&s5pc210_otg_data);
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
int exynos_early_init_f(void)
{
	return 0;
}
#endif
