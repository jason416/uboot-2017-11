/*
 * Clock Initialization for board based on EXYNOS4210
 *
 * Copyright (C) 2013 Samsung Electronics
 * Rajeshwari Shinde <rajeshwari.s@samsung.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <config.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clk.h>
#include <asm/arch/clock.h>
#include "common_setup.h"

#ifdef CONFIG_ITOP4412
#include "itop4412_setup.h"
#else
#include "exynos4_setup.h"
#endif
/*
 * system_clock_init: Initialize core clock and bus clock.
 * void system_clock_init(void)
 */
#ifndef CONFIG_ITOP4412
void system_clock_init(void)
{
	struct exynos4_clock *clk =
			(struct exynos4_clock *)samsung_get_base_clock();

	writel(CLK_SRC_CPU_VAL, &clk->src_cpu);

	sdelay(0x10000);

	writel(CLK_SRC_TOP0_VAL, &clk->src_top0);
	writel(CLK_SRC_TOP1_VAL, &clk->src_top1);
	writel(CLK_SRC_DMC_VAL, &clk->src_dmc);
	writel(CLK_SRC_LEFTBUS_VAL, &clk->src_leftbus);
	writel(CLK_SRC_RIGHTBUS_VAL, &clk->src_rightbus);
	writel(CLK_SRC_FSYS_VAL, &clk->src_fsys);
	writel(CLK_SRC_PERIL0_VAL, &clk->src_peril0);
	writel(CLK_SRC_CAM_VAL, &clk->src_cam);
	writel(CLK_SRC_MFC_VAL, &clk->src_mfc);
	writel(CLK_SRC_G3D_VAL, &clk->src_g3d);
	writel(CLK_SRC_LCD0_VAL, &clk->src_lcd0);

	sdelay(0x10000);

	writel(CLK_DIV_CPU0_VAL, &clk->div_cpu0);
	writel(CLK_DIV_CPU1_VAL, &clk->div_cpu1);
	writel(CLK_DIV_DMC0_VAL, &clk->div_dmc0);
	writel(CLK_DIV_DMC1_VAL, &clk->div_dmc1);
	writel(CLK_DIV_LEFTBUS_VAL, &clk->div_leftbus);
	writel(CLK_DIV_RIGHTBUS_VAL, &clk->div_rightbus);
	writel(CLK_DIV_TOP_VAL, &clk->div_top);
	writel(CLK_DIV_FSYS1_VAL, &clk->div_fsys1);
	writel(CLK_DIV_FSYS2_VAL, &clk->div_fsys2);
	writel(CLK_DIV_FSYS3_VAL, &clk->div_fsys3);
	writel(CLK_DIV_PERIL0_VAL, &clk->div_peril0);
	writel(CLK_DIV_CAM_VAL, &clk->div_cam);
	writel(CLK_DIV_MFC_VAL, &clk->div_mfc);
	writel(CLK_DIV_G3D_VAL, &clk->div_g3d);
	writel(CLK_DIV_LCD0_VAL, &clk->div_lcd0);

	/* Set PLL locktime */
	writel(PLL_LOCKTIME, &clk->apll_lock);
	writel(PLL_LOCKTIME, &clk->mpll_lock);
	writel(PLL_LOCKTIME, &clk->epll_lock);
	writel(PLL_LOCKTIME, &clk->vpll_lock);

	writel(APLL_CON1_VAL, &clk->apll_con1);
	writel(APLL_CON0_VAL, &clk->apll_con0);
	writel(MPLL_CON1_VAL, &clk->mpll_con1);
	writel(MPLL_CON0_VAL, &clk->mpll_con0);
	writel(EPLL_CON1_VAL, &clk->epll_con1);
	writel(EPLL_CON0_VAL, &clk->epll_con0);
	writel(VPLL_CON1_VAL, &clk->vpll_con1);
	writel(VPLL_CON0_VAL, &clk->vpll_con0);

	sdelay(0x30000);
}

#else

/**
 * freq (ARMCLK) = 1400 MHz at 1.3 V
 * freq (ACLK_COREM0) = 350 MHz at 1.3V
 * freq (ACLK_COREM1) = 188 MHz at 1.3 V
 * freq (PERIPHCLK) = 1400 MHz at 1.3 V
 * freq (ATCLK) = 214 MHz at 1.3 V
 * freq (PCLK_DBG) = 107 MHz at 1.3 V
 * freq (SCLK_DMC) = 400 MHz at 1.0 V
 * freq (ACLK_DMCD) = 200 MHz at 1.0 V
 * freq (ACLK_DMCP) = 100 MHz at 1.0 V
 * freq (ACLK_ACP) = 200 MHz at 1.0 V
 * freq (PCLK_ACP) = 100 MHz at 1.0 V
 * freq (SCLK_C2C) = 400 MHz at 1.0 V
 * freq (ACLK_C2C) = 200 MHz at 1.0 V
 * freq (ACLK_GDL) = 200 MHz at 1.0 V
 * freq (ACLK_GPL) = 100 MHz at 1.0 V
 * freq (ACLK_GDR) = 200 MHz at 1.0 V
 * freq (ACLK_GPR) = 100 MHz at 1.0 V
 * freq (ACLK_400_MCUISP) = 400 MHz at 1.0 V
 * freq (ACLK_200) = 160 MHz at 1.0 V
 * freq (ACLK_100) = 100 MHz at 1.0 V
 * freq (ACLK_160) = 160 MHz at 1.0 V
 * freq (ACLK_133) = 133 MHz at 1.0 V
 * freq (SCLK_ONENAND) = 160 MHz at 1.0 V
 */
void system_clock_init(void)
{
	unsigned int set, clr, clr_src_cpu, clr_pll_con0, clr_src_dmc;
	struct exynos4x12_clock *clk = (struct exynos4x12_clock *)
						samsung_get_base_clock();

/************************************************************
 * Step 1:
 *
 * Set PDIV, MDIV, and SDIV values (Refer to (A, M, E, V)
 * Change other PLL control values
 ************************************************************/

	/**
	 * Set dividers for MOUTcore = 1000 MHz
	 *
	 * DOUTcore    = MOUTcore / (CORE_RATIO +1)      = 1000 MHz (0)
	 * ACLK_COREM0 = ARMCLK   / (COREM0_RATIO +1)    = 250 MHz (3)
	 * ACLK_COREM1 = ARMCLK   / (COREM1_RATIO +1)    = 125 MHz (7)
	 * PERIPHCLK   = DOUTcore / (PERIPH_RATIO + 1)   = 1000 MHz (0)
	 * ATCLK       = MOUTcore / (ATB_RATIO + 1)      = 200 MHz (4)
	 * PCLK_DBG    = ATCLK    / (PCLK_DBG_RATIO + 1) = 100 MHz (1)
	 * SCLKapll    = MOUTapll / (APLL_RATIO + 1)     = 500 MHz (1)
	 * ARMCLK      = DOUTcore / (CORE2_RATIO + 1)    = 1000 MHz (0)
	 */

	/** CLK_DIV_CPU0 */
	clr = CORE_RATIO(7) | COREM0_RATIO(7) | COREM1_RATIO(7) |
	      PERIPH_RATIO(7) | ATB_RATIO(7) | PCLK_DBG_RATIO(7) |
	      APLL_RATIO(7) | CORE2_RATIO(7);
	set = CORE_RATIO(0) | COREM0_RATIO(3) | COREM1_RATIO(7) |
	      PERIPH_RATIO(0) | ATB_RATIO(4) | PCLK_DBG_RATIO(1) |
	      APLL_RATIO(1) | CORE2_RATIO(0);

	clrsetbits_le32(&clk->div_cpu0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_cpu0) & DIV_STAT_CPU0_CHANGING)
		continue;

	/**
	 * Set dividers for MOUThpm = 1000 MHz (MOUTapll)
	 *
	 * DOUTcopy    = MOUThpm   / (COPY_RATIO + 1)   = 200 MHz (4)
	 * SCLK_HPM    = DOUTcopy  / (HPM_RATIO + 1)    = 200 MHz (0)
	 * ACLK_CORES  = ARMCLK    / (CORES_RATIO + 1)  = 1000 MHz (0)
	 */

	/** CLK_DIV_CPU1 */
	clr = COPY_RATIO(7) | HPM_RATIO(7) | CORES_RATIO(7);
	set = COPY_RATIO(4) | HPM_RATIO(0) | CORES_RATIO(0);

	clrsetbits_le32(&clk->div_cpu1, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_cpu1) & DIV_STAT_CPU1_CHANGING)
		continue;

	/**
	 * Set dividers for -->
	 * MOUTdmc  = 800 MHz
	 * MOUTdphy = 800 MHz
	 *
	 * ACLK_ACP  = MOUTdmc   / (ACP_RATIO + 1)      = 200 MHz (3)
	 * PCLK_ACP  = ACLK_ACP  / (ACP_PCLK_RATIO + 1) = 100 MHz (1)
	 * SCLK_DPHY = MOUTdphy  / (DPHY_RATIO + 1)     = 400 MHz (1)
	 * SCLK_DMC  = MOUTdmc   / (DMC_RATIO + 1)      = 400 MHz (1)
	 * ACLK_DMCD = SCLK_DMC  / (DMCD_RATIO + 1)     = 200 MHz (1)
	 * ACLK_DMCP = ACLK_DMCD / (DMCP_RATIO + 1)     = 100 MHz (1)
	 */

	/** CLK_DIV_DMC0 */
	clr = ACP_RATIO(7) | ACP_PCLK_RATIO(7) | DPHY_RATIO(7) |
	      DMC_RATIO(7) | DMCD_RATIO(7) | DMCP_RATIO(7);
	set = ACP_RATIO(3) | ACP_PCLK_RATIO(1) | DPHY_RATIO(1) |
	      DMC_RATIO(1) | DMCD_RATIO(1) | DMCP_RATIO(1);

	clrsetbits_le32(&clk->div_dmc0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_dmc0) & DIV_STAT_DMC0_CHANGING)
		continue;

	/**
	 * For:
	 * MOUTg2d = 800 MHz
	 * MOUTc2c = 800 Mhz
	 * MOUTpwi = 24 MHz
	 *
	 * SCLK_G2D_ACP = MOUTg2d  / (G2D_ACP_RATIO + 1)  = 200 MHz (3)
	 * SCLK_C2C     = MOUTc2c  / (C2C_RATIO + 1)      = 400 MHz (1)
	 * SCLK_PWI     = MOUTpwi  / (PWI_RATIO + 1)      = 24 MHz (0)
	 * ACLK_C2C     = SCLK_C2C / (C2C_ACLK_RATIO + 1) = 200 MHz (1)
	 * DVSEM_RATIO : It decides frequency for PWM frame time slot in DVS emulation mode.
	 * DPM_RATIO   : It decides frequency of DPM channel clock.
	 */

	/** CLK_DIV_DMC1 */
	clr = G2D_ACP_RATIO(15) | C2C_RATIO(7) | PWI_RATIO(15) |
	      C2C_ACLK_RATIO(7) | DVSEM_RATIO(127) | DPM_RATIO(127);
	set = G2D_ACP_RATIO(3) | C2C_RATIO(1) | PWI_RATIO(0) |
	      C2C_ACLK_RATIO(1) | DVSEM_RATIO(1) | DPM_RATIO(1);

	clrsetbits_le32(&clk->div_dmc1, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_dmc1) & DIV_STAT_DMC1_CHANGING)
		continue;

	/**
	 * MOUTmpll        = 800 MHz
	 * MOUTvpll        = 54 MHz
	 *
	 * ACLK_200        = MOUTACLK_200        / (ACLK_200_RATIO + 1)        = 200 MHz (3)
	 * ACLK_100        = MOUTACLK_100        / (ACLK_100_RATIO + 1)        = 100 MHz (7)
	 * ACLK_160        = MOUTACLK_160        / (ACLK_160_RATIO + 1)        = 160 MHz (4)
	 * ACLK_133        = MOUTACLK_133        / (ACLK_133_RATIO + 1)        = 133 MHz (5)
	 * ONENAND         = MOUTONENAND_1       / (ONENAND_RATIO + 1)         = 160 MHz (0)
	 * ACLK_266_GPS    = MOUTACLK_266_GPS    / (ACLK_266_GPS_RATIO + 1)    = 266 MHz (2)
	 * ACLK_400_MCUISP = MOUTACLK_400_MCUISP / (ACLK_400_MCUISP_RATIO + 1) = 400 MHz (1)
	 */

	/** CLK_DIV_TOP */
	clr = ACLK_200_RATIO(7) | ACLK_100_RATIO(15) | ACLK_160_RATIO(7) | 
	      ACLK_133_RATIO(7) | ONENAND_RATIO(7) | ACLK_266_GPS_RATIO(7) | ACLK_400_MCUISP_RATIO(7);
	set = ACLK_200_RATIO(3) | ACLK_100_RATIO(7) | ACLK_160_RATIO(4) |
	      ACLK_133_RATIO(5) | ONENAND_RATIO(0) | ACLK_266_GPS_RATIO(2) | ACLK_400_MCUISP_RATIO(1);

	clrsetbits_le32(&clk->div_top, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_top) & DIV_STAT_TOP_CHANGING)
		continue;

	/**
	 * ACLK_GDL = MOUTGDL / (GDL_RATIO + 1) = 200 MHz (3)
	 * ACLK_GPL = MOUTGPL / (GPL_RATIO + 1) = 100 MHz (1)
	 */

	/** CLK_DIV_LEFTBUS */
	clr = GDL_RATIO(7) | GPL_RATIO(7);
	set = GDL_RATIO(3) | GPL_RATIO(1);

	clrsetbits_le32(&clk->div_leftbus, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_leftbus) & DIV_STAT_LEFTBUS_CHANGING)
		continue;

	/**
	 * ACLK_GDR = MOUTGDR / (GDR_RATIO + 1) = 200 MHz (3)
	 * ACLK_GPR = MOUTGPR / (GPR_RATIO + 1) = 100 MHz (1)
	 */

	/** CLK_DIV_RIGHTBUS */
	clr = GPR_RATIO(7) | GDR_RATIO(7);
	set = GPR_RATIO(3) | GDR_RATIO(1);

	clrsetbits_le32(&clk->div_rightbus, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_rightbus) & DIV_STAT_RIGHTBUS_CHANGING)
		continue;

	/**
	 * MOUTUART[1-4] = 800 Mhz (MPLL)
	 *
	 * SCLK_UART0 = MOUTUART0 / (UART0_RATIO + 1) = 100 MHz (7)
	 * SCLK_UART1 = MOUTUART1 / (UART1_RATIO + 1) = 100 MHz (7)
	 * SCLK_UART2 = MOUTUART2 / (UART2_RATIO + 1) = 100 MHz (7)
	 * SCLK_UART3 = MOUTUART3 / (UART3_RATIO + 1) = 100 MHz (7)
	 * SCLK_UART4 = MOUTUART4 / (UART4_RATIO + 1) = 100 MHz (7)
	 */
	/** CLK_DIV_PERIL0 */
	clr = UART0_RATIO(15) | UART1_RATIO(15) | UART2_RATIO(15) |
	      UART3_RATIO(15) | UART4_RATIO(15);
	set = UART0_RATIO(7) | UART1_RATIO(7) | UART2_RATIO(7) |
          UART3_RATIO(7) | UART4_RATIO(7);

	clrsetbits_le32(&clk->div_peril0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_peril0) & DIV_STAT_PERIL0_CHANGING)
		continue;
	/**
	 * For MOUTMMC0-3 = 800 MHz (MPLL)
	 *
	 * SCLK_MIPIHSI = MOUTMIPIHSI / (MIPIHSI_RATIO + 1) = 200 MHz (3)
	 */
	/* CLK_DIV_FSYS0 */
	clr = MIPIHSI_RATIO(15);
	set = MIPIHSI_RATIO(3);

	clrsetbits_le32(&clk->div_fsys0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys0) & DIV_STAT_FSYS0_CHANGING)
		continue;

	/**
	 * For MOUTMMC0-3 = 800 MHz (MPLL)
	 *
	 * DOUTMMC0  = MOUTMMC0 / (MMC0_RATIO + 1)     = 100 MHz (7)
	 * SCLK_MMC0 = DOUTMMC0 / (MMC0_PRE_RATIO + 1) = 50 MHz (1)
	 * DOUTMMC1  = MOUTMMC1 / (MMC1_RATIO + 1)     = 100 MHz (7)
	 * SCLK_MMC1 = DOUTMMC1 / (MMC1_PRE_RATIO + 1) = 50 MHz (1)
	 */
	/* CLK_DIV_FSYS1 */
	clr = MMC0_RATIO(15) | MMC0_PRE_RATIO(255) | MMC1_RATIO(15) |
	      MMC1_PRE_RATIO(255);
	
	set = MMC0_RATIO(7) | MMC0_PRE_RATIO(1) | MMC1_RATIO(7) |
			  MMC1_PRE_RATIO(1);

	clrsetbits_le32(&clk->div_fsys1, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys1) & DIV_STAT_FSYS1_CHANGING)
		continue;

	/**
	 * For MOUTmmc0-3 = 800 MHz (MPLL)
	 *
	 * DOUTmmc3  = MOUTmmc3 / (MMC2_RATIO + 1)     = 100 MHz (7)
	 * sclk_mmc3 = DOUTmmc3 / (MMC2_PRE_RATIO + 1) = 50 MHz (1)
	 * DOUTmmc2  = MOUTmmc2 / (MMC3_RATIO + 1)     = 100 MHz (7)
	 * sclk_mmc2 = DOUTmmc2 / (MMC3_PRE_RATIO + 1) = 50 MHz (1)
	*/
	/* CLK_DIV_FSYS2 */
	clr = MMC2_RATIO(15) | MMC2_PRE_RATIO(255) | MMC3_RATIO(15) |
	      MMC3_PRE_RATIO(255);
	set = MMC2_RATIO(7) | MMC2_PRE_RATIO(1) | MMC3_RATIO(7) |
	      MMC3_PRE_RATIO(1);

	clrsetbits_le32(&clk->div_fsys2, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys2) & DIV_STAT_FSYS2_CHANGING)
		continue;

	/**
	 * For MOUTmmc4 = 800 MHz (MPLL)
	 *
	 * DOUTmmc4  = MOUTmmc4 / (MMC4_RATIO + 1)     = 100 MHz (7)
	 * sclk_mmc4 = DOUTmmc4 / (MMC4_PRE_RATIO + 1) = 50 MHz (1)
	*/
	/* CLK_DIV_FSYS3 */
	clr = MMC4_RATIO(15) | MMC4_PRE_RATIO(255);
	set = MMC4_RATIO(7) | MMC4_PRE_RATIO(1);

	clrsetbits_le32(&clk->div_fsys3, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys3) & DIV_STAT_FSYS3_CHANGING)
		continue;

/************************************************************
 * Step 2:
 *
 * Set K, AFC, MRR, MFR values if necessary 
 * (Refer to (A, M, E, V)PLL_CON1 SFRs)
 * Turn on a PLL (Refer to (A, M, E, V) PLL_CON0 SFRs)
 ************************************************************/

	/* Set APLL to 1000MHz */
	/** APLL_CON1 */
	clr = AFC(31) | LOCK_CON_DLY(31) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(3) |FEED_EN(1)| AFC_ENB(1) |
	      DCC_ENB(1) | BYPASS(1) |RESV0(1) | RESV1(1);
	set = AFC(0) | LOCK_CON_DLY(8) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(0) |FEED_EN(0)| AFC_ENB(0) |
	      DCC_ENB(1) | BYPASS(0) |RESV0(0) | RESV1(0);

	clrsetbits_le32(&clk->apll_con1, clr, set);

	/** APLL_CON0 */
	clr_pll_con0 = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(0) | PDIV(3) | MDIV(125) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->apll_con0, clr_pll_con0, set);
	
	/* Wait for PLL to be locked */
	while (!(readl(&clk->apll_con0) & PLL_LOCKED_BIT))
		continue;

	/* Set MPLL to 800MHz */
	/** MPLL_CON1 */
	clr = AFC(31) | LOCK_CON_DLY(31) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(3) |FEED_EN(1)| AFC_ENB(1) |
	      DCC_ENB(1) | BYPASS(1) |RESV0(1) | RESV1(1);
	set = AFC(0) | LOCK_CON_DLY(8) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(0) |FEED_EN(0)| AFC_ENB(0) |
	      DCC_ENB(1) | BYPASS(0) |RESV0(0) | RESV1(0);

	clrsetbits_le32(&clk->mpll_con1, clr, set);

	/** MPLL_CON0 */
	clr_pll_con0 = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(0) | PDIV(3) | MDIV(100) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->mpll_con0, clr_pll_con0, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->mpll_con0) & PLL_LOCKED_BIT))
		continue;

	/* Set EPLL to 192MHz */
	/** EPLL_CON2 */
	clr = ICP_BOOST(7) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(1) | EV_AFC_ENB(1) | EV_DCC_ENB(1) | EXTAFC(1);
	set = ICP_BOOST(0) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(0) | EV_AFC_ENB(0) | EV_DCC_ENB(1) | EXTAFC(0);

	clrsetbits_le32(&clk->epll_con2, clr, set);

	/** EPLL_CON1 */
	/* there is null */

	/** EPLL_CON0 */
	clr_pll_con0 = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(2) | PDIV(2) | MDIV(64) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->epll_con0, clr_pll_con0, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->epll_con0) & PLL_LOCKED_BIT))
		continue;

	/* Set VPLL to 54MHz */
	/** VPLL_CON2 */
	clr = ICP_BOOST(7) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(1) | EV_AFC_ENB(1) | EV_DCC_ENB(1) | EXTAFC(1);
	set = ICP_BOOST(0) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(0) | EV_AFC_ENB(0) | EV_DCC_ENB(1) | EXTAFC(0);

	clrsetbits_le32(&clk->vpll_con2, clr, set);

	/** VPLL_CON1 */
	/* there is null */

	/** VPLL_CON0 */
	clr_pll_con0 = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(3) | PDIV(3) | MDIV(54) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->vpll_con0, clr_pll_con0, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->vpll_con0) & PLL_LOCKED_BIT))
		continue;

/************************************************************
 *Step 3:
 *
 * Wait until the PLL is locked
 ************************************************************/
	clr = PLL_LOCKTIME(65535);
	
	/** APLL LOCKTIME 1000MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->apll_lock, clr, set);

	/** MPLL LOCKTIME 800MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->mpll_lock, clr, set);

	/** EPLL LOCKTIME 192MHz */
	set = PLL_LOCKTIME(PDIV(2) * 270);
	clrsetbits_le32(&clk->epll_lock, clr, set);

	/** VPLL LOCKTIME 54MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->vpll_lock, clr, set);

/************************************************************
 * Step 4:
 *
 * Select the PLL output clock instead of input reference clock,
 * after PLL output clock is stabilized.
 * (Refer to CLK_SRC_CPU SFR for APLL and MPLL, 
 * CLK_SRC_TOP0 for EPLL and VPLL)
 * Once a PLL is turned on, do not turn it off.
 ************************************************************/

	/**
	 * before set system clocks,we switch system clocks src to FINpll
	 *
	 * Bit values:                 0  ;  1
	 * MUX_APLL_SEL:          FIN_PLL ; MOUTAPLLFOUT
	 * MUX_CORE_SEL:         MOUTAPLL ; SCLKMPLL
	 * MUX_HPM_SEL:          MOUTAPLL ; SCLKMPLL
	 * MUX_MPLL_USER_SEL_C:    FINPLL ; FOUTMPLL
	 */
	/** CLK_SRC_CPU */
	clr_src_cpu = MUX_APLL_SEL(1) | MUX_CORE_SEL(1) |
		      MUX_HPM_SEL(1) | MUX_MPLL_USER_SEL_C(1);
	set = MUX_APLL_SEL(1) | MUX_CORE_SEL(0) | MUX_HPM_SEL(0) |
	      MUX_MPLL_USER_SEL_C(1);

	clrsetbits_le32(&clk->src_cpu, clr_src_cpu, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_cpu) & MUX_STAT_CPU_CHANGING)
		continue;

	/**
	 * Set CMU_DMC default clocks src to APLL
	 *
	 * Bit values:             0  ; 1
	 * MUX_C2C_SEL:      SCLKMPLL ; SCLKAPLL
	 * MUX_DMC_BUS_SEL:  SCLKMPLL ; SCLKAPLL
	 * MUX_DPHY_SEL:     SCLKMPLL ; SCLKAPLL
	 * MUX_MPLL_SEL:     FINPLL   ; MOUT_MPLL_FOUT
	 * MUX_PWI_SEL:      0110 (MPLL); 0111 (EPLL); 1000 (VPLL); 0(XXTI)
	 * MUX_G2D_ACP0_SEL: SCLKMPLL ; SCLKAPLL
	 * MUX_G2D_ACP1_SEL: SCLKEPLL ; SCLKVPLL
	 * MUX_G2D_ACP_SEL:  OUT_ACP0 ; OUT_ACP1
	 */
	/** CLK_SRC_DMC */
	clr_src_dmc = MUX_C2C_SEL(1) | MUX_DMC_BUS_SEL(1) |
		          MUX_DPHY_SEL(1) | MUX_MPLL_SEL(1) |
		          MUX_PWI_SEL(15) | MUX_G2D_ACP0_SEL(1) |
		          MUX_G2D_ACP1_SEL(1) | MUX_G2D_ACP_SEL(1);
	set = MUX_C2C_SEL(0) | MUX_DMC_BUS_SEL(0) | MUX_DPHY_SEL(0) |
	      MUX_MPLL_SEL(1) | MUX_PWI_SEL(0) | MUX_G2D_ACP0_SEL(0) |
	      MUX_G2D_ACP1_SEL(0) | MUX_G2D_ACP_SEL(0);

	clrsetbits_le32(&clk->src_dmc, clr_src_dmc, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_dmc) & MUX_STAT_DMC_CHANGING)
		continue;

	/**
	 * Set CMU_TOP default clocks src to APLL
	 *
	 * Bit values:                           0 ; 1
	 * MUX_ONENAND_1_SEL           MOUTONENAND ; SCLKVPLL
	 * MUX_EPLL_SEL                     FINPLL ; FOUTEPLL
	 * MUX_VPLL_SEL                     FINPLL ; FOUTEPLL
	 * MUX_ACLK_200_SEL               SCLKMPLL ; SCLKAPLL
	 * MUX_ACLK_100_SEL               SCLKMPLL ; SCLKAPLL
	 * MUX_ACLK_160_SEL               SCLKMPLL ; SCLKAPLL
	 * MUX_ACLK_133_SEL               SCLKMPLL ; SCLKAPLL
	 * MUX_ONENAND_SEL                ACLK_133 ; ACLK_160
	 */

	/* CLK_SRC_TOP0 */
	clr = MUX_ONENAND_1_SEL(1) | MUX_EPLL_SEL(1) | MUX_VPLL_SEL(1) |
	      MUX_ACLK_200_SEL(1) | MUX_ACLK_100_SEL(1) | MUX_ACLK_160_SEL(1) |
	      MUX_ACLK_133_SEL(1) | MUX_ONENAND_SEL(1);
	set = MUX_ONENAND_1_SEL(0) | MUX_EPLL_SEL(1) | MUX_VPLL_SEL(1) |
	      MUX_ACLK_200_SEL(0) | MUX_ACLK_100_SEL(0) | MUX_ACLK_160_SEL(0) |
	      MUX_ACLK_133_SEL(0) | MUX_ONENAND_SEL(1);

	clrsetbits_le32(&clk->src_top0, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_top0) & MUX_STAT_TOP0_CHANGING)
		continue;

	/**
	 * Bit values:                           0 ; 1
	 * MUX_ACLK_266_GPS_SEL    SCLKMPLL_USER_T ; SCLKAPLL
	 * MUX_ACLK_400_MCUISP_SEL SCLKMPLL_USER_T ; SCLKAPLL
	 * MUX_MPLL_USER_SEL_T              FINPLL ; SCLKMPLLL
	 * MUX_ACLK_266_GPS_SUB_SEL         FINPLL ; DIVOUT_ACLK_266_GPS
	 * MUX_ACLK_200_SUB_SEL             FINPLL ; DIVOUT_ACLK_200
	 * MUX_ACLK_400_MCUISP_SUB_SEL      FINPLL
	 */

	/* CLK_SRC_TOP1 */
	clr = MUX_ACLK_266_GPS_SEL(1) | MUX_ACLK_400_MCUISP_SEL(1) |
	      MUX_MPLL_USER_SEL_T(1) | MUX_ACLK_266_GPS_SUB_SEL(1) |
	      MUX_ACLK_200_SUB_SEL(1) | MUX_ACLK_400_MCUISP_SUB_SEL(1);
	set = MUX_ACLK_266_GPS_SEL(0) | MUX_ACLK_400_MCUISP_SEL(0) |
	      MUX_MPLL_USER_SEL_T(1) | MUX_ACLK_266_GPS_SUB_SEL(1) |
	      MUX_ACLK_200_SUB_SEL(1) | MUX_ACLK_400_MCUISP_SUB_SEL(1);

	clrsetbits_le32(&clk->src_top1, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_top1) & MUX_STAT_TOP1_CHANGING)
		continue;

	/* CLK_SRC_LEFTBUS */
	clr = MUX_GDL_SEL(1) | MUX_MPLL_USER_SEL_L(1);
	set = MUX_GDL_SEL(0) | MUX_MPLL_USER_SEL_L(1);

	clrsetbits_le32(&clk->src_leftbus, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_leftbus) & MUX_STAT_LEFTBUS_CHANGING)
		continue;

	/* CLK_SRC_RIGHTBUS */
	clr = MUX_GDR_SEL(1) | MUX_MPLL_USER_SEL_R(1);
	set = MUX_GDR_SEL(0) | MUX_MPLL_USER_SEL_R(1);

	clrsetbits_le32(&clk->src_rightbus, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_rightbus) & MUX_STAT_RIGHTBUS_CHANGING)
		continue;

	/** CLK_SRC_PERIL0 */
	clr = UART0_SEL(15) | UART1_SEL(15) | UART2_SEL(15) |
	      UART3_SEL(15) | UART4_SEL(15);
	set = UART0_SEL(6) | UART1_SEL(6) | UART2_SEL(6) |
	      UART3_SEL(6) | UART4_SEL(6);

	clrsetbits_le32(&clk->src_peril0, clr, set);

	/** CLK_SRC_FSYS */
	clr = MMC1_SEL(15) | MMC2_SEL(15) | MMC3_SEL(15) |
	      MMC4_SEL(15) | MIPIHSI_SEL(1);
	set = MMC1_SEL(6) | MMC2_SEL(6) | MMC3_SEL(6) |
	      MMC4_SEL(6) | MIPIHSI_SEL(0);

	clrsetbits_le32(&clk->src_fsys, clr, set);
}
#endif

