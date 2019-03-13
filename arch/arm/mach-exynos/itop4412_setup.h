/*
 * Machine Specific Values for EXYNOS4412 based board
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _ITOP4412_SETUP_H
#define _ITOP4412_SETUP_H

#include <config.h>
#include <asm/arch/cpu.h>

#ifdef CONFIG_CLK_800_330_165
#define DRAM_CLK_330
#endif
#ifdef CONFIG_CLK_1000_200_200
#define DRAM_CLK_200
#endif
#ifdef CONFIG_CLK_1000_330_165
#define DRAM_CLK_330
#endif
#ifdef CONFIG_CLK_1000_400_200
#define DRAM_CLK_400
#endif

/* this state is changing for register */
#define MUX_STAT_CHANGING       0x100
#define DIV_STAT_CHANGING       0x1

/* A/M/EV PLL_CON0 */
#define SDIV(x)                 ((x) & 0x7)
#define PDIV(x)                 (((x) & 0x3f) << 8)
#define MDIV(x)                 (((x) & 0x3ff) << 16)
#define FSEL(x)                 (((x) & 0x1) << 27)
#define PLL_LOCKED_BIT          (0x1 << 29)
#define PLL_ENABLE(x)           (((x) & 0x1) << 31)

/* A/M PLL_CON1 */
#define AFC(x)                  ((x) & 0x1f)
#define LOCK_CON_DLY(x)         (((x) & 0x1f) << 8)
#define LOCK_CON_IN(x)          (((x) & 0x3) << 12)
#define LOCK_CON_OUT(x)         (((x) & 0x3) << 14)
#define FEED_EN(x)              (((x) & 0x1) << 16)
#define AFC_ENB(x)              (((x) & 0x1) << 20)
#define DCC_ENB(x)              (((x) & 0x1) << 21)
#define BYPASS(x)               (((x) & 0x1) << 22)
#define RESV0(x)                (((x) & 0x1) << 23)
#define RESV1(x)                (((x) & 0x1) << 24)

/* E/V PLL_CON1 */
#define K(x)                    ((x) & 0xffff)
#define MFR(x)                  (((x) & 0xff) << 16)
#define MRR(x)                  (((x) & 0x1f) << 24)
#define SEL_PF(x)               (((x) & 0x3) << 29)

/* E/V PLL_CON2 */
#define ICP_BOOST(x)            ((x) & 0x3)
#define EV_FSEL(x)              (((x) & 0x1) << 2)
#define FVCO_EN(x)              (((x) & 0x1) << 3)
#define EV_BYPASS(x)            (((x) & 0x1) << 4)
#define SSCG_EN(x)              (((x) & 0x1) << 5)
#define EV_AFC_ENB(x)           (((x) & 0x1) << 6)
#define EV_DCC_ENB(x)              (((x) & 0x1) << 7)
#define EXTAFC(x)               (((x) & 0x1f) << 8)

/* CLK_SRC_CPU */
#define MUX_APLL_SEL(x)         ((x) & 0x1)
#define MUX_CORE_SEL(x)         (((x) & 0x1) << 16)
#define MUX_HPM_SEL(x)          (((x) & 0x1) << 20)
#define MUX_MPLL_USER_SEL_C(x)  (((x) & 0x1) << 24)

/* CLK_MUX_STAT_CPU */
#define APLL_SEL(x)             ((x) & 0x7)
#define CORE_SEL(x)             (((x) & 0x7) << 16)
#define HPM_SEL(x)              (((x) & 0x7) << 20)
#define MPLL_USER_SEL_C(x)      (((x) & 0x7) << 24)
#define MUX_STAT_CPU_CHANGING   (APLL_SEL(MUX_STAT_CHANGING) | \
				CORE_SEL(MUX_STAT_CHANGING) | \
				HPM_SEL(MUX_STAT_CHANGING) | \
				MPLL_USER_SEL_C(MUX_STAT_CHANGING))

/* A/M/E/V PLL_LOCK */
#define PLL_LOCKTIME(x)         ((x) & 0xffff)

/* CLK_DIV_CPU0 */
#define CORE_RATIO(x)           ((x) & 0x7)
#define COREM0_RATIO(x)         (((x) & 0x7) << 4)
#define COREM1_RATIO(x)         (((x) & 0x7) << 8)
#define PERIPH_RATIO(x)         (((x) & 0x7) << 12)
#define ATB_RATIO(x)            (((x) & 0x7) << 16)
#define PCLK_DBG_RATIO(x)       (((x) & 0x7) << 20)
#define APLL_RATIO(x)           (((x) & 0x7) << 24)
#define CORE2_RATIO(x)          (((x) & 0x7) << 28)

/* CLK_DIV_CPU1 */
#define COPY_RATIO(x)           ((x) & 0x7)
#define HPM_RATIO(x)            (((x) & 0x7) << 4)
#define CORES_RATIO(x)          (((x) & 0x7) << 8)

/* CLK_DIV_STAT_CPU0 */
#define DIV_CORE(x)             ((x) & 0x1)
#define DIV_COREM0(x)           (((x) & 0x1) << 4)
#define DIV_COREM1(x)           (((x) & 0x1) << 8)
#define DIV_PERIPH(x)           (((x) & 0x1) << 12)
#define DIV_ATB(x)              (((x) & 0x1) << 16)
#define DIV_PCLK_DBG(x)         (((x) & 0x1) << 20)
#define DIV_APLL(x)             (((x) & 0x1) << 24)
#define DIV_CORE2(x)            (((x) & 0x1) << 28)

#define DIV_STAT_CPU0_CHANGING  (DIV_CORE(DIV_STAT_CHANGING) | \
				DIV_COREM0(DIV_STAT_CHANGING) | \
				DIV_COREM1(DIV_STAT_CHANGING) | \
				DIV_PERIPH(DIV_STAT_CHANGING) | \
				DIV_ATB(DIV_STAT_CHANGING) | \
				DIV_PCLK_DBG(DIV_STAT_CHANGING) | \
				DIV_APLL(DIV_STAT_CHANGING) | \
				DIV_CORE2(DIV_STAT_CHANGING))

/* CLK_DIV_STAT_CPU1 */
#define DIV_COPY(x)             ((x) & 0x1)
#define DIV_HPM(x)              (((x) & 0x1) << 4)
#define DIV_CORES(x)            (((x) & 0x1) << 8)

#define DIV_STAT_CPU1_CHANGING	(DIV_COPY(DIV_STAT_CHANGING) | \
				DIV_HPM(DIV_STAT_CHANGING) | \
				DIV_CORES(DIV_STAT_CHANGING))

/* CLK_SRC_DMC */
#define MUX_C2C_SEL(x)      ((x) & 0x1)
#define MUX_DMC_BUS_SEL(x)  (((x) & 0x1) << 4)
#define MUX_DPHY_SEL(x)     (((x) & 0x1) << 8)
#define MUX_MPLL_SEL(x)     (((x) & 0x1) << 12)
#define MUX_PWI_SEL(x)      (((x) & 0xf) << 16)
#define MUX_G2D_ACP0_SEL(x) (((x) & 0x1) << 20)
#define MUX_G2D_ACP1_SEL(x) (((x) & 0x1) << 24)
#define MUX_G2D_ACP_SEL(x)  (((x) & 0x1) << 28)

/* CLK_MUX_STAT_DMC */
#define C2C_SEL(x)          ((x) & 0x7)
#define DMC_BUS_SEL(x)      (((x) & 0x7) << 4)
#define DPHY_SEL(x)         (((x) & 0x7) << 8)
#define MPLL_SEL(x)         (((x) & 0x7) << 12)
#define G2D_ACP0_SEL(x)     (((x) & 0x7) << 20)
#define G2D_ACP1_SEL(x)     (((x) & 0x7) << 24)
#define G2D_ACP_SEL(x)      (((x) & 0x7) << 28)

#define MUX_STAT_DMC_CHANGING   (C2C_SEL(MUX_STAT_CHANGING) | \
				DMC_BUS_SEL(MUX_STAT_CHANGING) | \
				DPHY_SEL(MUX_STAT_CHANGING) | \
				MPLL_SEL(MUX_STAT_CHANGING) |\
				G2D_ACP0_SEL(MUX_STAT_CHANGING) | \
				G2D_ACP1_SEL(MUX_STAT_CHANGING) | \
				G2D_ACP_SEL(MUX_STAT_CHANGING))

/* CLK_DIV_DMC0 */
#define ACP_RATIO(x)        ((x) & 0x7)
#define ACP_PCLK_RATIO(x)   (((x) & 0x7) << 4)
#define DPHY_RATIO(x)       (((x) & 0x7) << 8)
#define DMC_RATIO(x)        (((x) & 0x7) << 12)
#define DMCD_RATIO(x)       (((x) & 0x7) << 16)
#define DMCP_RATIO(x)       (((x) & 0x7) << 20)

/* CLK_DIV_DMC1 */
#define G2D_ACP_RATIO(x)    ((x) & 0xf)
#define C2C_RATIO(x)        (((x) & 0x7) << 4)
#define PWI_RATIO(x)        (((x) & 0xf) << 8)
#define C2C_ACLK_RATIO(x)   (((x) & 0x7) << 12)
#define DVSEM_RATIO(x)      (((x) & 0x7f) << 16)
#define DPM_RATIO(x)        (((x) & 0x7f) << 24)

/* CLK_DIV_STAT_DMC0 */
#define DIV_ACP(x)          ((x) & 0x1)
#define DIV_ACP_PCLK(x)     (((x) & 0x1) << 4)
#define DIV_DPHY(x)         (((x) & 0x1) << 8)
#define DIV_DMC(x)          (((x) & 0x1) << 12)
#define DIV_DMCD(x)         (((x) & 0x1) << 16)
#define DIV_DMCP(x)         (((x) & 0x1) << 20)

#define DIV_STAT_DMC0_CHANGING  (DIV_ACP(DIV_STAT_CHANGING) | \
				DIV_ACP_PCLK(DIV_STAT_CHANGING) | \
				DIV_DPHY(DIV_STAT_CHANGING) | \
				DIV_DMC(DIV_STAT_CHANGING) | \
				DIV_DMCD(DIV_STAT_CHANGING) | \
				DIV_DMCP(DIV_STAT_CHANGING))

/* CLK_DIV_STAT_DMC1 */
#define DIV_G2D_ACP(x)       ((x) & 0x1)
#define DIV_C2C(x)           (((x) & 0x1) << 4)
#define DIV_PWI(x)           (((x) & 0x1) << 8)
#define DIV_C2C_ACLK(x)      (((x) & 0x1) << 12)
#define DIV_DVSEM(x)         (((x) & 0x1) << 16)
#define DIV_DPM(x)           (((x) & 0x1) << 24)

#define DIV_STAT_DMC1_CHANGING  (DIV_G2D_ACP(DIV_STAT_CHANGING) | \
				DIV_C2C(DIV_STAT_CHANGING) | \
				DIV_PWI(DIV_STAT_CHANGING) | \
				DIV_C2C_ACLK(DIV_STAT_CHANGING) | \
				DIV_DVSEM(DIV_STAT_CHANGING) | \
				DIV_DPM(DIV_STAT_CHANGING))

/* CLK_SRC_TOP0 */
#define MUX_ONENAND_1_SEL(x)    ((x) & 0x1)
#define MUX_EPLL_SEL(x)         (((x) & 0x1) << 4)
#define MUX_VPLL_SEL(x)         (((x) & 0x1) << 8)
#define MUX_ACLK_200_SEL(x)     (((x) & 0x1) << 12)
#define MUX_ACLK_100_SEL(x)     (((x) & 0x1) << 16)
#define MUX_ACLK_160_SEL(x)     (((x) & 0x1) << 20)
#define MUX_ACLK_133_SEL(x)     (((x) & 0x1) << 24)
#define MUX_ONENAND_SEL(x)      (((x) & 0x1) << 28)

/* CLK_MUX_STAT_TOP */
#define ONENAND_1_SEL(x)    ((x) & 0x3)
#define EPLL_SEL(x)         (((x) & 0x3) << 4)
#define VPLL_SEL(x)         (((x) & 0x3) << 8)
#define ACLK_200_SEL(x)     (((x) & 0x3) << 12)
#define ACLK_100_SEL(x)     (((x) & 0x3) << 16)
#define ACLK_160_SEL(x)     (((x) & 0x3) << 20)
#define ACLK_133_SEL(x)     (((x) & 0x3) << 24)
#define ONENAND_SEL(x)      (((x) & 0x3) << 28)

#define MUX_STAT_TOP0_CHANGING  (ONENAND_1_SEL(MUX_STAT_CHANGING) | \
				EPLL_SEL(MUX_STAT_CHANGING) | \
				EPLL_SEL(MUX_STAT_CHANGING) | \
				VPLL_SEL(MUX_STAT_CHANGING) | \
				ACLK_200_SEL(MUX_STAT_CHANGING) | \
				ACLK_100_SEL(MUX_STAT_CHANGING) | \
				ACLK_160_SEL(MUX_STAT_CHANGING) | \
				ACLK_133_SEL(MUX_STAT_CHANGING) | \
				ONENAND_SEL(MUX_STAT_CHANGING))

/* CLK_SRC_TOP1 */
#define MUX_ACLK_266_GPS_SEL(x)        (((x) & 0x1) << 4)
#define MUX_ACLK_400_MCUISP_SEL(x)     (((x) & 0x1) << 8)
#define MUX_MPLL_USER_SEL_T(x)         (((x) & 0x1) << 12)
#define MUX_ACLK_266_GPS_SUB_SEL(x)    (((x) & 0x1) << 16)
#define MUX_ACLK_200_SUB_SEL(x)        (((x) & 0x1) << 20)
#define MUX_ACLK_400_MCUISP_SUB_SEL(x) (((x) & 0x1) << 24)

/* CLK_MUX_STAT_TOP1 */
#define ACLK_266_GPS_SEL(x)        (((x) & 0x3) << 4)
#define ACLK_400_MCUISP_SEL(x)     (((x) & 0x3) << 8)
#define MPLL_USER_SEL_T(x)         (((x) & 0x3) << 12)
#define ACLK_266_GPS_SUB_SEL(x)    (((x) & 0x3) << 16)
#define ACLK_200_SUB_SEL(x)        (((x) & 0x3) << 20)
#define ACLK_400_MCUISP_SUB_SEL(x) (((x) & 0x3) << 24)

#define MUX_STAT_TOP1_CHANGING  (MUX_ACLK_266_GPS_SEL(MUX_STAT_CHANGING) | \
				ACLK_400_MCUISP_SEL(MUX_STAT_CHANGING) | \
				MPLL_USER_SEL_T(MUX_STAT_CHANGING) | \
				ACLK_266_GPS_SUB_SEL(MUX_STAT_CHANGING) | \
				ACLK_200_SUB_SEL(MUX_STAT_CHANGING) | \
				ACLK_400_MCUISP_SUB_SEL(MUX_STAT_CHANGING))

/* CLK_DIV_TOP */
#define ACLK_200_RATIO(x)        ((x) & 0x7)
#define ACLK_100_RATIO(x)        (((x) & 0xf) << 4)
#define ACLK_160_RATIO(x)        (((x) & 0x7) << 8)
#define	ACLK_133_RATIO(x)        (((x) & 0x7) << 12)
#define	ONENAND_RATIO(x)         (((x) & 0x7) << 16)
#define	ACLK_266_GPS_RATIO(x)    (((x) & 0x7) << 20)
#define	ACLK_400_MCUISP_RATIO(x) (((x) & 0x7) << 24)

#define DIV_STAT_TOP_CHANGING    (ACLK_400_MCUISP_RATIO(DIV_STAT_CHANGING) | \
				ACLK_266_GPS_RATIO(DIV_STAT_CHANGING) | \
				ONENAND_RATIO(DIV_STAT_CHANGING) | \
				ACLK_133_RATIO(DIV_STAT_CHANGING) | \
				ACLK_160_RATIO(DIV_STAT_CHANGING) | \
				ACLK_100_RATIO(DIV_STAT_CHANGING) | \
				ACLK_200_RATIO(DIV_STAT_CHANGING))

/* CLK_SRC_LEFTBUS */
#define	MUX_GDL_SEL(x)         ((x) & 0x1)
#define	MUX_MPLL_USER_SEL_L(x) (((x) & 0x1) << 4)

/* CLK_MUX_STAT_LEFTBUS */
#define	GDL_SEL(x)             ((x) & 0x7)
#define	MPLL_USER_SEL_L(x)     (((x) & 0x7) << 4)

#define MUX_STAT_LEFTBUS_CHANGING    (GDL_SEL(MUX_STAT_CHANGING) | \
				MPLL_USER_SEL_L(MUX_STAT_CHANGING))

/* CLK_DIV_LEFTBUS */
#define GDL_RATIO(x)        ((x) & 0x7)
#define GPL_RATIO(x)        (((x) & 0x7) << 4)

/* CLK_DIV_STAT_LEFTBUS */
#define DIV_GDL(x)          ((x) & 0x1)
#define DIV_GPL(x)          (((x) & 0x1) << 4)

#define DIV_STAT_LEFTBUS_CHANGING    (DIV_GDL(DIV_STAT_CHANGING) | \
				DIV_GPL(DIV_STAT_CHANGING))

/* CLK_SRC_RIGHTBUS */
#define MUX_GDR_SEL(x)            ((x) & 0x1)
#define MUX_MPLL_USER_SEL_R(x)    (((x) & 0x1) << 4)

/* CLK_MUX_STAT_RIGHTBUS */
#define GDR_SEL(x)                ((x) & 0x7)
#define MPLL_USER_SEL_R(x)        (((x) & 0x7) << 4)

#define MUX_STAT_RIGHTBUS_CHANGING    (GDR_SEL(MUX_STAT_CHANGING) | \
				MPLL_USER_SEL_R(MUX_STAT_CHANGING))

/* CLK_DIV_RIGHTBUS */
#define GPR_RATIO(x)         ((x) & 0x7)
#define GDR_RATIO(x)         (((x) & 0x7) << 4)

/* CLK_DIV_STAT_RIGHTBUS */
#define DIV_GDR(x)           ((x) & 0x1)
#define DIV_GPR(x)           ((x) & 0x1)

#define DIV_STAT_RIGHTBUS_CHANGING    (DIV_GDR(DIV_STAT_CHANGING) | \
				DIV_GPR(DIV_STAT_CHANGING))

/* CLK_SRC_PERIL0 */
#define UART0_SEL(x)        ((x) & 0xf)
#define UART1_SEL(x)        (((x) & 0xf) << 4)
#define UART2_SEL(x)        (((x) & 0xf) << 8)
#define UART3_SEL(x)        (((x) & 0xf) << 12)
#define UART4_SEL(x)        (((x) & 0xf) << 16)

/* CLK_DIV_PERIL0 */
#define UART0_RATIO(x)      ((x) & 0xf)
#define UART1_RATIO(x)      (((x) & 0xf) << 4)
#define UART2_RATIO(x)      (((x) & 0xf) << 8)
#define UART3_RATIO(x)      (((x) & 0xf) << 12)
#define UART4_RATIO(x)      (((x) & 0xf) << 16)

/* CLK_DIV_STAT_PERIL0 */
#define DIV_UART0(x)        ((x) & 0x1)
#define DIV_UART1(x)        (((x) & 0x1) << 4)
#define DIV_UART2(x)        (((x) & 0x1) << 8)
#define DIV_UART3(x)        (((x) & 0x1) << 12)
#define DIV_UART4(x)        (((x) & 0x1) << 16)

#define DIV_STAT_PERIL0_CHANGING    (DIV_UART4(DIV_STAT_CHANGING) | \
				DIV_UART3(DIV_STAT_CHANGING) | \
				DIV_UART2(DIV_STAT_CHANGING) | \
				DIV_UART1(DIV_STAT_CHANGING) | \
				DIV_UART0(DIV_STAT_CHANGING))

/* CLK_SRC_FSYS */
#define MMC1_SEL(x)         (((x) & 0xf) << 4)
#define MMC2_SEL(x)         (((x) & 0xf) << 8)
#define MMC3_SEL(x)         (((x) & 0xf) << 12)
#define MMC4_SEL(x)         (((x) & 0xf) << 16)
#define MIPIHSI_SEL(x)      (((x) & 0x1) << 24)

/* CLK_DIV_FSYS0 */
#define MIPIHSI_RATIO(x)    (((x) & 0xf) << 20)

/* CLK_DIV_STAT_FSYS0 */
#define DIV_MIPIHSI(x)    (((x) & 0x1) << 20)

#define DIV_STAT_FSYS0_CHANGING    (DIV_MIPIHSI(DIV_STAT_CHANGING))

/* CLK_DIV_FSYS1 */
#define MMC0_RATIO(x)       ((x) & 0xf)
#define MMC0_PRE_RATIO(x)   (((x) & 0xff) << 8)
#define MMC1_RATIO(x)       (((x) & 0xf) << 16)
#define MMC1_PRE_RATIO(x)   (((x) & 0xff) << 24)

/* CLK_DIV_STAT_FSYS1 */
#define DIV_MMC0(x)         ((x) & 1)
#define DIV_MMC0_PRE(x)     (((x) & 1) << 8)
#define DIV_MMC1(x)         (((x) & 1) << 16)
#define DIV_MMC1_PRE(x)     (((x) & 1) << 24)

#define DIV_STAT_FSYS1_CHANGING    (DIV_MMC0(DIV_STAT_CHANGING) | \
				DIV_MMC0_PRE(DIV_STAT_CHANGING) | \
				DIV_MMC1(DIV_STAT_CHANGING) | \
				DIV_MMC1_PRE(DIV_STAT_CHANGING))

/* CLK_DIV_FSYS2 */
#define MMC2_RATIO(x)       ((x) & 0xf)
#define MMC2_PRE_RATIO(x)   (((x) & 0xff) << 8)
#define MMC3_RATIO(x)       (((x) & 0xf) << 16)
#define MMC3_PRE_RATIO(x)   (((x) & 0xff) << 24)

/* CLK_DIV_STAT_FSYS2 */
#define DIV_MMC2(x)         ((x) & 0x1)
#define DIV_MMC2_PRE(x)     (((x) & 0x1) << 8)
#define DIV_MMC3(x)         (((x) & 0x1) << 16)
#define DIV_MMC3_PRE(x)     (((x) & 0x1) << 24)

#define DIV_STAT_FSYS2_CHANGING    (DIV_MMC2(DIV_STAT_CHANGING) | \
				DIV_MMC2_PRE(DIV_STAT_CHANGING) | \
				DIV_MMC3(DIV_STAT_CHANGING) | \
				DIV_MMC3_PRE(DIV_STAT_CHANGING))

/* CLK_DIV_FSYS3 */
#define MMC4_RATIO(x)       ((x) & 0x7)
#define MMC4_PRE_RATIO(x)   (((x) & 0xff) << 8)

/* CLK_DIV_STAT_FSYS3 */
#define DIV_MMC4(x)         ((x) & 0x1)
#define DIV_MMC4_PRE(x)     (((x) & 0x1) << 8)

#define DIV_STAT_FSYS3_CHANGING    (DIV_MMC4(DIV_STAT_CHANGING) | \
				DIV_MMC4_PRE(DIV_STAT_CHANGING))

/* DMC */
#ifdef CONFIG_CLK_800_330_165
#define DRAM_CLK_330
#endif
#ifdef CONFIG_CLK_1000_200_200
#define DRAM_CLK_200
#endif
#ifdef CONFIG_CLK_1000_330_165
#define DRAM_CLK_330
#endif
#ifdef CONFIG_CLK_1000_400_200
#define DRAM_CLK_400
#endif

/* Bus Configuration Register Address */
#define ASYNC_CONFIG		0x10010350

#define DIRECT_CMD_NOP	0x07000000
#define DIRECT_CMD_ZQ	0x0a000000
#define DIRECT_CMD_CHIP1_SHIFT	(1 << 20)
#define MEM_TIMINGS_MSR_COUNT	4
#define CTRL_START	(1 << 0)
#define CTRL_DLL_ON	(1 << 1)
#define AREF_EN		(1 << 5)
#define DRV_TYPE	(1 << 6)

struct mem_timings {
	unsigned direct_cmd_msr[MEM_TIMINGS_MSR_COUNT];
	unsigned timingref;
	unsigned timingrow;
	unsigned timingdata;
	unsigned timingpower;
	unsigned zqcontrol;
	unsigned control0;
	unsigned control1;
	unsigned control2;
	unsigned concontrol;
	unsigned prechconfig;
	unsigned memcontrol;
	unsigned memconfig0;
	unsigned memconfig1;
	unsigned dll_resync;
	unsigned dll_on;
};

/* MIU */
/* MIU Config Register Offsets*/
#define APB_SFR_INTERLEAVE_CONF_OFFSET	0x400
#define APB_SFR_ARBRITATION_CONF_OFFSET	0xC00
#define ABP_SFR_SLV_ADDRMAP_CONF_OFFSET	0x800
#define ABP_SFR_INTERLEAVE_ADDRMAP_START_OFFSET	0x808
#define ABP_SFR_INTERLEAVE_ADDRMAP_END_OFFSET	0x810
#define ABP_SFR_SLV0_SINGLE_ADDRMAP_START_OFFSET	0x818
#define ABP_SFR_SLV0_SINGLE_ADDRMAP_END_OFFSET	0x820
#define ABP_SFR_SLV1_SINGLE_ADDRMAP_START_OFFSET	0x828
#define ABP_SFR_SLV1_SINGLE_ADDRMAP_END_OFFSET	0x830

#if (defined CONFIG_ORIGEN) || (defined CONFIG_ITOP4412)
/* Interleave: 2Bit, Interleave_bit1: 0x15, Interleave_bit0: 0x7 */
#define APB_SFR_INTERLEAVE_CONF_VAL	0x20001507
#define APB_SFR_ARBRITATION_CONF_VAL	0x00000001
#endif

#define INTERLEAVE_ADDR_MAP_START_ADDR	0x40000000
#define INTERLEAVE_ADDR_MAP_END_ADDR	0xbfffffff
#define INTERLEAVE_ADDR_MAP_EN		0x00000001

#ifdef CONFIG_MIU_1BIT_INTERLEAVED
/* Interleave_bit0: 0xC*/
#define APB_SFR_INTERLEAVE_CONF_VAL	0x0000000c
#endif
#ifdef CONFIG_MIU_2BIT_INTERLEAVED
/* Interleave: 2Bit, Interleave_bit1: 0x15, Interleave_bit0: 0xc */
#define APB_SFR_INTERLEAVE_CONF_VAL	0x2000150c
#endif
#define SLAVE0_SINGLE_ADDR_MAP_START_ADDR	0x40000000
#define SLAVE0_SINGLE_ADDR_MAP_END_ADDR		0x7fffffff
#define SLAVE1_SINGLE_ADDR_MAP_START_ADDR	0x80000000
#define SLAVE1_SINGLE_ADDR_MAP_END_ADDR		0xbfffffff
/* Enable SME0 and SME1*/
#define APB_SFR_SLV_ADDR_MAP_CONF_VAL		0x00000006

#define FORCE_DLL_RESYNC	3
#define DLL_CONTROL_ON		1

#define DIRECT_CMD1	0x00020000
#define DIRECT_CMD2	0x00030000
#define DIRECT_CMD3	0x00010002
#define DIRECT_CMD4	0x00000328

#define CTRL_ZQ_MODE_NOTERM	(0x1 << 0)
#define CTRL_ZQ_START		(0x1 << 1)
#define CTRL_ZQ_DIV		(0 << 4)
#define CTRL_ZQ_MODE_DDS	(0x7 << 8)
#define CTRL_ZQ_MODE_TERM	(0x2 << 11)
#define CTRL_ZQ_FORCE_IMPN	(0x5 << 14)
#define CTRL_ZQ_FORCE_IMPP	(0x6 << 17)
#define CTRL_DCC		(0xE38 << 20)
#define ZQ_CONTROL_VAL		(CTRL_ZQ_MODE_NOTERM | CTRL_ZQ_START\
				| CTRL_ZQ_DIV | CTRL_ZQ_MODE_DDS\
				| CTRL_ZQ_MODE_TERM | CTRL_ZQ_FORCE_IMPN\
				| CTRL_ZQ_FORCE_IMPP | CTRL_DCC)

#define ASYNC			(0 << 0)
#define CLK_RATIO		(1 << 1)
#define DIV_PIPE		(1 << 3)
#define AWR_ON			(1 << 4)
#define AREF_DISABLE		(0 << 5)
#define DRV_TYPE_DISABLE	(0 << 6)
#define CHIP0_NOT_EMPTY		(0 << 8)
#define CHIP1_NOT_EMPTY		(0 << 9)
#define DQ_SWAP_DISABLE		(0 << 10)
#define QOS_FAST_DISABLE	(0 << 11)
#define RD_FETCH		(0x3 << 12)
#define TIMEOUT_LEVEL0		(0xFFF << 16)
#define CONCONTROL_VAL		(ASYNC | CLK_RATIO | DIV_PIPE | AWR_ON\
				| AREF_DISABLE | DRV_TYPE_DISABLE\
				| CHIP0_NOT_EMPTY | CHIP1_NOT_EMPTY\
				| DQ_SWAP_DISABLE | QOS_FAST_DISABLE\
				| RD_FETCH | TIMEOUT_LEVEL0)

#define CLK_STOP_DISABLE	(0 << 1)
#define DPWRDN_DISABLE		(0 << 2)
#define DPWRDN_TYPE		(0 << 3)
#define TP_DISABLE		(0 << 4)
#define DSREF_DIABLE		(0 << 5)
#define ADD_LAT_PALL		(1 << 6)
#define MEM_TYPE_DDR3		(0x6 << 8)
#define MEM_WIDTH_32		(0x2 << 12)
#define NUM_CHIP_2		(0 << 16)
#define BL_8			(0x3 << 20)
#define MEMCONTROL_VAL		(CLK_STOP_DISABLE | DPWRDN_DISABLE\
				| DPWRDN_TYPE | TP_DISABLE | DSREF_DIABLE\
				| ADD_LAT_PALL | MEM_TYPE_DDR3 | MEM_WIDTH_32\
				| NUM_CHIP_2 | BL_8)


#define CHIP_BANK_8		(0x3 << 0)
#define CHIP_ROW_14		(0x3 << 4)
#define CHIP_COL_10		(0x3 << 8)
#define CHIP_MAP_INTERLEAVED	(1 << 12)
#define CHIP_MASK		(0xC0 << 16)
#ifdef CONFIG_MIU_LINEAR
#define CHIP0_BASE		(0x40 << 24)
#define CHIP1_BASE		(0x60 << 24)
#else
#define CHIP0_BASE		(0x40 << 24)
#define CHIP1_BASE		(0x80 << 24)
#endif
#define MEMCONFIG0_VAL		(CHIP_BANK_8 | CHIP_ROW_14 | CHIP_COL_10\
				| CHIP_MAP_INTERLEAVED | CHIP_MASK | CHIP0_BASE)
#define MEMCONFIG1_VAL		(CHIP_BANK_8 | CHIP_ROW_14 | CHIP_COL_10\
				| CHIP_MAP_INTERLEAVED | CHIP_MASK | CHIP1_BASE)

#define TP_CNT			(0xff << 24)
#define PRECHCONFIG		TP_CNT

#define CTRL_OFF		(0 << 0)
#define CTRL_DLL_OFF		(0 << 1)
#define CTRL_HALF		(0 << 2)
#define CTRL_DFDQS		(1 << 3)
#define DQS_DELAY		(0 << 4)
#define CTRL_START_POINT	(0x10 << 8)
#define CTRL_INC		(0x10 << 16)
#define CTRL_FORCE		(0x71 << 24)
#define CONTROL0_VAL		(CTRL_OFF | CTRL_DLL_OFF | CTRL_HALF\
				| CTRL_DFDQS | DQS_DELAY | CTRL_START_POINT\
				| CTRL_INC | CTRL_FORCE)

#define CTRL_SHIFTC		(0x6 << 0)
#define CTRL_REF		(8 << 4)
#define CTRL_SHGATE		(1 << 29)
#define TERM_READ_EN		(1 << 30)
#define TERM_WRITE_EN		(1 << 31)
#define CONTROL1_VAL		(CTRL_SHIFTC | CTRL_REF | CTRL_SHGATE\
				| TERM_READ_EN | TERM_WRITE_EN)

#define CONTROL2_VAL		0x00000000

#ifdef CONFIG_ITOP4412
#define TIMINGREF_VAL		0x000000BB
#define TIMINGROW_VAL		0x4046654f
#define	TIMINGDATA_VAL		0x46400506
#define	TIMINGPOWER_VAL		0x52000A3C
#else
#define TIMINGREF_VAL		0x000000BC
#ifdef DRAM_CLK_330
#define TIMINGROW_VAL		0x3545548d
#define	TIMINGDATA_VAL		0x45430506
#define	TIMINGPOWER_VAL		0x4439033c
#endif
#ifdef DRAM_CLK_400
#define TIMINGROW_VAL		0x45430506
#define	TIMINGDATA_VAL		0x56500506
#define	TIMINGPOWER_VAL		0x5444033d
#endif
#endif

#ifdef CONFIG_BOARD_TYPES
extern void sdelay(unsigned long);
#endif


#endif

