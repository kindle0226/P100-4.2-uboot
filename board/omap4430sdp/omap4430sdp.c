/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
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
#include <linux/mtd/compat.h>
#include <common.h>
#include <asm/arch/mux.h>
#include <asm/io.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include "logo.h"
#include "battery_0.h"
#include "low_battery_0.h"
#include "C1_Fastboot-1.h"
#include "C1_Fastboot-2.h"
#include "C1_Fastboot-3.h"
#include "character_img/0.h"
#include "character_img/1.h"
#include "character_img/2.h"
#include "character_img/3.h"
#include "character_img/4.h"
#include "character_img/5.h"
#include "character_img/6.h"
#include "character_img/7.h"
#include "character_img/8.h"
#include "character_img/9.h"
#include "character_img/p.h"
#include "chrg_prompt.h"
/* function protypes */
extern int omap4_mmc_init(void);
extern int select_bus(int, int);
#if defined(OMAP44XX_TABLET_CONFIG)
static int tablet_check_display_boardid(void);
#endif

#define		OMAP44XX_WKUP_CTRL_BASE		0x4A31E000
#define		OMAP44XX_CONTROL_DSIPHY		0x4A100618
#define		CONTROL_CONTROL_GPIOWK		0x0600

#define 	GPIO2_OE			0x48055134
#define 	GPIO2_DATAIN			0x48055138
#define 	GPIO5_OE			0x4805B134
#define 	GPIO5_CLEARDATAOUT		0x4805B190
#define 	GPIO5_SETDATAOUT		0x4805B194
#if 1
#define M0_SAFE M0
#define M1_SAFE M1
#define M2_SAFE M2
#define M4_SAFE M4
#define M7_SAFE M7
#define M3_SAFE M3
#define M5_SAFE M5
#define M6_SAFE M6
#else
#define M0_SAFE M7
#define M1_SAFE M7
#define M2_SAFE M7
#define M4_SAFE M7
#define M7_SAFE M7
#define M3_SAFE M7
#define M5_SAFE M7
#define M6_SAFE M7
#endif
#define		MV(OFFSET, VALUE)\
			__raw_writew((VALUE), OMAP44XX_CTRL_BASE + (OFFSET));
#define		MV1(OFFSET, VALUE)\
			__raw_writew((VALUE), OMAP44XX_WKUP_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)
#define		WK(x)	(CONTROL_WKUP_##x)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#ifdef CONFIG_LEDS_INDICATOR
#define RED_LED			155
#define BLUE_LED		153
#define GREEN_LED		152
#define GPIO5_OE		0x4805B134
#define GPIO5_DATAOUT		0x4805B13C
#define GPIO5_CLEARDATAOUT	0x4805B190
#define GPIO5_SETDATAOUT	0x4805B194
#endif

enum omap_dss_load_mode {
	OMAP_DSS_LOAD_CLUT_AND_FRAME	= 0,
	OMAP_DSS_LOAD_CLUT_ONLY		= 1,
	OMAP_DSS_LOAD_FRAME_ONLY	= 2,
	OMAP_DSS_LOAD_CLUT_ONCE_FRAME	= 3,
};
enum fifo_size {
	DSI_FIFO_SIZE_0		= 0,
	DSI_FIFO_SIZE_32	= 1,
	DSI_FIFO_SIZE_64	= 2,
	DSI_FIFO_SIZE_96	= 3,
	DSI_FIFO_SIZE_128	= 4,
};
enum dsi_pll_power_state {
	DSI_PLL_POWER_OFF	= 0x0,
	DSI_PLL_POWER_ON_HSCLK	= 0x1,
	DSI_PLL_POWER_ON_ALL	= 0x2,
	DSI_PLL_POWER_ON_DIV	= 0x3,
};
struct dsi_data {
	struct {
		enum fifo_size fifo_size;
		int vc_id;
	} vc[4];

	unsigned num_lanes_supported;
	int num_line_buffers;
};
struct dsi_data dsi;

static u8 display_initialized = 0;
#ifdef CONFIG_KEEP_DISPLAY_ALIVE
static u8 ql_is_on = 0;
#endif
static u8 bl_is_on = 0;
/* MIPI DSI Processor-to-Peripheral transaction types */
enum {
	MIPI_DSI_V_SYNC_START				= 0x01,
	MIPI_DSI_V_SYNC_END				= 0x11,
	MIPI_DSI_H_SYNC_START				= 0x21,
	MIPI_DSI_H_SYNC_END				= 0x31,

	MIPI_DSI_COLOR_MODE_OFF				= 0x02,
	MIPI_DSI_COLOR_MODE_ON				= 0x12,
	MIPI_DSI_SHUTDOWN_PERIPHERAL			= 0x22,
	MIPI_DSI_TURN_ON_PERIPHERAL			= 0x32,

	MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM		= 0x03,
	MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM		= 0x13,
	MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM		= 0x23,

	MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM		= 0x04,
	MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM		= 0x14,
	MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM		= 0x24,

	MIPI_DSI_DCS_SHORT_WRITE			= 0x05,
	MIPI_DSI_DCS_SHORT_WRITE_PARAM			= 0x15,

	MIPI_DSI_DCS_READ				= 0x06,

	MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE		= 0x37,

	MIPI_DSI_END_OF_TRANSMISSION			= 0x08,

	MIPI_DSI_NULL_PACKET				= 0x09,
	MIPI_DSI_BLANKING_PACKET			= 0x19,
	MIPI_DSI_GENERIC_LONG_WRITE			= 0x29,
	MIPI_DSI_DCS_LONG_WRITE				= 0x39,

	MIPI_DSI_LOOSELY_PACKED_PIXEL_STREAM_YCBCR20	= 0x0c,
	MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR24		= 0x1c,
	MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR16		= 0x2c,

	MIPI_DSI_PACKED_PIXEL_STREAM_30			= 0x0d,
	MIPI_DSI_PACKED_PIXEL_STREAM_36			= 0x1d,
	MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR12		= 0x3d,

	MIPI_DSI_PACKED_PIXEL_STREAM_16			= 0x0e,
	MIPI_DSI_PACKED_PIXEL_STREAM_18			= 0x1e,
	MIPI_DSI_PIXEL_STREAM_3BYTE_18			= 0x2e,
	MIPI_DSI_PACKED_PIXEL_STREAM_24			= 0x3e,
};

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */

#define MUX_DEFAULT_OMAP4() \
	MV(CP(GPMC_AD0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M3)) /* gpio_32 */ \
	MV(CP(GPMC_AD13),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_37 */ \
	MV(CP(GPMC_AD14),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_38 */ \
	MV(CP(GPMC_AD15),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_39 */ \
	MV(CP(GPMC_A18),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row6 */ \
	MV(CP(GPMC_A19),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row7 */ \
	MV(CP(GPMC_A22),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col6 */ \
	MV(CP(GPMC_A23),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col7 */ \
	MV(CP(GPMC_NOE),	(PTU | OFF_EN | OFF_OUT_PTD | M1)) /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_cmd */ \
	MV(CP(CSI21_DX4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_75 */ \
	MV(CP(CSI21_DY4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_76 */ \
	MV(CP(CAM_SHUTTER),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_shutter */ \
	MV(CP(CAM_STROBE),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_strobe */ \
	MV(CP(CAM_GLOBALRESET),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_83 */ \
	MV(CP(USBB1_HSIC_DATA),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_strobe */ \
	MV(CP(SDMMC1_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat4 */ \
	MV(CP(SDMMC1_DAT5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat5 */ \
	MV(CP(SDMMC1_DAT6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat6 */ \
	MV(CP(SDMMC1_DAT7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat7 */ \
	MV(CP(ABE_MCBSP2_CLKX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_clkx */ \
	MV(CP(ABE_MCBSP2_DR),	(IEN | OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dr */ \
	MV(CP(ABE_MCBSP2_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dx */ \
	MV(CP(ABE_MCBSP2_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_fsx */ \
	MV(CP(ABE_MCBSP1_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp1_dx */ \
	MV(CP(ABE_MCBSP1_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp1_fsx */ \
	MV(CP(UART2_CTS),	(PTU | IEN | M0)) /* uart2_cts */ \
	MV(CP(UART2_RTS),	(M0)) /* uart2_rts */ \
	MV(CP(UART2_RX),	(IEN | M0)) /* uart2_rx */ \
	MV(CP(UART2_TX),	(M0)) /* uart2_tx */ \
	MV(CP(I2C1_SCL),	(PTU | IEN | M0)) /* i2c1_scl */ \
	MV(CP(I2C1_SDA),	(PTU | IEN | M0)) /* i2c1_sda */ \
	MV(CP(I2C2_SCL),	(PTU | IEN | M0)) /* i2c2_scl */ \
	MV(CP(I2C2_SDA),	(PTU | IEN | M0)) /* i2c2_sda */ \
	MV(CP(I2C3_SCL),	(PTU | IEN | M0)) /* i2c3_scl */ \
	MV(CP(I2C3_SDA),	(PTU | IEN | M0)) /* i2c3_sda */ \
	MV(CP(I2C4_SCL),	(PTU | IEN | M0)) /* i2c4_scl */ \
	MV(CP(I2C4_SDA),	(PTU | IEN | M0)) /* i2c4_sda */ \
	MV(CP(MCSPI1_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_clk */ \
	MV(CP(MCSPI1_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_somi */ \
	MV(CP(MCSPI1_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_simo */ \
	MV(CP(MCSPI1_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_cs0 */ \
	MV(CP(MCSPI1_CS2),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_139 */ \
	MV(CP(UART3_CTS_RCTX),	(PTU | IEN | M0)) /* uart3_tx */ \
	MV(CP(UART3_RTS_SD),	(M0)) /* uart3_rts_sd */ \
	MV(CP(UART3_RX_IRRX),	(IEN | M0)) /* uart3_rx */ \
	MV(CP(UART3_TX_IRTX),	(M0)) /* uart3_tx */ \
	MV(CP(SDMMC5_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc5_clk */ \
	MV(CP(SDMMC5_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_cmd */ \
	MV(CP(SDMMC5_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat0 */ \
	MV(CP(SDMMC5_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat1 */ \
	MV(CP(SDMMC5_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat2 */ \
	MV(CP(SDMMC5_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat3 */ \
	MV(CP(MCSPI4_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_simo */ \
	MV(CP(MCSPI4_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_somi */ \
	MV(CP(MCSPI4_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_cs0 */ \
	MV(CP(UART4_RX),	(IEN | M0)) /* uart4_rx */ \
	MV(CP(UART4_TX),	(M0)) /* uart4_tx */ \
	MV(CP(USBB2_ULPITLL_CLK),	(PTD | IEN | M3)) /* gpio_157 */ \
	MV(CP(USBB2_ULPITLL_STP),	(M5)) /* dispc2_data23 */ \
	MV(CP(USBB2_ULPITLL_DIR),	(M5)) /* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT),	(M5)) /* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0),	(M5)) /* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1),	(M5)) /* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2),	(M5)) /* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3),	(M5)) /* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4),	(M5)) /* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5),	(M5)) /* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6),	(M5)) /* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7),	(M5)) /* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_169 */ \
	MV(CP(USBB2_HSIC_STROBE),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_170 */ \
	MV(CP(UNIPRO_TX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col0 */ \
	MV(CP(UNIPRO_TY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col1 */ \
	MV(CP(UNIPRO_TX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col2 */ \
	MV(CP(UNIPRO_TY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col3 */ \
	MV(CP(UNIPRO_TX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col4 */ \
	MV(CP(UNIPRO_TY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col5 */ \
	MV(CP(UNIPRO_RX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row0 */ \
	MV(CP(UNIPRO_RY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row1 */ \
	MV(CP(UNIPRO_RX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row2 */ \
	MV(CP(UNIPRO_RY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row3 */ \
	MV(CP(UNIPRO_RX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row4 */ \
	MV(CP(UNIPRO_RY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row5 */ \
	MV(CP(USBA0_OTG_CE),	(PTU | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* usba0_otg_ce */ \
	MV(CP(USBA0_OTG_DP),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dp */ \
	MV(CP(USBA0_OTG_DM),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dm */ \
	MV(CP(SYS_NIRQ1),	(PTU | IEN | M0)) /* sys_nirq1 */ \
	MV(CP(SYS_NIRQ2),	(PTU | IEN | M0)) /* sys_nirq2 */ \
	MV(CP(DPM_EMU3),	(M5)) /* dispc2_data10 */ \
	MV(CP(DPM_EMU4),	(M5)) /* dispc2_data9 */ \
	MV(CP(DPM_EMU5),	(M5)) /* dispc2_data16 */ \
	MV(CP(DPM_EMU6),	(M5)) /* dispc2_data17 */ \
	MV(CP(DPM_EMU7),	(M5)) /* dispc2_hsync */ \
	MV(CP(DPM_EMU8),	(M5)) /* dispc2_pclk */ \
	MV(CP(DPM_EMU9),	(M5)) /* dispc2_vsync */ \
	MV(CP(DPM_EMU10),	(M5)) /* dispc2_de */ \
	MV(CP(DPM_EMU11),	(M5)) /* dispc2_data8 */ \
	MV(CP(DPM_EMU12),	(M5)) /* dispc2_data7 */ \
	MV(CP(DPM_EMU13),	(M5)) /* dispc2_data6 */ \
	MV(CP(DPM_EMU14),	(M5)) /* dispc2_data5 */ \
	MV(CP(DPM_EMU15),	(M5)) /* dispc2_data4 */ \
	MV(CP(DPM_EMU16),	(M5)) /* dispc2_data3/dmtimer8_pwm_evt */ \
	MV(CP(DPM_EMU17),	(M5)) /* dispc2_data2 */ \
	MV(CP(DPM_EMU18),	(M5)) /* dispc2_data1 */ \
	MV(CP(DPM_EMU19),	(M5)) /* dispc2_data0 */ \
	MV1(WK(PAD1_SR_SCL),	(PTU | IEN | M0)) /* sr_scl */ \
	MV1(WK(PAD0_SR_SDA),	(PTU | IEN | M0)) /* sr_sda */ \
	MV1(WK(PAD1_JTAG_TCK),	(IEN | M0)) /* jtag_tck */ \
	MV1(WK(PAD0_JTAG_RTCK),	(M0)) /* jtag_rtck */ \
	MV1(WK(PAD1_JTAG_TMS_TMSC),	(IEN | M0)) /* jtag_tms_tmsc */ \
	MV1(WK(PAD0_JTAG_TDI),	(IEN | M0)) /* jtag_tdi */ \
	MV1(WK(PAD1_JTAG_TDO),	(M0)) 		  /* jtag_tdo */ \
	MV(CP(MCSPI4_SIMO),	(PTD | M3)) /* GPIO_152, G_LED */ \
	MV(CP(MCSPI4_SOMI),	(PTD | M3)) /* GPIO 153, B_LED */ \
	MV(CP(UART4_RX),	(PTD | M3)) /* GPIO 155, R_LED */ \
	MV(CP(HDQ_SIO),		(M3)) /* GPIO_127, AUDPWRON */ 

#define MUX_DEFAULT_OMAP4_ALL() \
	MV(CP(GPMC_AD0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M3)) /* gpio_32 */ \
	MV(CP(GPMC_AD9),	(M3_SAFE)) /* gpio_33 */ \
	MV(CP(GPMC_AD10),	(M3_SAFE)) /* gpio_34 */ \
	MV(CP(GPMC_AD11),	(M3_SAFE)) /* gpio_35 */ \
	MV(CP(GPMC_AD12),	(M3_SAFE)) /* gpio_36 */ \
	MV(CP(GPMC_AD13),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_37 */ \
	MV(CP(GPMC_AD14),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_38 */ \
	MV(CP(GPMC_AD15),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_39 */ \
	MV(CP(GPMC_A16),	(M3_SAFE)) /* gpio_40 */ \
	MV(CP(GPMC_A17),	(M3_SAFE)) /* gpio_41 */ \
	MV(CP(GPMC_A18),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row6 */ \
	MV(CP(GPMC_A19),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row7 */ \
	MV(CP(GPMC_A20),	(M3_SAFE)) /* gpio_44 */ \
	MV(CP(GPMC_A21),	(M3_SAFE)) /* gpio_45 */ \
	MV(CP(GPMC_A22),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col6 */ \
	MV(CP(GPMC_A23),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col7 */ \
	MV(CP(GPMC_A24),	(M3_SAFE)) /* gpio_48 */ \
	MV(CP(GPMC_A25),	(M3_SAFE)) /* gpio_49 */ \
	MV(CP(GPMC_NCS0),	(M3_SAFE)) /* gpio_50 */ \
	MV(CP(GPMC_NCS1),	(M3_SAFE)) /* gpio_51 */ \
	MV(CP(GPMC_NCS2),	(M3_SAFE)) /* gpio_52 */ \
	MV(CP(GPMC_NCS3),	(M3_SAFE)) /* gpio_53 */ \
	MV(CP(GPMC_NWP),	(M3_SAFE)) /* gpio_54 */ \
	MV(CP(GPMC_CLK),	(M3_SAFE)) /* gpio_55 */ \
	MV(CP(GPMC_NADV_ALE),	(M3_SAFE)) /* gpio_56 */ \
	MV(CP(GPMC_NOE),	(PTU | OFF_EN | OFF_OUT_PTD | M1)) /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_cmd */ \
	MV(CP(GPMC_NBE0_CLE),	(M3_SAFE)) /* gpio_59 */ \
	MV(CP(GPMC_NBE1),	(M3_SAFE)) /* gpio_60 */ \
	MV(CP(GPMC_WAIT0),	(M3_SAFE)) /* gpio_61 */ \
	MV(CP(GPMC_WAIT1),	(M3_SAFE)) /* gpio_62 */ \
	MV(CP(C2C_DATA11),	(M4_SAFE)) /* gpio_100 */ \
	MV(CP(C2C_DATA12),	(M2_SAFE)) /* dsi1_te0 */ \
	MV(CP(C2C_DATA13),	(M4_SAFE)) /* gpio_102 */ \
	MV(CP(C2C_DATA14),	(M2_SAFE)) /* dsi2_te0 */ \
	MV(CP(C2C_DATA15),	(M4_SAFE)) /* gpio_104 */ \
	MV(CP(HDMI_HPD),	(M0_SAFE)) /* hdmi_hpd */ \
	MV(CP(HDMI_CEC),	(M0_SAFE)) /* hdmi_cec */ \
	MV(CP(HDMI_DDC_SCL),	(M0_SAFE)) /* hdmi_ddc_scl */ \
	MV(CP(HDMI_DDC_SDA),	(M0_SAFE)) /* hdmi_ddc_sda */ \
	MV(CP(CSI21_DX0),	(M0_SAFE)) /* csi21_dx0 */ \
	MV(CP(CSI21_DY0),	(M0_SAFE)) /* csi21_dy0 */ \
	MV(CP(CSI21_DX1),	(M0_SAFE)) /* csi21_dx1 */ \
	MV(CP(CSI21_DY1),	(M0_SAFE)) /* csi21_dy1 */ \
	MV(CP(CSI21_DX2),	(M0_SAFE)) /* csi21_dx2 */ \
	MV(CP(CSI21_DY2),	(M0_SAFE)) /* csi21_dy2 */ \
	MV(CP(CSI21_DX3),	(M7_SAFE)) /* csi21_dx3 */ \
	MV(CP(CSI21_DY3),	(M7_SAFE)) /* csi21_dy3 */ \
	MV(CP(CSI21_DX4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_75 */ \
	MV(CP(CSI21_DY4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_76 */ \
	MV(CP(CSI22_DX0),	(M0_SAFE)) /* csi22_dx0 */ \
	MV(CP(CSI22_DY0),	(M0_SAFE)) /* csi22_dy0 */ \
	MV(CP(CSI22_DX1),	(M0_SAFE)) /* csi22_dx1 */ \
	MV(CP(CSI22_DY1),	(M0_SAFE)) /* csi22_dy1 */ \
	MV(CP(CAM_SHUTTER),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_shutter */ \
	MV(CP(CAM_STROBE),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_strobe */ \
	MV(CP(CAM_GLOBALRESET),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_83 */ \
	MV(CP(USBB1_ULPITLL_CLK),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_clk */ \
	MV(CP(USBB1_ULPITLL_STP),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* usbb1_ulpiphy_stp */ \
	MV(CP(USBB1_ULPITLL_DIR),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dir */ \
	MV(CP(USBB1_ULPITLL_NXT),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_nxt */ \
	MV(CP(USBB1_ULPITLL_DAT0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat0 */ \
	MV(CP(USBB1_ULPITLL_DAT1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat1 */ \
	MV(CP(USBB1_ULPITLL_DAT2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat2 */ \
	MV(CP(USBB1_ULPITLL_DAT3),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat3 */ \
	MV(CP(USBB1_ULPITLL_DAT4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat4 */ \
	MV(CP(USBB1_ULPITLL_DAT5),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat5 */ \
	MV(CP(USBB1_ULPITLL_DAT6),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat6 */ \
	MV(CP(USBB1_ULPITLL_DAT7),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat7 */ \
	MV(CP(USBB1_HSIC_DATA),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_strobe */ \
	MV(CP(USBC1_ICUSB_DP),	(M0_SAFE)) /* usbc1_icusb_dp */ \
	MV(CP(USBC1_ICUSB_DM),	(M0_SAFE)) /* usbc1_icusb_dm */ \
	MV(CP(SDMMC1_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat4 */ \
	MV(CP(SDMMC1_DAT5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat5 */ \
	MV(CP(SDMMC1_DAT6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat6 */ \
	MV(CP(SDMMC1_DAT7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat7 */ \
	MV(CP(ABE_MCBSP2_CLKX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_clkx */ \
	MV(CP(ABE_MCBSP2_DR),	(IEN | OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dr */ \
	MV(CP(ABE_MCBSP2_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dx */ \
	MV(CP(ABE_MCBSP2_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_fsx */ \
	MV(CP(ABE_MCBSP1_CLKX),	(M1_SAFE)) /* abe_slimbus1_clock */ \
	MV(CP(ABE_MCBSP1_DR),	(M1_SAFE)) /* abe_slimbus1_data */ \
	MV(CP(ABE_MCBSP1_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp1_dx */ \
	MV(CP(ABE_MCBSP1_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp1_fsx */ \
	MV(CP(ABE_PDM_UL_DATA),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_ul_data */ \
	MV(CP(ABE_PDM_DL_DATA),	(PTD | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_dl_data */ \
	MV(CP(ABE_PDM_FRAME),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_frame */ \
	MV(CP(ABE_PDM_LB_CLK),	(OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_lb_clk */ \
	MV(CP(ABE_CLKS),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_clks */ \
	MV(CP(ABE_DMIC_CLK1),	(M0_SAFE)) /* abe_dmic_clk1 */ \
	MV(CP(ABE_DMIC_DIN1),	(M0_SAFE)) /* abe_dmic_din1 */ \
	MV(CP(ABE_DMIC_DIN2),	(M0_SAFE)) /* abe_dmic_din2 */ \
	MV(CP(ABE_DMIC_DIN3),	(M0_SAFE)) /* abe_dmic_din3 */ \
	MV(CP(UART2_CTS),	(PTU | IEN | M0)) /* uart2_cts */ \
	MV(CP(UART2_RTS),	(M0)) /* uart2_rts */ \
	MV(CP(UART2_RX),	(IEN | M0)) /* uart2_rx */ \
	MV(CP(UART2_TX),	(M0)) /* uart2_tx */ \
	MV(CP(HDQ_SIO),	(M3_SAFE)) /* gpio_127 */ \
	MV(CP(I2C1_SCL),	(PTU | IEN | M0)) /* i2c1_scl */ \
	MV(CP(I2C1_SDA),	(PTU | IEN | M0)) /* i2c1_sda */ \
	MV(CP(I2C2_SCL),	(PTU | IEN | M0)) /* i2c2_scl */ \
	MV(CP(I2C2_SDA),	(PTU | IEN | M0)) /* i2c2_sda */ \
	MV(CP(I2C3_SCL),	(PTU | IEN | M0)) /* i2c3_scl */ \
	MV(CP(I2C3_SDA),	(PTU | IEN | M0)) /* i2c3_sda */ \
	MV(CP(I2C4_SCL),	(PTU | IEN | M0)) /* i2c4_scl */ \
	MV(CP(I2C4_SDA),	(PTU | IEN | M0)) /* i2c4_sda */ \
	MV(CP(MCSPI1_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_clk */ \
	MV(CP(MCSPI1_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_somi */ \
	MV(CP(MCSPI1_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_simo */ \
	MV(CP(MCSPI1_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_cs0 */ \
	MV(CP(MCSPI1_CS1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* mcspi1_cs1 */ \
	MV(CP(MCSPI1_CS2),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_139 */ \
	MV(CP(MCSPI1_CS3),	(M3_SAFE)) /* gpio_140 */ \
	MV(CP(UART3_CTS_RCTX),	(PTU | IEN | M0)) /* uart3_tx */ \
	MV(CP(UART3_RTS_SD),	(M0)) /* uart3_rts_sd */ \
	MV(CP(UART3_RX_IRRX),	(IEN | M0)) /* uart3_rx */ \
	MV(CP(UART3_TX_IRTX),	(M0)) /* uart3_tx */ \
	MV(CP(SDMMC5_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc5_clk */ \
	MV(CP(SDMMC5_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_cmd */ \
	MV(CP(SDMMC5_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat0 */ \
	MV(CP(SDMMC5_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat1 */ \
	MV(CP(SDMMC5_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat2 */ \
	MV(CP(SDMMC5_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat3 */ \
	MV(CP(MCSPI4_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_simo */ \
	MV(CP(MCSPI4_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_somi */ \
	MV(CP(MCSPI4_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_cs0 */ \
	MV(CP(UART4_RX),	(IEN | M0)) /* uart4_rx */ \
	MV(CP(UART4_TX),	(M0)) /* uart4_tx */ \
	MV(CP(USBB2_ULPITLL_CLK),	(PTD | IEN | M3)) /* gpio_157 */ \
	MV(CP(USBB2_ULPITLL_STP),	(M5)) /* dispc2_data23 */ \
	MV(CP(USBB2_ULPITLL_DIR),	(M5)) /* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT),	(M5)) /* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0),	(M5)) /* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1),	(M5)) /* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2),	(M5)) /* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3),	(M5)) /* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4),	(M5)) /* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5),	(M5)) /* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6),	(M5)) /* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7),	(M5)) /* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_169 */ \
	MV(CP(USBB2_HSIC_STROBE),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_170 */ \
	MV(CP(UNIPRO_TX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col0 */ \
	MV(CP(UNIPRO_TY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col1 */ \
	MV(CP(UNIPRO_TX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col2 */ \
	MV(CP(UNIPRO_TY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col3 */ \
	MV(CP(UNIPRO_TX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col4 */ \
	MV(CP(UNIPRO_TY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col5 */ \
	MV(CP(UNIPRO_RX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row0 */ \
	MV(CP(UNIPRO_RY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row1 */ \
	MV(CP(UNIPRO_RX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row2 */ \
	MV(CP(UNIPRO_RY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row3 */ \
	MV(CP(UNIPRO_RX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row4 */ \
	MV(CP(UNIPRO_RY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row5 */ \
	MV(CP(USBA0_OTG_CE),	(PTU | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* usba0_otg_ce */ \
	MV(CP(USBA0_OTG_DP),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dp */ \
	MV(CP(USBA0_OTG_DM),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dm */ \
	MV(CP(FREF_CLK1_OUT),	(M0_SAFE)) /* fref_clk1_out */ \
	MV(CP(FREF_CLK2_OUT),	(M0_SAFE)) /* fref_clk2_out */ \
	MV(CP(SYS_NIRQ1),	(PTU | IEN | M0)) /* sys_nirq1 */ \
	MV(CP(SYS_NIRQ2),	(PTU | IEN | M0)) /* sys_nirq2 */ \
	MV(CP(SYS_BOOT0),	(M3_SAFE)) /* gpio_184 */ \
	MV(CP(SYS_BOOT1),	(M3_SAFE)) /* gpio_185 */ \
	MV(CP(SYS_BOOT2),	(M3_SAFE)) /* gpio_186 */ \
	MV(CP(SYS_BOOT3),	(PTD | IEN | M3)) /* gpio_187 */ \
	MV(CP(SYS_BOOT4),	(M3_SAFE)) /* gpio_188 */ \
	MV(CP(SYS_BOOT5),	(M3_SAFE)) /* gpio_189 */ \
	MV(CP(DPM_EMU0),	(M0_SAFE)) /* dpm_emu0 */ \
	MV(CP(DPM_EMU1),	(M0_SAFE)) /* dpm_emu1 */ \
	MV(CP(DPM_EMU2),	(M0_SAFE)) /* dpm_emu2 */ \
	MV(CP(DPM_EMU3),	(M5)) /* dispc2_data10 */ \
	MV(CP(DPM_EMU4),	(M5)) /* dispc2_data9 */ \
	MV(CP(DPM_EMU5),	(M5)) /* dispc2_data16 */ \
	MV(CP(DPM_EMU6),	(M5)) /* dispc2_data17 */ \
	MV(CP(DPM_EMU7),	(M5)) /* dispc2_hsync */ \
	MV(CP(DPM_EMU8),	(M5)) /* dispc2_pclk */ \
	MV(CP(DPM_EMU9),	(M5)) /* dispc2_vsync */ \
	MV(CP(DPM_EMU10),	(M5)) /* dispc2_de */ \
	MV(CP(DPM_EMU11),	(M5)) /* dispc2_data8 */ \
	MV(CP(DPM_EMU12),	(M5)) /* dispc2_data7 */ \
	MV(CP(DPM_EMU13),	(M5)) /* dispc2_data6 */ \
	MV(CP(DPM_EMU14),	(M5)) /* dispc2_data5 */ \
	MV(CP(DPM_EMU15),	(M5)) /* dispc2_data4 */ \
	MV(CP(DPM_EMU16),	(M5)) /* dispc2_data3/dmtimer8_pwm_evt */ \
	MV(CP(DPM_EMU17),	(M5)) /* dispc2_data2 */ \
	MV(CP(DPM_EMU18),	(M5)) /* dispc2_data1 */ \
	MV(CP(DPM_EMU19),	(M5)) /* dispc2_data0 */ \
	MV1(WK(PAD0_SIM_IO),	(M0_SAFE)) /* sim_io */ \
	MV1(WK(PAD1_SIM_CLK),	(M0_SAFE)) /* sim_clk */ \
	MV1(WK(PAD0_SIM_RESET),	(M0_SAFE)) /* sim_reset */ \
	MV1(WK(PAD1_SIM_CD),	(M0_SAFE)) /* sim_cd */ \
	MV1(WK(PAD0_SIM_PWRCTRL),	(M0_SAFE)) /* sim_pwrctrl */ \
	MV1(WK(PAD1_SR_SCL),	(PTU | IEN | M0)) /* sr_scl */ \
	MV1(WK(PAD0_SR_SDA),	(PTU | IEN | M0)) /* sr_sda */ \
	MV1(WK(PAD1_FREF_XTAL_IN),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_SLICER_IN),	(M0_SAFE)) /* fref_slicer_in */ \
	MV1(WK(PAD1_FREF_CLK_IOREQ),	(M0_SAFE)) /* fref_clk_ioreq */ \
	MV1(WK(PAD0_FREF_CLK0_OUT),	(M3_SAFE)) /* sys_drm_msecure */ \
	MV1(WK(PAD1_FREF_CLK3_REQ),	(M7_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_CLK3_OUT),	(M0_SAFE)) /* fref_clk3_out */ \
	MV1(WK(PAD1_FREF_CLK4_REQ),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_CLK4_OUT),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD1_SYS_32K),	(IEN | M0_SAFE)) /* sys_32k */ \
	MV1(WK(PAD0_SYS_NRESPWRON),	(IEN | M0_SAFE)) /* sys_nrespwron */ \
	MV1(WK(PAD1_SYS_NRESWARM),	(IEN | M0_SAFE)) /* sys_nreswarm */ \
	MV1(WK(PAD0_SYS_PWR_REQ),	(M0_SAFE)) /* sys_pwr_req */ \
	MV1(WK(PAD1_SYS_PWRON_RESET),	(M3_SAFE)) /* gpio_wk29 */ \
	MV1(WK(PAD0_SYS_BOOT6),	(M3_SAFE)) /* gpio_wk9 */ \
	MV1(WK(PAD1_SYS_BOOT7),	(M3_SAFE)) /* gpio_wk10 */ \
	MV1(WK(PAD1_JTAG_TCK),	(IEN | M0)) /* jtag_tck */ \
	MV1(WK(PAD0_JTAG_RTCK),	(M0)) /* jtag_rtck */ \
	MV1(WK(PAD1_JTAG_TMS_TMSC),	(IEN | M0)) /* jtag_tms_tmsc */ \
	MV1(WK(PAD0_JTAG_TDI),	(IEN | M0)) /* jtag_tdi */ \
	MV1(WK(PAD1_JTAG_TDO),	(M0)) 		  /* jtag_tdo */

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT_OMAP4();
	return;
}

/******************************************************************************
 * Routine: update_mux()
 * Description:Update balls which are different between boards.  All should be
 *             updated to match functionality.  However, I'm only updating ones
 *             which I'll be using for now.  When power comes into play they
 *             all need updating.
 *****************************************************************************/
void update_mux(u32 btype, u32 mtype)
{
	/* REVISIT  */
	return;

}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init();

#if 0 /* No eMMC env partition for now */
	/* Intializing env functional pointers with eMMC */
	boot_env_get_char_spec = mmc_env_get_char_spec;
	boot_env_init = mmc_env_init;
	boot_saveenv = mmc_saveenv;
	boot_env_relocate_spec = mmc_env_relocate_spec;
	env_ptr = (env_t *) (CFG_FLASH_BASE + CFG_ENV_OFFSET);
	env_name_spec = mmc_env_name_spec;
#endif

	/* board id for Linux */
#if defined(OMAP44XX_TABLET_CONFIG)
	gd->bd->bi_arch_number = MACH_TYPE_OMAP_BLAZE;
#else
	gd->bd->bi_arch_number = MACH_TYPE_OMAP_4430SDP;
#endif
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */

#if 0
	// Change I2C3 pullup resistor to 4.5k/1.66k ohm
	__raw_writel((__raw_readl(OMAP44XX_CTRL_BASE+CONTROL_I2C_0) & 0xF9FFF9FF),
		OMAP44XX_CTRL_BASE+CONTROL_I2C_0);
#endif

	return 0;
}

typedef enum {
	C1_EVT1		= 3,
	C1_EVT2		= 0,
	C1_DVT1_ATCN	= 1,
	C1_DVT1_ATCE	= 2,
	UNVERSIONED	= 4,
} c1_version;

static c1_version C1_VERSION = UNVERSIONED;
void c1_version_init(void)
{
// The internal pullup resistors will cause problems on DVT devices
// Do NOT turn on PTU unless for EVT1 devices
	MV(CP(GPMC_A24),	(IEN | M3)); /* gpio_48 */
	MV(CP(GPMC_A25),	(IEN | M3)); /* gpio_49 */

	__raw_writel(__raw_readl(GPIO2_OE) | (3 << (48%32)), GPIO2_OE);
	C1_VERSION = (__raw_readl(GPIO2_DATAIN) & (3 << (48%32))) >> (48%32);
// debug message for HW revision check
#if 1
	switch (C1_VERSION) {
		case C1_EVT1:
			printf("*** Detected as EVT1 ***\n");
			break;
		case C1_EVT2:
			printf("*** Detected as EVT2 ***\n");
			break;
		case C1_DVT1_ATCN:
			printf("*** Detected as DVT1(ATCN) ***\n");
			break;
		case C1_DVT1_ATCE:
			printf("*** Detected as DVT1(ATCE) ***\n");
			break;
		default:
			printf("*** Unknown HW revision! ***\n");
			break;
	}
#endif
}
int board_is_c1_evt1(void)
{
	return (C1_VERSION == C1_EVT1);
}
int board_is_c1_evt2(void)
{
	return (C1_VERSION == C1_EVT2);
}

#define WKEN (1 << 14)
// FIXME: This process should move to kernel for better power control
int setup_gpio_wk0_wk1(void)
{
	u32 value = 0;
	u8 data = 0;
	void *reg;

	/* EVT1 doesn't use gpio_wk0/gpio_wk1 */
	if (board_is_c1_evt1()) return 0;

	/* PBIASLITE1_PWRDNZ = 0, USBC1_ICUSB_PWRDNZ = 0 */
	reg = OMAP44XX_CTRL_BASE + CONTROL_PBIASLITE;
	value = __raw_readl(reg);
	value = value & 0xEFEFFFFF;
	__raw_writel(value, reg);

	/* GPIO_WK1_LOW = 0 */
	reg = OMAP44XX_WKUP_CTRL_BASE + CONTROL_CONTROL_GPIOWK;
	value = __raw_readl(reg);
	value = value & 0xEFFFFFFF;
	__raw_writel(value, reg);

	/* PBIASLITE1_HIZ_MODE = 0 */
	reg = OMAP44XX_CTRL_BASE + CONTROL_PBIASLITE;
	value = __raw_readl(reg);
	value = value & 0x7FFFFFFF;
	__raw_writel(value, reg);

	/* GPIOWK_IO_PWRDNZ = 0 */
	reg = OMAP44XX_WKUP_CTRL_BASE + CONTROL_CONTROL_GPIOWK;
	value = __raw_readl(reg);
	value = value & 0x5FFFFFFF;
	__raw_writel(value, reg);

	/* Phoenix LDO config */
	data = 0x01;
	i2c_write(0x48, 0xA4, 1, &data, 1);
	data = 0x03;
	i2c_write(0x48, 0xA5, 1, &data, 1);
	data = 0x21;
	i2c_write(0x48, 0xA6, 1, &data, 1);
	data = 0x09; // 0x09=1.8V 0x15=3.0V
	i2c_write(0x48, 0xA7, 1, &data, 1);

	/* PBIASLITE1_VMODE = 0 */
	reg = OMAP44XX_CTRL_BASE + CONTROL_PBIASLITE;
	value = __raw_readl(reg);
	value = value & 0xF7FFFFFF;
	__raw_writel(value, reg);

	udelay(1000);

	/* PBIASLITE1_PWRDNZ = 1, USBC1_ICUSB_PWRDNZ = 1 */
	reg = OMAP44XX_CTRL_BASE + CONTROL_PBIASLITE;
	value = __raw_readl(reg);
	value = value | 0x10100000;
	__raw_writel(value, reg);

	/* GPIOWK_IO_PWRDNZ = 1 */
	reg = OMAP44XX_WKUP_CTRL_BASE + CONTROL_CONTROL_GPIOWK;
	value = __raw_readl(reg);
	value = value | 0x10000000;
	__raw_writel(value, reg);

	/*gpio wk0, tca6416 irq*/
	MV1(WK(PAD0_SIM_IO),	(WKEN | IEN | M3));
	/*gpio wk1, tp irq*/
	MV1(WK(PAD1_SIM_CLK),	(WKEN | PTU | IEN | M3));

	return 0;
}

#define QUICKVX_I2C_ADDR     0x64
#define TCA6416_I2C_ADDR     0x21
#define PANEL_I2C_ADDR     0x48
#define GB_I2C_BUS      2
#define I2C_RETRY_TIMES      5

#define TCA6416_INPUT	0
#define TCA6416_OUTPUT	2
#define TCA6416_INVERT	4
#define TCA6416_CONFIG	6//0 output, 1 input

#define EN1V2_PIN 0
#define EN1V8_PIN 1
#define EN2V8_PIN 2
#define EN3V3_PIN 3
#define EN10V_PIN 4
#define DISP_OE_RSTB_PIN 8
#define BRIDGE_RSTN_PIN 9
#define PWM_HIGH 6
#define PWM_MID 7
#define PWM_LOW 12

struct tca6416_chip {
	u16 reg_output;
	u16 reg_config;
};
struct tca6416_chip tca6416;

static int tca6416_gpio_output(u8 off, u8 val)
{
	uchar val_to_write; 
	u16 reg_val;
	uint reg_addr;
	int ret;
	
	/* set output level */
	if (val)
		reg_val = tca6416.reg_output | (1u << off);
	else
		reg_val = tca6416.reg_output & ~(1u << off);
	if(off < 8){
		val_to_write = reg_val & 0xff;
		reg_addr = TCA6416_OUTPUT;
	}else{
		val_to_write = (reg_val >> 8) & 0xff;
		reg_addr = TCA6416_OUTPUT + 1;
	}
	if (ret = i2c_write(TCA6416_I2C_ADDR, reg_addr, 1, &val_to_write, 1)) 
		goto exit;
	tca6416.reg_output = reg_val;

	/* then config direction */
	reg_val = tca6416.reg_config & ~(1u << off);
	if(off < 8){
		val_to_write = reg_val & 0xff;
		reg_addr = TCA6416_CONFIG;
	}else{
		val_to_write = (reg_val >> 8) & 0xff;
		reg_addr = TCA6416_CONFIG + 1;
	}
	if (ret = i2c_write(TCA6416_I2C_ADDR, reg_addr, 1, &val_to_write, 1)) 
		goto exit;
	tca6416.reg_config = reg_val;
	
	ret = 0;
exit:
	return ret;
}
static int tca6416_gpio_input(u8 off)
{
	uchar val_to_write; 
	u16 reg_val;
	uint reg_addr;
	int ret;

	reg_val = tca6416.reg_config | (1u << off);
	if(off < 8){
		val_to_write = reg_val & 0xff;
		reg_addr = TCA6416_CONFIG;
	}else{
		val_to_write = (reg_val >> 8) & 0xff;
		reg_addr = TCA6416_CONFIG + 1;
	}
	if (ret = i2c_write(TCA6416_I2C_ADDR, reg_addr, 1, &val_to_write, 1)) 
		goto exit;
	tca6416.reg_config = reg_val;

	ret = 0;
exit:
	return ret;
}

static int enable_display_power(void)
{
	int status;

	printf("Initializing LCD power\n");
	if (!board_is_c1_evt1()) {
		//Turn on GB pwr (GPIO_151)
		MV(CP(MCSPI4_CLK),        (PTU | M3))
		__raw_writel(__raw_readl(GPIO5_OE) & ~(1 << (151%32)), GPIO5_OE);
		__raw_writel(__raw_readl(GPIO5_SETDATAOUT) | (1 << (151%32)), GPIO5_SETDATAOUT);
	}
	/* configure I2C bus */
	if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return status;
	}

#ifndef CONFIG_COMPACT_I2C_SETTING
	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	if (status)
		return status;
	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
	if (status)
		return status;

	//enable 1v2
	tca6416_gpio_output(EN1V2_PIN, 1);
	udelay(20000);
	//enable 1v8
	tca6416_gpio_output(EN1V8_PIN, 0);
	udelay(20000);
	//enable 3v3
	tca6416_gpio_output(EN3V3_PIN, 1);
	udelay(20000);
	//enable 10v
	tca6416_gpio_output(EN10V_PIN, 1);
	udelay(20000);
	//reset bridge
	tca6416_gpio_output(BRIDGE_RSTN_PIN, 1);
	udelay(20000);
	//reset panel
	tca6416_gpio_output(DISP_OE_RSTB_PIN, 1);
	udelay(20000);
#else
	// reduce i2c accesses to speed up the process
	tca6416.reg_output = (tca6416.reg_output | 0x0309) & ~(0x0012);
	tca6416.reg_config = tca6416.reg_config & ~(0x031b);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
#endif

	// Turn off backlight to avoid screen flickering
#if 0
	//set pwm high to low
	tca6416_gpio_output(PWM_HIGH, 0);
	udelay(20000);
	//set pwm mid to high
	tca6416_gpio_output(PWM_MID, 1);
	udelay(20000);
	//set pwm low to low
	tca6416_gpio_output(PWM_LOW, 0);
	udelay(20000);
#endif

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");

	return status;
}

static int disable_display_power(void)
{
	int status;
	
	printf("Uninitializing LCD power\n");
	if (!board_is_c1_evt1()) {
		//Turn off GB pwr (GPIO_151)
		MV(CP(MCSPI4_CLK),        (PTD | M3))
		__raw_writel(__raw_readl(GPIO5_OE) & ~(1 << (151%32)), GPIO5_OE);
		__raw_writel(__raw_readl(GPIO5_CLEARDATAOUT) | (1 << (151%32)), GPIO5_CLEARDATAOUT);
	} else {
		/* configure I2C bus */
		if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
			printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
			return status;
		}

#ifndef CONFIG_COMPACT_I2C_SETTING
		status = i2c_read(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
		if (status)
			return status;
		status = i2c_read(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
		if (status)
			return status;

		//disable 1v2
		tca6416_gpio_output(EN1V2_PIN, 0);
		udelay(20000);
		//disable 1v8
		tca6416_gpio_output(EN1V8_PIN, 1);
		udelay(20000);
		//disable 3v3
		tca6416_gpio_output(EN3V3_PIN, 0);
		udelay(20000);
		//disable 10v
		tca6416_gpio_output(EN10V_PIN, 0);
		udelay(20000);
		//reset bridge
		tca6416_gpio_output(BRIDGE_RSTN_PIN, 0);
		udelay(20000);
		//reset panel
		tca6416_gpio_output(DISP_OE_RSTB_PIN, 0);
		udelay(20000);
#else
		// reduce i2c accesses to speed up the process
		tca6416.reg_output = (tca6416.reg_output | 0x0309) & ~(0x0012);
		tca6416.reg_config = tca6416.reg_config & ~(0x031b);
		i2c_write(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
		i2c_write(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
#endif

		/* restore default settings */
		if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
			printf("Setting i2c defaults: FAILED\n");
	}

	return status;
}

static int enable_backlight(void)
{
	int status;

	if(bl_is_on)
		return 0;
	/* configure I2C bus */
	if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return status;
	}

#ifndef CONFIG_COMPACT_I2C_SETTING
	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	if (status)
		return status;

	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
	if (status)
		return status;

	//set pwm high to low
	tca6416_gpio_output(PWM_HIGH, 0);
	udelay(20000);
	//set pwm mid to high
	tca6416_gpio_output(PWM_MID, 1);
	udelay(20000);
	//set pwm low to low
	tca6416_gpio_output(PWM_LOW, 0);
	udelay(20000);
#else
	// reduce i2c accesses to speed up the process
	tca6416.reg_output = (tca6416.reg_output | 0x0080) & ~(0x1040);
	tca6416.reg_config = tca6416.reg_config & ~(0x10c0);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
#endif

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");
	bl_is_on = 1;
	return status;
}

static int disable_backlight(void)
{
	int status;

	if(!bl_is_on)
		return 0;
	/* configure I2C bus */
	if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return status;
	}

#ifndef CONFIG_COMPACT_I2C_SETTING
	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	if (status)
		return status;
	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
	if (status)
		return status;

	//set pwm high to low
	tca6416_gpio_output(PWM_HIGH, 0);
	udelay(20000);
	//set pwm mid to low
	tca6416_gpio_output(PWM_MID, 0);
	udelay(20000);
	//set pwm low to low
	tca6416_gpio_output(PWM_LOW, 0);
	udelay(20000);
#else
	// reduce i2c accesses to speed up the process
	tca6416.reg_output = tca6416.reg_output & ~(0x10c0);
	tca6416.reg_config = tca6416.reg_config & ~(0x10c0);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
#endif

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");
	bl_is_on = 0;
	return status;
}

static char panel_data[][2] =
{
	{0x00, 0x01},
	{0x02, 0x04},
	{0x03, 0x01}, //VCOMPN Setting
	{0x04, 0x0A}, //VCOM Setting
	{0x05, 0x01}, //VCOMPN Setting
	{0x06, 0x0A}, //VCOM Setting
	{0x07, 0xF9}, //VRINGP Setting
	{0x08, 0x05}, //VRINGN Setting
	{0x09, 0xFF}, //RGMAP1 Setting
	{0x0A, 0x84}, //RGMAP2 Setting
	{0x0B, 0x68}, //RGMAP3 Setting
	{0x0C, 0x54}, //RGMAP4 Setting
	{0x0D, 0x34}, //RGMAP5 Setting
	{0x0E, 0x0A}, //RGMAP6 Setting
	{0x0F, 0x00}, //RGMAN1 Setting
	{0x10, 0x7A}, //RGMAN2 Setting
	{0x11, 0x96}, //RGMAN3 Setting
	{0x12, 0xAA}, //RGMAN4 Setting
	{0x13, 0xCA}, //RGMAN5 Setting
	{0x14, 0xFF}, //RGMAN6 Setting
	{0x15, 0xFF}, //GGMAP1 Setting
	{0x16, 0x84}, //GGMAP2 Setting
	{0x17, 0x68}, //GGMAP3 Setting
	{0x18, 0x54}, //GGMAP4 Setting
	{0x19, 0x34}, //GGMAP5 Setting
	{0x1A, 0x0A}, //GGMAP6 Setting
	{0x1B, 0x00}, //GGMAN1 Setting
	{0x1C, 0x7A}, //GGMAN2 Setting
	{0x1D, 0x96}, //GGMAN3 Setting
	{0x1E, 0xAA}, //GGMAN4 Setting
	{0x1F, 0xCA}, //GGMAN5 Setting
	{0x20, 0xFF}, //GGMAN6 Setting
	{0x21, 0xFF}, //BGMAP1 Setting
	{0x22, 0x84}, //BGMAP2 Setting
	{0x23, 0x68}, //BGMAP3 Setting
	{0x24, 0x54}, //BGMAP4 Setting
	{0x25, 0x34}, //BGMAP5 Setting
	{0x26, 0x0A}, //BGMAP6 Setting
	{0x27, 0x00}, //BGMAN1 Setting
	{0x28, 0x7A}, //BGMAN2 Setting
	{0x29, 0x96}, //BGMAN3 Setting
	{0x2A, 0xAA}, //BGMAN4 Setting
	{0x2B, 0XCA}, //BGMAN5 Setting
	{0x2C, 0xFF}, //BGMAN6 Setting
	{0xFF, 0xFF}	//end
};
static int panel_eyepiece_init(void)
{
	int status;
	int i;
	int retry;

	/* configure I2C bus */
	if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return status;
	}
	
    /* write panel init buffer data*/
	for (i = 0; 0xFF != panel_data[i][0]; i++) {
		retry = 5;
		while (retry-- > 0) {
			if ((status = i2c_write(PANEL_I2C_ADDR, panel_data[i][0], 1, &panel_data[i][1], 1)) == 0) {
				//printf("panel write register: %02x %02x succeed on %d try \n", panel_data[i][0], panel_data[i][1], 5-retry);
				break;
			}
			printf("panel write register: reg(%02x) val(%02x) failed on %d try\n", panel_data[i][0], panel_data[i][1], 5-retry);
			udelay(10000);
		}
	}

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");
			
	return status;
}

/* QuickVX Register Settings */
struct chip_init_data {
	u16 reg_addr; 
	u32 data;
};
struct chip_init_data vx3_init_data[] = {
	0x700 , 0x40900040 ,
	0x704 , 0x1F01E3 ,
	0x70C , 0x4604 ,
	0x710 , 0x5CD0007 ,
	0x714 , 0x0 ,
	0x718 , 0x00000102 ,
	0x71C , 0xA800B ,
	0x720 , 0x0 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x700 , 0x40900040 ,
	0x70C , 0x5E26 ,
	0x718 , 0x00000002 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x120 , 0x5 ,
	0x124 , 0x74F0320 ,
	0x128 , 0x10480A ,
	0x12C , 0x1D ,
	0x130 , 0x3C10 ,
	0x134 , 0x5 ,
	0x138 , 0xFF8000 ,
	0x13C , 0x0 ,
	0x140 , 0x10000 ,
	0x20C , 0x22 ,
	0x21C , 0x780 ,
	0x224 , 0x0 ,
	0x228 , 0x50000 ,
	0x22C , 0xFF06 ,
	0x230 , 0x1 ,
	0x234 , 0xCA033E10 ,
	0x238 , 0x00000060 ,
	0x23C , 0x82E86030 ,
	0x244 , 0x001E0285 ,
	0x258 , 0x14001D ,
	0x158 , 0x0 ,
	0x158 , 0x1 ,
	0x37C , 0x00001063 ,
	0x380 , 0x82A86030 ,
	0x384 , 0x2861408B ,
	0x388 , 0x00130285 ,
	0x38C , 0x10630009 ,
	0x394 , 0x400B82A8 ,
	//0x600 , 0x16CC78C ,
	//0x604 , 0x3FFFFFE0 ,
	0x608 , 0x50F ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	// internel debug signal
#if 0
	0x710, 0x4d000f,
	0x71c, 0xa8002b,
	0x70c, 0x5666,
	0x130, 0x3c78,
	0x134, 0x5,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
#endif

#if 0
	//don't remove this. Need to restart the host after the init.......
	{0x300,0x00000000},
	{0x300,0x00000001},
	{0x304,0xffffffff},
	{0x204,0xffffffff},
	{0x148,0xffffffff},
#endif
};

#define CONTROL_BYTE_GEN       (0x09u)

#define GEN_QL_CSR_OFFSET_LENGTH  {\
	CONTROL_BYTE_GEN, \
	0x29,  /* Data ID */\
	0x05,  /* Vendor Id 1 */\
	0x01,  /* Vendor Id 2 */\
	0x41,  /* Vendor Unique Command */\
	0x00,  /* Address LS */\
	0x00,  /* Address MS */\
	0x00,  /* Length LS */\
	0x00,  /* Length MS */\
}

#define GEN_QL_CSR_WRITE  {\
	CONTROL_BYTE_GEN, \
	0x29,  /* Data ID */\
	0x05,  /* Vendor Id 1 */\
	0x01,  /* Vendor Id 2 */\
	0x40,  /* Vendor Unique Command */\
	0x00,  /* Address LS */\
	0x00,  /* Address MS */\
	0x00,  /* data LS */\
	0x00, \
	0x00, \
	0x00,  /* data MS */\
}
int ql_i2c_read(u16 addr, u32 *val, int data_size) 
{
	uint32_t data;
	char buf[] = GEN_QL_CSR_OFFSET_LENGTH;
	char rx[10];
	int ret = -1;
	int write_size;
	int retry_times = I2C_RETRY_TIMES;

	buf[5] = addr & 0xff;
	buf[6] = (addr >> 8) & 0xff;
	buf[7] = data_size & 0xff;
	buf[8] = (data_size >> 8) & 0xff;
	write_size = 9;
retry1:
	if (ret = i2c_xfer(QUICKVX_I2C_ADDR, buf, write_size, I2C_XFER_M_WR)) {
		printf( "ql_i2c_read: i2c_write_datas GEN failed (%d)!\n", ret);
		if(retry_times -- > 0)
			goto retry1;
		return -1;
	}
	
	retry_times = I2C_RETRY_TIMES;
	buf[0] = CONTROL_BYTE_GEN;
	buf[1] =    0x24;  /* Data ID */
	buf[2] =    0x05;  /* Vendor Id 1 */
	buf[3] =    0x01;  /* Vendor Id 2 */
	write_size = 4;
retry2:
	if (ret = i2c_xfer(QUICKVX_I2C_ADDR, buf, write_size, I2C_XFER_M_WR)) {
		printf( "ql_i2c_read: i2c_write_datas failed (%d)!\n", ret);
		if(retry_times -- > 0)
			goto retry2;
		return -1;
	}
	
	retry_times = I2C_RETRY_TIMES;
retry3:
	/* Return number of bytes or error */
	if (ret = i2c_xfer(QUICKVX_I2C_ADDR, rx, data_size, I2C_XFER_M_RD)) {
		printf( "ql_i2c_read: i2c_master_recv failed (%d)!\n", ret);
		if(retry_times -- > 0)
			goto retry3;
		return -1;
	}

	data = rx[0];
	if (data_size > 1) 
		data |= (rx[1] << 8);
	if (data_size > 2)
		data |= (rx[2] << 16) | (rx[3] << 24);

	*val = data;

	//printf("r 0x%x=0x%x\n",addr,data);
	return 0;
}

int ql_i2c_write(u16 addr, u32 val, int data_size)
{
	int write_size;
	char buf[] = GEN_QL_CSR_WRITE;

	buf[5] = (uint8_t)addr;  /* Address LS */
	buf[6] = (uint8_t)(addr >> 8);  /* Address MS */

	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 2) ? ((val >> 24) & 0xff) : 0;

	write_size = data_size + 7;

	if ( i2c_xfer(QUICKVX_I2C_ADDR, buf, write_size, I2C_XFER_M_WR)) {
		//printf("ql_i2c_write error\n");
		return -1;
	}
	return 0;
}

static void ql_chip_init(void)
{
	int status;
	int i;
	int reg_addr;
	int data;
	int retry;
	u32 val = 0x8899;
	
	printf("ql_chip_init\n");
	/* configure I2C bus */
	if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return status;
	}

	ql_i2c_read(0x4fe, &val, 2); 
	printf("VEE ID 0x%x\n", val);
#if 0	
	ql_i2c_write(0x448, 0x12345678, 4); 
	ql_i2c_read(0x448, &val, 4);
	printf("R/W Test 0x%x\n", val);
#endif
	
	for (i=0; i < (sizeof(vx3_init_data)/sizeof(struct chip_init_data)); i++) {
		reg_addr = vx3_init_data[i].reg_addr;
		data = vx3_init_data[i].data;
		
		retry = I2C_RETRY_TIMES;
		while (retry-- > 0) {
			if ((status = ql_i2c_write(reg_addr, data, 4)) == 0) {
				//printf("QUICKVX write register: reg(0x%04x) val(0x%08x) succeed on %d try \n", reg_addr, data, I2C_RETRY_TIMES-retry);
				break;
			}
			printf("QUICKVX write register: reg(0x%04x) val(0x%08x) failed on %d try\n", reg_addr, data, I2C_RETRY_TIMES-retry);
			udelay(10000);
		}
	}

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");
		
	return status;
	
}


/* DSS registers */
#define DSS_BASE                        0x48040000
#define DSS_REVISION                    (DSS_BASE + 0x0000)
#define DSS_SYSCONFIG                   (DSS_BASE + 0x0010)
#define DSS_SYSSTATUS                   (DSS_BASE + 0x0014)
#define DSS_CONTROL                     (DSS_BASE + 0x0040)
#define DSS_SDI_CONTROL                 (DSS_BASE + 0x0044)
#define DSS_PLL_CONTROL                 (DSS_BASE + 0x0048)
#define DSS_STATUS                      (DSS_BASE + 0x005C)

/* DISPC common registers */
#define DISPC_BASE			0x48041000
#define DISPC_REVISION			(DISPC_BASE + 0x0000)
#define DISPC_SYSCONFIG			(DISPC_BASE + 0x0010)
#define DISPC_SYSSTATUS			(DISPC_BASE + 0x0014)
#define DISPC_IRQSTATUS			(DISPC_BASE + 0x0018)
#define DISPC_IRQENABLE			(DISPC_BASE + 0x001C)
#define DISPC_CONTROL			(DISPC_BASE + 0x0040)
#define DISPC_CONFIG			(DISPC_BASE + 0x0044)
#define DISPC_LINE_STATUS		(DISPC_BASE + 0x005C)
#define DISPC_LINE_NUMBER		(DISPC_BASE + 0x0060)
#define DISPC_GLOBAL_ALPHA		(DISPC_BASE + 0x0074)
#define DISPC_CONTROL2			(DISPC_BASE + 0x0238)
#define DISPC_CONFIG2			(DISPC_BASE + 0x0620)
#define DISPC_DIVISOR			(DISPC_BASE + 0x0804)
#define DISPC_GLOBAL_MFLAG		(DISPC_BASE + 0x085C)

/* DISPC overlay registers */
#define DISPC_GFX_BA0                   (DISPC_BASE + 0x0080)
#define DISPC_GFX_BA1                   (DISPC_BASE + 0x0084)
#define DISPC_GFX_POSITION              (DISPC_BASE + 0x0088)
#define DISPC_GFX_SIZE                  (DISPC_BASE + 0x008C)
#define DISPC_GFX_ATTRIBUTES            (DISPC_BASE + 0x00A0)
#define DISPC_GFX_BUF_THRESHOLD         (DISPC_BASE + 0x00A4)
#define DISPC_GFX_BUF_SIZE_STATUS       (DISPC_BASE + 0x00A8)
#define DISPC_GFX_ROW_INC               (DISPC_BASE + 0x00AC)
#define DISPC_GFX_PIXEL_INC             (DISPC_BASE + 0x00B0)
#define DISPC_GFX_TABLE_BA              (DISPC_BASE + 0x00B8)
#define DISPC_GFX_PRELOAD               (DISPC_BASE + 0x022C)

/* DSI Protocol Engine1 */
#define DSI_BASE			0x48044000
#define DSI_REVISION			(DSI_BASE + 0x0000)
#define DSI_SYSCONFIG			(DSI_BASE + 0x0010)
#define DSI_SYSSTATUS			(DSI_BASE + 0x0014)
#define DSI_IRQSTATUS			(DSI_BASE + 0x0018)
#define DSI_IRQENABLE			(DSI_BASE + 0x001C)
#define DSI_CTRL			(DSI_BASE + 0x0040)
#define DSI_GNQ				(DSI_BASE + 0x0044)
#define DSI_COMPLEXIO_CFG1		(DSI_BASE + 0x0048)
#define DSI_COMPLEXIO_IRQ_STATUS	(DSI_BASE + 0x004C)
#define DSI_COMPLEXIO_IRQ_ENABLE	(DSI_BASE + 0x0050)
#define DSI_CLK_CTRL			(DSI_BASE + 0x0054)
#define DSI_TIMING1			(DSI_BASE + 0x0058)
#define DSI_TIMING2			(DSI_BASE + 0x005C)
#define DSI_VM_TIMING1			(DSI_BASE + 0x0060)
#define DSI_VM_TIMING2			(DSI_BASE + 0x0064)
#define DSI_VM_TIMING3			(DSI_BASE + 0x0068)
#define DSI_CLK_TIMING			(DSI_BASE + 0x006C)
#define DSI_TX_FIFO_VC_SIZE		(DSI_BASE + 0x0070)
#define DSI_RX_FIFO_VC_SIZE		(DSI_BASE + 0x0074)
#define DSI_COMPLEXIO_CFG2		(DSI_BASE + 0x0078)
#define DSI_RX_FIFO_VC_FULLNESS		(DSI_BASE + 0x007C)
#define DSI_VM_TIMING4			(DSI_BASE + 0x0080)
#define DSI_TX_FIFO_VC_EMPTINESS	(DSI_BASE + 0x0084)
#define DSI_VM_TIMING5			(DSI_BASE + 0x0088)
#define DSI_VM_TIMING6			(DSI_BASE + 0x008C)
#define DSI_VM_TIMING7			(DSI_BASE + 0x0090)
#define DSI_STOPCLK_TIMING		(DSI_BASE + 0x0094)
#define DSI_VC_CTRL(n)			(DSI_BASE + 0x0100 + (n * 0x20))
#define DSI_VC_TE(n)			(DSI_BASE + 0x0104 + (n * 0x20))
#define DSI_VC_LONG_PACKET_HEADER(n)	(DSI_BASE + 0x0108 + (n * 0x20))
#define DSI_VC_LONG_PACKET_PAYLOAD(n)	(DSI_BASE + 0x010C + (n * 0x20))
#define DSI_VC_SHORT_PACKET_HEADER(n)	(DSI_BASE + 0x0110 + (n * 0x20))
#define DSI_VC_IRQSTATUS(n)		(DSI_BASE + 0x0118 + (n * 0x20))
#define DSI_VC_IRQENABLE(n)		(DSI_BASE + 0x011C + (n * 0x20))

/* DSIPHY_SCP */
#define DSI_DSIPHY_CFG0			(DSI_BASE + 0x200 + 0x0000)
#define DSI_DSIPHY_CFG1			(DSI_BASE + 0x200 + 0x0004)
#define DSI_DSIPHY_CFG2			(DSI_BASE + 0x200 + 0x0008)
#define DSI_DSIPHY_CFG5			(DSI_BASE + 0x200 + 0x0014)
#define DSI_DSIPHY_CFG10		(DSI_BASE + 0x200 + 0x0028)

/* DSI_PLL_CTRL_SCP */
#define DSI_PLL_CONTROL			(DSI_BASE + 0x300)
#define DSI_PLL_STATUS			(DSI_BASE + 0x304)
#define DSI_PLL_GO			(DSI_BASE + 0x308)
#define DSI_PLL_CONFIGURATION1		(DSI_BASE + 0x30C)
#define DSI_PLL_CONFIGURATION2		(DSI_BASE + 0x310)

/* CONTROL MODULE */
#define CONTROL_DSIPHY			0x4A100618

enum omap_channel {
	OMAP_DSS_CHANNEL_LCD	= 0,
	OMAP_DSS_CHANNEL_DIGIT	= 1,
	OMAP_DSS_CHANNEL_LCD2	= 2,
};

/* DISPC manager/channel specific registers */
static inline u32 DISPC_DEFAULT_COLOR(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_BASE + 0x004C;
	case OMAP_DSS_CHANNEL_DIGIT:
		return DISPC_BASE + 0x0050;
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_BASE + 0x03AC;
	default:
		BUG();
	}
}

static inline u32 DISPC_TIMING_H(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_BASE + 0x0064;
	case OMAP_DSS_CHANNEL_DIGIT:
		BUG();
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_BASE + 0x0400;
	default:
		BUG();
	}
}

static inline u32 DISPC_TIMING_V(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_BASE + 0x0068;
	case OMAP_DSS_CHANNEL_DIGIT:
		BUG();
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_BASE + 0x0404;
	default:
		BUG();
	}
}

static inline u32 DISPC_DIVISORo(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_BASE + 0x0070;
	case OMAP_DSS_CHANNEL_DIGIT:
		return DISPC_BASE + 0x0070;
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_BASE + 0x040C;
	default:
		BUG();
	}
}

static inline u32 DISPC_SIZE_MGR(enum omap_channel channel)
{
	switch (channel) {
	case OMAP_DSS_CHANNEL_LCD:
		return DISPC_BASE + 0x007C;
	case OMAP_DSS_CHANNEL_DIGIT:
		return DISPC_BASE + 0x0078;
	case OMAP_DSS_CHANNEL_LCD2:
		return DISPC_BASE + 0x03CC;
	default:
		BUG();
	}
}

static inline u32 DISPC_POL_FREQ(enum omap_channel channel)
{
        switch (channel) {
        case OMAP_DSS_CHANNEL_LCD:
                return DISPC_BASE + 0x006C;
        case OMAP_DSS_CHANNEL_DIGIT:
                BUG();
        case OMAP_DSS_CHANNEL_LCD2:
                return DISPC_BASE + 0x0408;
        default:
                BUG();
        }
}

/* Panel config */
#define PANEL_WIDTH     800    
#define PANEL_HEIGHT    480  
#define logo_width      220
#define logo_height     68
#define FRAME_ADDR     0x9a000000
#define bat_img_width	134
#define bat_img_height	212
#define bat_level_width	82
#define level_5_percentage_height 7
#define percentage_img_height	36
#define fastboot_img_top_width	186
#define fastboot_img_top_height	186
#define fastboot_img_mid_width	188
#define fastboot_img_mid_height	39
#define fastboot_img_bot_width	622
#define fastboot_img_bot_height	29
#define prompt_text_width	206
#define prompt_text_height	30

#define DISPC_HFP	72
#define DISPC_HSW	7
#define DISPC_HBP	1
#define DISPC_VFP	18
#define DISPC_VSW	20
#define DISPC_VBP	10
#define BPP		24
#define LANES		2

/* PLL config */
#define LCK_DIV		1
#define PCK_DIV		3
#define REGM		279
#define REGN		15//value on kernel - 1
#define REGM_DISPC	8
#define REGM_DSI	8

#define LANEMASK1	(0x1F >> (4-LANES))
#define LANEMASK2	(0x54321 & (0xFFFFF >> (4*(4-LANES))))

/* OMAP TRM gives bitfields as start:end, where start is the higher bit
   number. For example 7:0 */
#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) (((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))
	

#define REG_GET(regaddr, start, end) \
	FLD_GET(__raw_readl(regaddr), start, end)

#define REG_FLD_MOD(regaddr, val, start, end) \
	__raw_writel(FLD_MOD(__raw_readl(regaddr), val, start, end), regaddr)
	
static int wait_for_bit_change(u32 regaddr, int bitnum, int value)
{
	int t;
	
	/* first busyloop to see if the bit changes right away */
	t = 100;
	while (t-- > 0) {
		if (REG_GET(regaddr, bitnum, bitnum) == value)
			return value;
	}
	t = 1000;
	while (t-- > 0) {
		if (REG_GET(regaddr, bitnum, bitnum) == value)
			return value;
		udelay(10);
	}

	return !value;
}
static inline void dsi_enable_scp_clk(void)
{
	REG_FLD_MOD(DSI_CLK_CTRL, 1, 14, 14); /* CIO_CLK_ICG */
}
static inline void dsi_disable_scp_clk(void)
{
	REG_FLD_MOD(DSI_CLK_CTRL, 0, 14, 14); /* CIO_CLK_ICG */
}
static inline void dsi_enable_pll_clock(void)
{
	__raw_writel(0x00000702, CM_DSS_CLKSTCTRL);
}
static inline void dsi_disable_pll_clock(void)
{
	__raw_writel(0x00000001, CM_DSS_CLKSTCTRL);
}

static int dsi_pll_set_clock_div(void)
{
	int r = 0;
	u32 l;
	int f = 0;

	/* DSI_PLL_AUTOMODE = manual */
	REG_FLD_MOD(DSI_PLL_CONTROL, 0, 0, 0);
	
	l = __raw_readl(DSI_PLL_CONFIGURATION1);
	l = FLD_MOD(l, 1, 0, 0);		/* DSI_PLL_STOPMODE */
	/* DSI_PLL_REGN */
	l = FLD_MOD(l, REGN, 8, 1);
	/* DSI_PLL_REGM */
	l = FLD_MOD(l, REGM, 20, 9);
	/* DSI_CLOCK_DIV */
	l = FLD_MOD(l, REGM_DISPC > 0 ? REGM_DISPC - 1 : 0,
			25, 21);
	/* DSIPROTO_CLOCK_DIV */
	l = FLD_MOD(l, REGM_DSI > 0 ? REGM_DSI - 1 : 0,
			30, 26);
	__raw_writel(l, DSI_PLL_CONFIGURATION1);

	l = __raw_readl(DSI_PLL_CONFIGURATION2);

	l = FLD_MOD(l, 0, 11, 11);		/* DSI_PLL_CLKSEL */
	l = FLD_MOD(l, 0, 12, 12);		/* DSI_PLL_HIGHFREQ */
	l = FLD_MOD(l, 1, 13, 13);		/* DSI_PLL_REFEN */
	l = FLD_MOD(l, 0, 14, 14);		/* DSIPHY_CLKINEN */
	l = FLD_MOD(l, 1, 20, 20);		/* DSI_HSDIVBYPASS */

	__raw_writel(l, DSI_PLL_CONFIGURATION2);
	REG_FLD_MOD(DSI_PLL_GO, 1, 0, 0);	/* DSI_PLL_GO */

	if (wait_for_bit_change(DSI_PLL_GO, 0, 0) != 0) {
		printf("dsi pll go bit not going down.\n");
		r = -1;
		return r;
	}
	if (wait_for_bit_change(DSI_PLL_STATUS, 1, 1) != 1) {
		printf("cannot lock PLL\n");
		r = -1;
		return r;
	}
	l = __raw_readl(DSI_PLL_CONFIGURATION2);
	l = FLD_MOD(l, 0, 0, 0);	/* DSI_PLL_IDLE */
	l = FLD_MOD(l, 0, 5, 5);	/* DSI_PLL_PLLLPMODE */
	l = FLD_MOD(l, 0, 6, 6);	/* DSI_PLL_LOWCURRSTBY */
	l = FLD_MOD(l, 0, 7, 7);	/* DSI_PLL_TIGHTPHASELOCK */
	l = FLD_MOD(l, 0, 8, 8);	/* DSI_PLL_DRIFTGUARDEN */
	l = FLD_MOD(l, 0, 10, 9);	/* DSI_PLL_LOCKSEL */
	l = FLD_MOD(l, 1, 13, 13);	/* DSI_PLL_REFEN */
	l = FLD_MOD(l, 1, 14, 14);	/* DSIPHY_CLKINEN */
	l = FLD_MOD(l, 0, 15, 15);	/* DSI_BYPASSEN */
	l = FLD_MOD(l, 1, 16, 16);	/* DSS_CLOCK_EN */
	l = FLD_MOD(l, 0, 17, 17);	/* DSS_CLOCK_PWDN */
	l = FLD_MOD(l, 1, 18, 18);	/* DSI_PROTO_CLOCK_EN */
	l = FLD_MOD(l, 0, 19, 19);	/* DSI_PROTO_CLOCK_PWDN */
	l = FLD_MOD(l, 0, 20, 20);	/* DSI_HSDIVBYPASS */
	__raw_writel(l, DSI_PLL_CONFIGURATION2);
	return 0;
}

static int dsi_cio_wait_tx_clk_esc_reset(void)
{
	int t, i;
	u8 in_use[5];
	static const u8 offsets_new[] = { 24, 25, 26, 27, 28 };
	const u8 *offsets;

	dsi.num_lanes_supported = 5;
	offsets = offsets_new;

	in_use[0] = 1;
	in_use[1] = 1;
	in_use[2] = 1;
	in_use[3] = 0;
	in_use[4] = 0;
	
	t = 100000;
	while (1) {
		u32 l;
		int ok;

		l = __raw_readl(DSI_DSIPHY_CFG5);
		ok = 0;
		for (i = 0; i < dsi.num_lanes_supported; ++i) {
			if (!in_use[i] || (l & (1 << offsets[i])))
				ok++;
		}

		if (ok == dsi.num_lanes_supported)
			break;

		if (--t == 0) {
			for (i = 0; i < dsi.num_lanes_supported; ++i) {
				if (!in_use[i] || (l & (1 << offsets[i])))
					continue;

				printf("CIO TXCLKESC%d domain not coming " \
						"out of reset\n", i);
			}
			return -1;
		}
	}

	return 0;
}

static inline void dsi_cio_timings(void)
{
	__raw_writel(0x1a3d1a31, DSI_DSIPHY_CFG0);
	
	__raw_writel(0x42091757, DSI_DSIPHY_CFG1);

	__raw_writel(0xb8000016, DSI_DSIPHY_CFG2);
}

static inline void dsi_set_lane_config(void)
{

	u32 r;

	r = __raw_readl(DSI_COMPLEXIO_CFG1);
	r = 0x321;
	__raw_writel(r, DSI_COMPLEXIO_CFG1);

	return 0;
}

enum dsi_cio_power_state {
	DSI_COMPLEXIO_POWER_OFF		= 0x0,
	DSI_COMPLEXIO_POWER_ON		= 0x1,
	DSI_COMPLEXIO_POWER_ULPS	= 0x2,
};
static int dsi_cio_power(enum dsi_cio_power_state state)
{
	int t = 0;

	/* PWR_CMD */
	REG_FLD_MOD(DSI_COMPLEXIO_CFG1, state, 28, 27);
	/* PWR_STATUS */
	while (FLD_GET(__raw_readl(DSI_COMPLEXIO_CFG1), 26, 25) != state) {
		if (++t > 1000) {
			//printf("failed to set complexio power state to %d\n", state);
			return -1;
		}
		udelay(10);
	}

	return 0;
}
static inline int dsi_if_enable(u8 enable)
{
	enable = enable ? 1 : 0;
	REG_FLD_MOD(DSI_CTRL, enable, 0, 0); /* IF_EN */
	if (wait_for_bit_change(DSI_CTRL, 0, enable) != enable) {
			//printf("Failed to set dsi_if_enable to %d\n", enable);
			return -1;
	}
	return 0;
}
static int dsi_force_tx_stop_mode_io(void)
{
	u32 r;

	r = __raw_readl(DSI_TIMING1);
	r = FLD_MOD(r, 1, 15, 15);	/* FORCE_TX_STOP_MODE_IO */
	__raw_writel(r, DSI_TIMING1);

	if (wait_for_bit_change(DSI_TIMING1, 15, 0) != 0) {
		printf("TX_STOP bit not going down\n");
		return -1;
	}
	return 0;
}
static int omap4_dsi_mux_pads(int dsi_id, unsigned lanes)
{
	u32 enable_mask, enable_shift;
	u32 pipd_mask, pipd_shift;
	u32 reg;
	
#define OMAP4_DSI1_LANEENABLE_SHIFT				24
#define OMAP4_DSI1_LANEENABLE_MASK				(0x1f << 24)
#define OMAP4_DSI1_PIPD_SHIFT					19
#define OMAP4_DSI1_PIPD_MASK					(0x1f << 19)
	enable_mask = OMAP4_DSI1_LANEENABLE_MASK;
	enable_shift = OMAP4_DSI1_LANEENABLE_SHIFT;
	pipd_mask = OMAP4_DSI1_PIPD_MASK;
	pipd_shift = OMAP4_DSI1_PIPD_SHIFT;

	reg = __raw_readl(OMAP44XX_CONTROL_DSIPHY);
	reg &= ~enable_mask;
	reg &= ~pipd_mask;

	reg |= (lanes << enable_shift) & enable_mask;
	reg |= (lanes << pipd_shift) & pipd_mask;

	__raw_writel(reg, OMAP44XX_CONTROL_DSIPHY);
	return 0;
}

static void dss_dsi_enable_pads(int dsi_id, unsigned lane_mask)
{
	omap4_dsi_mux_pads(dsi_id, lane_mask);
}
static void omap_dsi_disable_pads(int dsi_id)
{
	omap4_dsi_mux_pads(dsi_id, 0);
}

static int dsi_cio_init(void)
{
	int r;
	u32 l;

	dss_dsi_enable_pads(0, (1<<2) | (1<<1) | (1<<0));//use DSI1 lane 0 1 and 2

	/* A dummy read using the SCP interface to any DSIPHY register is
	 * required after DSIPHY reset to complete the reset of the DSI complex
	 * I/O. */
	l = __raw_readl(DSI_DSIPHY_CFG5);
	
	if (wait_for_bit_change(DSI_DSIPHY_CFG5, 30, 1) != 1) {
		printf("CIO SCP Clock domain not coming out of reset.\n");
		r = -1;
		return r;
	}
	dsi_set_lane_config();

	/* set TX STOP MODE timer to maximum for this operation */
	l = __raw_readl(DSI_TIMING1);
	l = FLD_MOD(l, 1, 15, 15);	/* FORCE_TX_STOP_MODE_IO */
	l = FLD_MOD(l, 1, 14, 14);	/* STOP_STATE_X16_IO */
	l = FLD_MOD(l, 1, 13, 13);	/* STOP_STATE_X4_IO */
	l = FLD_MOD(l, 0x1fff, 12, 0);	/* STOP_STATE_COUNTER_IO */
	__raw_writel(l, DSI_TIMING1);

	r = dsi_cio_power(DSI_COMPLEXIO_POWER_ON);
	if (r)
		return r;

	if (wait_for_bit_change(DSI_COMPLEXIO_CFG1, 29, 1) != 1) {
		printf("CIO PWR clock domain not coming out of reset.\n");
		r = -1;
		return r;
	}

	dsi_if_enable(1);
	dsi_if_enable(0);
	REG_FLD_MOD(DSI_CLK_CTRL, 1, 20, 20); /* LP_CLK_ENABLE */
	r = dsi_cio_wait_tx_clk_esc_reset();
	if (r)
		return r;

	/* FORCE_TX_STOP_MODE_IO */
	REG_FLD_MOD(DSI_TIMING1, 0, 15, 15);
	dsi_cio_timings();

	/* DDR_CLK_ALWAYS_ON */
	REG_FLD_MOD(DSI_CLK_CTRL, 1, 13, 13);

	return 0;
}
static void dsi_cio_uninit(void)
{
	REG_FLD_MOD(DSI_CLK_CTRL, 0, 13, 13);

	dsi_cio_power(DSI_COMPLEXIO_POWER_OFF);
	dsi_disable_scp_clk();
	omap_dsi_disable_pads(0);
}
static inline void dsi_proto_timings(void)
{
	__raw_writel(0x2515, DSI_CLK_TIMING);

	__raw_writel(0x170016, DSI_VM_TIMING7);

	__raw_writel(0x1401e, DSI_VM_TIMING1);

	__raw_writel(0x414120a, DSI_VM_TIMING2);

	__raw_writel(0x4e701e0, DSI_VM_TIMING3);

	__raw_writel(0, DSI_VM_TIMING4);

	__raw_writel(0, DSI_VM_TIMING5);

	__raw_writel(0x2500011, DSI_VM_TIMING6);
}
static inline int dsi_set_lp_clk_divisor(void)
{
	unsigned lp_clk_div = 9;

	/* LP_CLK_DIVISOR */
	REG_FLD_MOD(DSI_CLK_CTRL, lp_clk_div, 12, 0);

	/* LP_RX_SYNCHRO_ENABLE */
	REG_FLD_MOD(DSI_CLK_CTRL, 1, 21, 21);
	return 0;
}
static void dsi_config_tx_fifo(void)
{
	u32 r = 0;
	int add = 0;
	int i;

	dsi.vc[0].fifo_size = dsi.vc[1].fifo_size = dsi.vc[2].fifo_size = dsi.vc[3].fifo_size = DSI_FIFO_SIZE_32;

	for (i = 0; i < 4; i++) {
		u8 v;
		int size = dsi.vc[i].fifo_size;

		if (add + size > 4) {
			printf("Illegal FIFO configuration\n");
		}

		v = FLD_VAL(add, 2, 0) | FLD_VAL(size, 7, 4);
		r |= v << (8 * i);
		/*printf("TX FIFO vc %d: size %d, add %d\n", i, size, add); */
		add += size;
	}
	__raw_writel(r, DSI_TX_FIFO_VC_SIZE);
}

static void dsi_config_rx_fifo(void)
{
	u32 r = 0;
	int add = 0;
	int i;
	
	dsi.vc[0].fifo_size = dsi.vc[1].fifo_size = dsi.vc[2].fifo_size = dsi.vc[3].fifo_size = DSI_FIFO_SIZE_32;

	for (i = 0; i < 4; i++) {
		u8 v;
		int size = dsi.vc[i].fifo_size;

		if (add + size > 4) {
			printf("Illegal FIFO configuration\n");
		}

		v = FLD_VAL(add, 2, 0) | FLD_VAL(size, 7, 4);
		r |= v << (8 * i);
		/*printf("RX FIFO vc %d: size %d, add %d\n", i, size, add); */
		add += size;
	}
	__raw_writel(r, DSI_RX_FIFO_VC_SIZE);
}
static inline void dsi_set_stop_state_counter(unsigned ticks, u8 x4, u8 x16)
{
	u32 r;

	r = __raw_readl(DSI_TIMING1);
	r = FLD_MOD(r, 1, 15, 15);	/* FORCE_TX_STOP_MODE_IO */
	r = FLD_MOD(r, x16 ? 1 : 0, 14, 14);	/* STOP_STATE_X16_IO */
	r = FLD_MOD(r, x4 ? 1 : 0, 13, 13);	/* STOP_STATE_X4_IO */
	r = FLD_MOD(r, ticks, 12, 0);	/* STOP_STATE_COUNTER_IO */
	__raw_writel(r, DSI_TIMING1);
}
static inline void dsi_set_ta_timeout(unsigned ticks, u8 x8, u8 x16)
{
	unsigned long fck;
	u32 r;

	r = __raw_readl(DSI_TIMING1);
	r = FLD_MOD(r, 1, 31, 31);	/* TA_TO */
	r = FLD_MOD(r, x16 ? 1 : 0, 30, 30);	/* TA_TO_X16 */
	r = FLD_MOD(r, x8 ? 1 : 0, 29, 29);	/* TA_TO_X8 */
	r = FLD_MOD(r, ticks, 28, 16);	/* TA_TO_COUNTER */
	__raw_writel(r, DSI_TIMING1);
}
static inline void dsi_set_lp_rx_timeout(unsigned ticks, u8 x4, u8 x16)
{
	u32 r;

	r = __raw_readl(DSI_TIMING2);
	r = FLD_MOD(r, 1, 15, 15);	/* LP_RX_TO */
	r = FLD_MOD(r, x16 ? 1 : 0, 14, 14);	/* LP_RX_TO_X16 */
	r = FLD_MOD(r, x4 ? 1 : 0, 13, 13);	/* LP_RX_TO_X4 */
	r = FLD_MOD(r, ticks, 12, 0);	/* LP_RX_COUNTER */
	__raw_writel(r, DSI_TIMING2);
}
static inline void dsi_set_hs_tx_timeout(unsigned ticks, u8 x4, u8 x16)
{
	u32 r;

	r = __raw_readl(DSI_TIMING2);
	r = FLD_MOD(r, 1, 31, 31);	/* HS_TX_TO */
	r = FLD_MOD(r, x16 ? 1 : 0, 30, 30);	/* HS_TX_TO_X16 */
	r = FLD_MOD(r, x4 ? 1 : 0, 29, 29);	/* HS_TX_TO_X8 (4 really) */
	r = FLD_MOD(r, ticks, 28, 16);	/* HS_TX_TO_COUNTER */
	__raw_writel(r, DSI_TIMING2);
}
static inline void dsi_config_vp_num_line_buffers(void)
{
	dsi.num_line_buffers = 2;

	/* LINE_BUFFER */
	REG_FLD_MOD(DSI_CTRL, dsi.num_line_buffers, 13, 12);
}
static inline void dsi_config_vp_sync_events(void)
{
	u32 r = 0xaee9e;
	__raw_writel(r, DSI_CTRL);
}
static inline void dsi_config_blanking_modes(void)
{
	u32 r = 0xeaee9e;
	__raw_writel(r, DSI_CTRL);
}
static inline void dsi_vc_initial_config(int channel)
{
	u32 r;

	r = __raw_readl(DSI_VC_CTRL(channel));

	if (FLD_GET(r, 15, 15)) /* VC_BUSY */
		printf("VC(%d) busy when trying to configure it!\n",
				channel);

	r = FLD_MOD(r, 0, 1, 1); /* SOURCE, 0 = L4 */
	r = FLD_MOD(r, 0, 2, 2); /* BTA_SHORT_EN  */
	r = FLD_MOD(r, 0, 3, 3); /* BTA_LONG_EN */
	r = FLD_MOD(r, 0, 4, 4); /* MODE, 0 = command */
	r = FLD_MOD(r, 1, 7, 7); /* CS_TX_EN */
	r = FLD_MOD(r, 1, 8, 8); /* ECC_TX_EN */
	r = FLD_MOD(r, 0, 9, 9); /* MODE_SPEED, high speed on/off */
	r = FLD_MOD(r, 3, 11, 10);	/* OCP_WIDTH = 32 bit */

	r = FLD_MOD(r, 4, 29, 27); /* DMA_RX_REQ_NB = no dma */
	r = FLD_MOD(r, 4, 23, 21); /* DMA_TX_REQ_NB = no dma */

	__raw_writel(r, DSI_VC_CTRL(channel));
}
static int dsi_proto_config(void)
{
	u32 r;
	int buswidth = 0;

	dsi_config_tx_fifo();

	dsi_config_rx_fifo();

	/* XXX what values for the timeouts? */
	dsi_set_stop_state_counter(0x1000, 0, 0);
	dsi_set_ta_timeout(0x1fff, 1, 1);
	dsi_set_lp_rx_timeout(0x1fff, 1, 1);
	dsi_set_hs_tx_timeout(0x1fff, 1, 1);

	buswidth = 2;
	r = __raw_readl(DSI_CTRL);
	r = FLD_MOD(r, 1, 1, 1);	/* CS_RX_EN */
	r = FLD_MOD(r, 1, 2, 2);	/* ECC_RX_EN */
	r = FLD_MOD(r, 1, 3, 3);	/* TX_FIFO_ARBITRATION */
	r = FLD_MOD(r, 1, 4, 4);	/* VP_CLK_RATIO, always 1, see errata*/
	r = FLD_MOD(r, buswidth, 7, 6); /* VP_DATA_BUS_WIDTH */
	r = FLD_MOD(r, 0, 8, 8);	/* VP_CLK_POL */
	r = FLD_MOD(r, 1, 14, 14);	/* TRIGGER_RESET_MODE */
	r = FLD_MOD(r, 1, 19, 19);	/* EOT_ENABLE */
	__raw_writel(r, DSI_CTRL);

	dsi_config_vp_num_line_buffers();

	dsi_config_vp_sync_events();
	dsi_config_blanking_modes();

	dsi_vc_initial_config(0);
	dsi_vc_initial_config(1);
	dsi_vc_initial_config(2);
	dsi_vc_initial_config(3);

	return 0;
}
static int dsi_vc_enable(int channel, u8 enable)
{
	enable = enable ? 1 : 0;
	REG_FLD_MOD(DSI_VC_CTRL(channel), enable, 0, 0);
	if (wait_for_bit_change(DSI_VC_CTRL(channel),
		0, enable) != enable) {
			printf("Failed to set dsi_vc_enable to %d\n", enable);
			return -1;
	}
	return 0;
}
static void dsi_wait_pll_hsdiv_dispc_active(void)
{
	if (wait_for_bit_change(DSI_PLL_STATUS, 7, 1) != 1)
		printf("DSI_PLL_HSDIV_DISPC not active\n");
}
static void dsi_wait_pll_hsdiv_dsi_active(void)
{
	if (wait_for_bit_change(DSI_PLL_STATUS, 8, 1) != 1)
		printf("DSI_PLL_HSDIV_DSI not active\n");
}

static void dss_select_dispc_clk_source(void)
{
	int b;
	u8 start, end;

	b = 1;
	start = 9;
	end = 8;
	dsi_wait_pll_hsdiv_dispc_active();
	
	REG_FLD_MOD(DSS_CONTROL, b, start, end);	/* DISPC_CLK_SWITCH */
}
static void dss_select_dsi_clk_source(void)
{
	int b;

	b = 1;
	dsi_wait_pll_hsdiv_dsi_active();

	REG_FLD_MOD(DSS_CONTROL, b, 1, 1);	/* DSI_CLK_SWITCH */
}

static void dss_select_lcd_clk_source(void)
{
	int b, ix, pos;

	b = 1;
	pos = 0;
	dsi_wait_pll_hsdiv_dispc_active();

	REG_FLD_MOD(DSS_CONTROL, b, pos, pos);	/* LCDx_CLK_SWITCH */
}
static inline void dispc_mgr_set_lcd_divisor(void)
{
	__raw_writel(FLD_VAL(LCK_DIV, 23, 16) | FLD_VAL(PCK_DIV, 7, 0), DISPC_DIVISORo(OMAP_DSS_CHANNEL_LCD));	
}
static int dsi_pll_power(u8 state)
{
	int t = 0;
	
	/* PLL_PWR_CMD */
	REG_FLD_MOD(DSI_CLK_CTRL, state, 31, 30);
	/* PLL_PWR_STATUS */
	while (FLD_GET(__raw_readl(DSI_CLK_CTRL), 29, 28) != state) {
		if (++t > 1000) {
			printf("Failed to set DSI PLL power mode to %d\n", state);
			return -1;
		}
		udelay(10);
	}
	return 0;
}
static int dsi_pll_init(void)
{
	int r = 0;
	
	dsi_enable_pll_clock();
	dsi_enable_scp_clk();
	
	if (wait_for_bit_change(DSI_PLL_STATUS, 0, 1) != 1) {
		printf("PLL not coming out of reset.\n");
		r = -1;
		return r;
	}

	r = dsi_pll_power(DSI_PLL_POWER_ON_ALL);

	return r;
}
static void dsi_pll_uninit(void)
{
	dsi_pll_power(DSI_PLL_POWER_OFF);

	dsi_disable_scp_clk();
	dsi_disable_pll_clock();
}

static int dsi_display_init_dsi(void)
{
	int r;

	r = dsi_pll_init();
	if (r)
		return r;
	
	r = dsi_pll_set_clock_div();
	if (r)
		return r;
	
	dss_select_dispc_clk_source();
	dss_select_dsi_clk_source();
	dss_select_lcd_clk_source();

	dispc_mgr_set_lcd_divisor();

	r = dsi_cio_init();
	if (r)
		return r;

	dsi_proto_timings();

	dsi_set_lp_clk_divisor();
	
	r = dsi_proto_config();
	if (r)
		return r;

	/* enable interface */
	dsi_vc_enable(0, 1);
	dsi_vc_enable(1, 1);
	dsi_vc_enable(2, 1);
	dsi_vc_enable(3, 1);
	dsi_if_enable(1);
	dsi_force_tx_stop_mode_io();

	return 0;
}

static void dsi_display_uninit_dsi(void)
{
	/* disable interface */
	dsi_if_enable(0);
	dsi_vc_enable(0, 0);
	dsi_vc_enable(1, 0);
	dsi_vc_enable(2, 0);
	dsi_vc_enable(3, 0);

	dsi_cio_uninit();
	dsi_pll_uninit();
}

static inline void dsi_display_init_dispc(void)
{
	REG_FLD_MOD(DISPC_CONFIG, 0, 16, 16);
	__raw_writel(((DISPC_HBP-1 << 20) | (DISPC_HFP-1 << 8) | (DISPC_HSW-1)), DISPC_TIMING_H(OMAP_DSS_CHANNEL_LCD));
	__raw_writel(((DISPC_VBP << 20) | (DISPC_VFP << 8) | (DISPC_VSW-1)), DISPC_TIMING_V(OMAP_DSS_CHANNEL_LCD));
	__raw_writel((((PANEL_HEIGHT-1) << 16) | (PANEL_WIDTH-1)), DISPC_SIZE_MGR(OMAP_DSS_CHANNEL_LCD));
	__raw_writel(0x00000308, DISPC_CONTROL);
}


static int omapdss_dsi_display_enable(void)
{
	int r = 0;
	
	dsi_enable_pll_clock();
	dsi_display_init_dispc();
	r = dsi_display_init_dsi();
	
	return r;
}

static void omapdss_dsi_display_disable(void)
{
	dsi_display_uninit_dsi();
	dsi_disable_pll_clock();
}
static inline void dsi_vc_write_long_header(int channel, u8 data_type, u16 len, u8 ecc)
{
	u32 val;
	u8 data_id;
	dsi.vc[channel].vc_id = 0;
	
	data_id = data_type | dsi.vc[channel].vc_id << 6;

	val = FLD_VAL(data_id, 7, 0) | FLD_VAL(len, 23, 8) |
		FLD_VAL(ecc, 31, 24);
	__raw_writel(val, DSI_VC_LONG_PACKET_HEADER(channel));
}

static inline void dsi_vc_write_long_payload(int channel, u8 b1, u8 b2, u8 b3, u8 b4)
{
	u32 val;

	val = b4 << 24 | b3 << 16 | b2 << 8  | b1 << 0;
	__raw_writel(val, DSI_VC_LONG_PACKET_PAYLOAD(channel));
}
static int dsi_vc_send_long(int channel,
		u8 data_type, u8 *data, u16 len, u8 ecc)
{
	int i;
	u8 *p;
	int r = 0;
	u8 b1, b2, b3, b4;

	/* len + header */
	if (dsi.vc[channel].fifo_size * 32 * 4 < len + 4) {
		printf("unable to send long packet: packet too long.\n");
		return -1;
	}

	dsi_vc_write_long_header(channel, data_type, len, ecc);

	p = data;
	for (i = 0; i < len >> 2; i++) {
		printf("\tsending full packet %d\n", i);

		b1 = *p++;
		b2 = *p++;
		b3 = *p++;
		b4 = *p++;
		dsi_vc_write_long_payload(channel, b1, b2, b3, b4);
	}

	i = len % 4;
	if (i) {
		b1 = 0; b2 = 0; b3 = 0;

		printf("\tsending remainder bytes %d\n", i);
		switch (i) {
		case 3:
			b1 = *p++;
			b2 = *p++;
			b3 = *p++;
			break;
		case 2:
			b1 = *p++;
			b2 = *p++;
			break;
		case 1:
			b1 = *p++;
			break;
		}
		dsi_vc_write_long_payload(channel, b1, b2, b3, 0);
	}
	return r;
}
static inline int dsi_vc_send_short(int channel,
		u8 data_type, u16 data, u8 ecc)
{
	u32 r;
	u8 data_id;

	if (FLD_GET(__raw_readl(DSI_VC_CTRL(channel)), 16, 16)) {
		printf("ERROR FIFO FULL, aborting transfer\n");
		return -1;
	}

	data_id = data_type | dsi.vc[channel].vc_id << 6;

	r = (data_id << 0) | (data << 8) | (ecc << 24);
	__raw_writel(r, DSI_VC_SHORT_PACKET_HEADER(channel));
	return 0;
}
static int dsi_vc_send_null(int channel)
{
	return dsi_vc_send_long(channel, MIPI_DSI_NULL_PACKET, NULL,
		0, 0);
}
static void omapdss_dsi_vc_enable_hs(int channel, u8 enable)
{
	dsi_vc_enable(channel, 0);
	dsi_if_enable(0);

	REG_FLD_MOD(DSI_VC_CTRL(channel), enable, 9, 9);
	dsi_vc_enable(channel, 1);
	dsi_if_enable(1);

	dsi_force_tx_stop_mode_io();

	/* start the DDR clock by sending a NULL packet */
	if (enable)
		dsi_vc_send_null(channel);
}

static int dsi_vc_set_max_rx_packet_size(int channel, u16 len)
{
	return dsi_vc_send_short(channel,
			MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, len, 0);
}

static void dispc_enable_lcd_out(enum omap_channel channel, u8 enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD2) {
		REG_FLD_MOD(DISPC_CONTROL2, enable ? 1 : 0, 0, 0);
		/* flush posted write */
		__raw_readl(DISPC_CONTROL2);
	} else {
		REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 0, 0);
		__raw_readl(DISPC_CONTROL);
	}
}

static void dispc_mgr_go(void)
{
	int bit;
	u8 enable_bit, go_bit;

	bit = 0; /* LCDENABLE */
	/* if the channel is not enabled, we don't need GO */
	enable_bit = REG_GET(DISPC_CONTROL, bit, bit) == 1;
	if (!enable_bit)
		return;
	
	bit = 5; /* GOLCD */
	go_bit = REG_GET(DISPC_CONTROL, bit, bit) == 1;
	if (go_bit) {
		printf("GO bit not down\n");
		return;
	}
		
	REG_FLD_MOD(DISPC_CONTROL, 1, bit, bit);
}

static void dispc_mgr_enable_lcd_out(enum omap_channel channel, u8 enable)
{
	dispc_enable_lcd_out(channel, enable);
}

static void dsi_enable_video_output(int channel)
{
	u8 data_type;
	u16 word_count;
	int r;

	data_type = MIPI_DSI_PACKED_PIXEL_STREAM_24;

	dsi_if_enable(0);
	dsi_vc_enable(channel, 0);

	/* MODE, 1 = video mode */
	REG_FLD_MOD(DSI_VC_CTRL(channel), 1, 4, 4);

	word_count = DIV_ROUND_UP(PANEL_WIDTH * BPP, 8);
	dsi_vc_write_long_header(channel, data_type,
			word_count, 0);

	dsi_vc_enable(channel, 1);
	dsi_if_enable(1);
	dispc_mgr_enable_lcd_out(0, 1);
	dispc_mgr_go();
}

static void dsi_disable_video_output(int channel)
{
	dsi_if_enable(0);
	dsi_vc_enable(channel, 0);

	/* MODE, 0 = command mode */
	REG_FLD_MOD(DSI_VC_CTRL(channel), 0, 4, 4);

	dsi_vc_enable(channel, 1);
	dsi_if_enable(1);
	dispc_mgr_enable_lcd_out(0, 0);
}

static inline void dispc_set_loadmode(enum omap_dss_load_mode mode)
{
	REG_FLD_MOD(DISPC_CONFIG, mode, 2, 1);
}
static void _omap_dispc_initial_config(void)
{
	u32 l;

	/* Exclusively enable DISPC_CORE_CLK and set divider to 1 */
	l = __raw_readl(DISPC_DIVISOR);
	/* Use DISPC_DIVISOR.LCD, instead of DISPC_DIVISOR1.LCD */
	l = FLD_MOD(l, 1, 0, 0);
	l = FLD_MOD(l, 1, 23, 16);
	__raw_writel(l, DISPC_DIVISOR);

	REG_FLD_MOD(DISPC_CONFIG, 1, 17, 17);
	dispc_set_loadmode(OMAP_DSS_LOAD_FRAME_ONLY);
}
static void loading_logo()
{
	unsigned char pixel[3];
	int i;

	memset(FRAME_ADDR, 0, logo_width*logo_height*4);
	for (i = 0; i < logo_width*logo_height*4; i+=4) {
		HEADER_PIXEL(header_data, pixel);
		*((unsigned char *)(FRAME_ADDR + i)) = pixel[2];
		*((unsigned char *)(FRAME_ADDR + i+1)) = pixel[1];
		*((unsigned char *)(FRAME_ADDR + i+2)) = pixel[0];
		*((unsigned char *)(FRAME_ADDR + i+3)) = 0x00;
	}
	/* Configure GFX pipeline */
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA0);
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA1);
	__raw_writel(0x12000098, DISPC_GFX_ATTRIBUTES);
	__raw_writel(((((PANEL_HEIGHT-logo_height)/2) << 16) | ((PANEL_WIDTH-logo_width)/2)), DISPC_GFX_POSITION);
	__raw_writel((((logo_height-1) << 16) | (logo_width-1)), DISPC_GFX_SIZE);
	/* Enable GFX pipeline */
	__raw_writel(__raw_readl(DISPC_GFX_ATTRIBUTES) | 0x1, DISPC_GFX_ATTRIBUTES);
}

static void loading_low_battery_image(int img_index)
{
	unsigned char pixel[3];
	char * p_bat_img;
	char * p_mem_img;
	int i, blank_l, blank_r, pix_pos_in_line, bat_level_height;

	memset(FRAME_ADDR, 0, bat_img_width*bat_img_height*4);
	p_bat_img = low_battery_0_image_data;
	for (i = 0; i < bat_img_width*bat_img_height*4; i+=4) {
		HEADER_PIXEL(p_bat_img, pixel);
		*((unsigned char *)(FRAME_ADDR + i)) = pixel[2];
		*((unsigned char *)(FRAME_ADDR + i+1)) = pixel[1];
		*((unsigned char *)(FRAME_ADDR + i+2)) = pixel[0];
		*((unsigned char *)(FRAME_ADDR + i+3)) = 0x00;
	}
	/*fill battery level retangle, color is:5A2B2B*/
	if(img_index > 0){
		pixel[0] = 0xA5;
		pixel[1] = 0x2B;
		pixel[2] = 0x2B;

		bat_level_height = img_index * level_5_percentage_height;
		blank_l = (bat_img_width -bat_level_width)/2;
		blank_r = bat_img_width -bat_level_width - blank_l;
		p_mem_img = (unsigned char *)FRAME_ADDR + bat_img_width*(bat_img_height - 25 - bat_level_height)*4;
		for (i = 0; i < bat_img_width*bat_level_height*4; i+=4) {
			pix_pos_in_line = (i/4)%bat_img_width;
			if(blank_l <= pix_pos_in_line && pix_pos_in_line < (bat_img_width - blank_r)){
				*((unsigned char *)(p_mem_img + i)) = pixel[2];
				*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
				*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
				*((unsigned char *)(p_mem_img + i+3)) = 0x00;
			}
		}
	}

	/* Configure GFX pipeline */
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA0);
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA1);
	__raw_writel(0x12000098, DISPC_GFX_ATTRIBUTES);
	__raw_writel(((((PANEL_HEIGHT-bat_img_height)/2) << 16) | ((PANEL_WIDTH-bat_img_width)/2)), DISPC_GFX_POSITION);
	__raw_writel((((bat_img_height-1) << 16) | (bat_img_width-1)), DISPC_GFX_SIZE);
	/* Enable GFX pipeline */
	__raw_writel(__raw_readl(DISPC_GFX_ATTRIBUTES) | 0x1, DISPC_GFX_ATTRIBUTES);
}

static int number_width[] = {
	23, 		/* 0 */
	13, 		/* 1 */
	22, 		/* 2 */
	23, 		/* 3 */
	26, 		/* 4 */
	21, 		/* 5 */
	23, 		/* 6 */
	23, 		/* 7 */
	22, 		/* 8 */
	23, 		/* 9 */
	35, 		/* % */
};
static int word_space[] = {
	3, 		/* 0 and %*/
	12, 		/* 1 and %*/
	5, 		/* 2 and %*/
	5, 		/* 3 and %*/
	3, 		/* 4 and %*/
	5, 		/* 5 and %*/
	4, 		/* 6 and %*/
	4, 		/* 7 and %*/
	5, 		/* 8 and %*/
	4, 		/* 9 and %*/
	12, 		/* 1 and 0*/
	14, 		/* 1 and 1*/
	12, 		/* 1 and 2*/
	11, 		/* 1 and 3*/
	10, 		/* 1 and 4*/
	13, 		/* 1 and 5*/
	12, 		/* 1 and 6*/
	12, 		/* 1 and 7*/
	12, 		/* 1 and 8*/
	12, 		/* 1 and 9*/
	5, 		/* 2 and 0*/
	7, 		/* 2 and 1*/
	5, 		/* 2 and 2*/
	4, 		/* 2 and 3*/
	3, 		/* 2 and 4*/
	6, 		/* 2 and 5*/
	5, 		/* 2 and 6*/
	5, 		/* 2 and 7*/
	5, 		/* 2 and 8*/
	5, 		/* 2 and 9*/
	5, 		/* 3 and 0*/
// the following numbers are not confirmed yet
	7, 		/* 3 and 1*/
	5, 		/* 3 and 2*/
	4, 		/* 3 and 3*/
	3, 		/* 3 and 4*/
	6, 		/* 3 and 5*/
	5, 		/* 3 and 6*/
	5, 		/* 3 and 7*/
	5, 		/* 3 and 8*/
	5, 		/* 3 and 9*/
	2, 		/* 4 and 0*/
	4, 		/* 4 and 1*/
	2, 		/* 4 and 2*/
	1, 		/* 4 and 3*/
	0, 		/* 4 and 4*/
	3, 		/* 4 and 5*/
	2, 		/* 4 and 6*/
	2, 		/* 4 and 7*/
	2, 		/* 4 and 8*/
	2, 		/* 4 and 9*/
	6, 		/* 5 and 0*/
	8, 		/* 5 and 1*/
	6, 		/* 5 and 2*/
	5, 		/* 5 and 3*/
	4, 		/* 5 and 4*/
	7, 		/* 5 and 5*/
	6, 		/* 5 and 6*/
	6, 		/* 5 and 7*/
	6, 		/* 5 and 8*/
	6, 		/* 5 and 9*/
	5, 		/* 6 and 0*/
	7, 		/* 6 and 1*/
	5, 		/* 6 and 2*/
	4, 		/* 6 and 3*/
	3, 		/* 6 and 4*/
	6, 		/* 6 and 5*/
	5, 		/* 6 and 6*/
	5, 		/* 6 and 7*/
	5, 		/* 6 and 8*/
	5, 		/* 6 and 9*/
	5, 		/* 7 and 0*/
	7, 		/* 7 and 1*/
	5, 		/* 7 and 2*/
	4, 		/* 7 and 3*/
	3, 		/* 7 and 4*/
	6, 		/* 7 and 5*/
	5, 		/* 7 and 6*/
	5, 		/* 7 and 7*/
	5, 		/* 7 and 8*/
	5, 		/* 7 and 9*/
	5, 		/* 8 and 0*/
	7, 		/* 8 and 1*/
	5, 		/* 8 and 2*/
	4, 		/* 8 and 3*/
	3, 		/* 8 and 4*/
	6, 		/* 8 and 5*/
	5, 		/* 8 and 6*/
	5, 		/* 8 and 7*/
	5, 		/* 8 and 8*/
	5, 		/* 8 and 9*/
	5, 		/* 9 and 0*/
	7, 		/* 9 and 1*/
	5, 		/* 9 and 2*/
	4, 		/* 9 and 3*/
	3, 		/* 9 and 4*/
	6, 		/* 9 and 5*/
	5, 		/* 9 and 6*/
	5, 		/* 9 and 7*/
	5, 		/* 9 and 8*/
	5, 		/* 9 and 9*/
	5, 		/* 0 and 0*/
};
static void loading_charging_battery_image(int bat_index, int percentage_index)
{
	unsigned char pixel[3];
	char * p_mem_img;
	char * p_bat_img;
	char * p_character_1_img;
	char * p_character_2_img;
	char * p_character_3_img;
	char * p_character_4_img;
	char * p_txt_img;
	int i, blank_l, blank_r, pix_pos_in_line, bat_level_height;
	int character_1_width, character_2_width, character_3_width, character_4_width, word_space1, word_space2, word_space3;
	int percentage_part_width, img_width, img_height;

	if(percentage_index < 10){
		character_1_width = number_width[percentage_index];
		character_2_width = number_width[10];/* % */
		character_3_width = 0;
		character_4_width = 0;
		word_space1 = word_space[percentage_index];
		word_space2 = 0;
		word_space3 = 0;
		switch(percentage_index){
		case 0:
			p_character_1_img = character_0_image_data;
			break;
		case 1:
			p_character_1_img = character_1_image_data;
			break;
		case 2:
			p_character_1_img = character_2_image_data;
			break;
		case 3:
			p_character_1_img = character_3_image_data;
			break;
		case 4:
			p_character_1_img = character_4_image_data;
			break;
		case 5:
			p_character_1_img = character_5_image_data;
			break;
		case 6:
			p_character_1_img = character_6_image_data;
			break;
		case 7:
			p_character_1_img = character_7_image_data;
			break;
		case 8:
			p_character_1_img = character_8_image_data;
			break;
		case 9:
			p_character_1_img = character_9_image_data;
			break;
		}
		p_character_2_img = character_p_image_data;/* % */
		p_character_3_img = NULL;
		p_character_4_img = NULL;
	}else if (percentage_index == 100){
		character_1_width = number_width[1];
		character_2_width = number_width[0];
		character_3_width = number_width[0];
		character_4_width = number_width[10];/* % */
		word_space1 = word_space[10];
		word_space2 = word_space[100];
		word_space3 = word_space[0];
		p_character_1_img = character_1_image_data;
		p_character_2_img = character_0_image_data;
		p_character_3_img = character_0_image_data;
		p_character_4_img = character_p_image_data;
	}else{
		character_1_width = number_width[percentage_index / 10];
		character_2_width = number_width[percentage_index % 10];
		character_3_width = number_width[10];/* % */
		character_4_width = 0;
		word_space1 = word_space[percentage_index];
		word_space2 = word_space[percentage_index % 10];
		word_space3 = 0;
		switch(percentage_index / 10){
		case 0:
			p_character_1_img = character_0_image_data;
			break;
		case 1:
			p_character_1_img = character_1_image_data;
			break;
		case 2:
			p_character_1_img = character_2_image_data;
			break;
		case 3:
			p_character_1_img = character_3_image_data;
			break;
		case 4:
			p_character_1_img = character_4_image_data;
			break;
		case 5:
			p_character_1_img = character_5_image_data;
			break;
		case 6:
			p_character_1_img = character_6_image_data;
			break;
		case 7:
			p_character_1_img = character_7_image_data;
			break;
		case 8:
			p_character_1_img = character_8_image_data;
			break;
		case 9:
			p_character_1_img = character_9_image_data;
			break;
		}
		switch(percentage_index % 10){
		case 0:
			p_character_2_img = character_0_image_data;
			break;
		case 1:
			p_character_2_img = character_1_image_data;
			break;
		case 2:
			p_character_2_img = character_2_image_data;
			break;
		case 3:
			p_character_2_img = character_3_image_data;
			break;
		case 4:
			p_character_2_img = character_4_image_data;
			break;
		case 5:
			p_character_2_img = character_5_image_data;
			break;
		case 6:
			p_character_2_img = character_6_image_data;
			break;
		case 7:
			p_character_2_img = character_7_image_data;
			break;
		case 8:
			p_character_2_img = character_8_image_data;
			break;
		case 9:
			p_character_2_img = character_9_image_data;
			break;
		}
		p_character_3_img = character_p_image_data;/* % */
		p_character_4_img = NULL;
	}
	percentage_part_width = character_1_width + word_space1 + character_2_width + word_space2 + character_3_width
		+ word_space3 + character_4_width;
	//img_width = bat_img_width > percentage_part_width ? bat_img_width : percentage_part_width;
	img_width = prompt_text_width;
	img_height = bat_img_height + 29 + percentage_img_height + 12 + prompt_text_height;

	memset(FRAME_ADDR, 0, img_width*img_height*4);

	/*fill battery image*/
	p_bat_img = charging_battery_0_image_data;
	if (percentage_index == 100)
		p_bat_img = low_battery_0_image_data;
	if(img_width == bat_img_width){
		for (i = 0; i < bat_img_width*bat_img_height*4; i+=4) {
			HEADER_PIXEL(p_bat_img, pixel);
			*((unsigned char *)(FRAME_ADDR + i)) = pixel[2];
			*((unsigned char *)(FRAME_ADDR + i+1)) = pixel[1];
			*((unsigned char *)(FRAME_ADDR + i+2)) = pixel[0];
			*((unsigned char *)(FRAME_ADDR + i+3)) = 0x00;
		}
	}else{
		blank_l = (img_width -bat_img_width)/2;
		blank_r = img_width - bat_img_width - blank_l;

		for (i = 0; i < img_width*bat_img_height*4; i+=4) {
			pix_pos_in_line = (i/4)%img_width;
			if(blank_l <= pix_pos_in_line && pix_pos_in_line < (img_width - blank_r)){
				HEADER_PIXEL(p_bat_img, pixel);
				*((unsigned char *)(FRAME_ADDR + i)) = pixel[2];
				*((unsigned char *)(FRAME_ADDR + i+1)) = pixel[1];
				*((unsigned char *)(FRAME_ADDR + i+2)) = pixel[0];
				*((unsigned char *)(FRAME_ADDR + i+3)) = 0x00;
			}
		}
	}

	/*fill battery level retangle, color is: FFFFFF or A52B2B*/
	if(bat_index > 0){
#if 1
		if (percentage_index>15) {
			pixel[0] = 0xFF;
			pixel[1] = 0xFF;
			pixel[2] = 0xFF;
		} else {
			pixel[0] = 0xa5;
			pixel[1] = 0x2b;
			pixel[2] - 0x2b;
		}
#else
		pixel[0] = (u8)(0xa5-((percentage_index>0x30)?(percentage_index>>4)-3:0)*0x20);
		pixel[1] = (u8)(0x2b+((percentage_index<0x30)?(percentage_index>>4):3)*0x20);
		pixel[2] = 0x2b;
#endif

		bat_level_height = bat_index * level_5_percentage_height;
		blank_l = (img_width -bat_level_width)/2;
		blank_r = img_width -bat_level_width - blank_l;
		p_mem_img = (unsigned char *)FRAME_ADDR + img_width*(bat_img_height - 25 - bat_level_height)*4;
		for (i = 0; i < img_width*bat_level_height*4; i+=4) {
			pix_pos_in_line = (i/4)%img_width;
			if(blank_l <= pix_pos_in_line && pix_pos_in_line < (img_width - blank_r)
					&& *((unsigned int *)(p_mem_img + i)) == 0){
				*((unsigned char *)(p_mem_img + i)) = pixel[2];
				*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
				*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
				*((unsigned char *)(p_mem_img + i+3)) = 0x00;
			}
		}
	}
#if 0

	/*fill percentage pixel*/
	blank_l = (img_width -percentage_part_width)/2;
	blank_r = img_width - percentage_part_width - blank_l;
	p_mem_img = (unsigned char *)FRAME_ADDR + img_width*(bat_img_height + 29)*4;
	for (i = 0; i < img_width*percentage_img_height*4; i+=4) {
		pix_pos_in_line = (i/4)%img_width;
		if(blank_l <= pix_pos_in_line && pix_pos_in_line < (blank_l + character_1_width)){
			HEADER_PIXEL(p_character_1_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}else if((blank_l + character_1_width + word_space1) <= pix_pos_in_line && 
				pix_pos_in_line < (blank_l + character_1_width + word_space1 + character_2_width)){
			HEADER_PIXEL(p_character_2_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}else if(character_3_width != 0 && (blank_l + character_1_width + word_space1 + character_2_width + word_space2)
				<= pix_pos_in_line && pix_pos_in_line < (blank_l + character_1_width + word_space1
				+ character_2_width + word_space2 + character_3_width)){
			HEADER_PIXEL(p_character_3_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}else if(character_4_width != 0
				&& (blank_l + character_1_width + word_space1 + character_2_width + word_space2
				+ character_3_width + word_space3) <= pix_pos_in_line && pix_pos_in_line < (img_width - blank_r)){
			HEADER_PIXEL(p_character_4_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}
	}
#endif

	if(percentage_index > 5){
		/*fill prompt text*/
		p_txt_img = charging_prompt_text_image_data;
		p_mem_img = (unsigned char *)FRAME_ADDR + img_width*(img_height - prompt_text_height)*4;
		for (i = 0; i < prompt_text_width*prompt_text_height*4; i+=4) {
			HEADER_PIXEL(p_txt_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}
	}

		/* Configure GFX pipeline */
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA0);
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA1);
	__raw_writel(0x12000098, DISPC_GFX_ATTRIBUTES);

	__raw_writel(((((PANEL_HEIGHT-img_height)/2) << 16) | ((PANEL_WIDTH-img_width)/2)), DISPC_GFX_POSITION);
	__raw_writel((((img_height-1) << 16) | (img_width-1)), DISPC_GFX_SIZE);

	/* Enable GFX pipeline */
	__raw_writel(__raw_readl(DISPC_GFX_ATTRIBUTES) | 0x1, DISPC_GFX_ATTRIBUTES);
}

static void loading_fastboot_ui(void)
{
	unsigned char pixel[3];
	char * p_mem_img;
	char * p_fastboot_img;
	int i, fastboot_ui_width, fastboot_ui_height, blank_l, blank_r, pix_pos_in_line;

	fastboot_ui_width = fastboot_img_bot_width;
	fastboot_ui_height = fastboot_img_top_height + fastboot_img_mid_height + fastboot_img_bot_height + 30 + 27;
	memset(FRAME_ADDR, 0, fastboot_ui_width*fastboot_ui_height*4);

	/*fill fastboot_img_top pixel*/
	p_mem_img = (unsigned char *)FRAME_ADDR;
	p_fastboot_img = fastboot_top_image_data;
	blank_l = (fastboot_img_bot_width -fastboot_img_top_width)/2;
	blank_r = fastboot_img_bot_width -fastboot_img_top_width - blank_l;
	for (i = 0; i < fastboot_img_bot_width*fastboot_img_top_height*4; i+=4) {
		pix_pos_in_line = (i/4)%fastboot_img_bot_width;
		if(blank_l <= pix_pos_in_line && pix_pos_in_line < (fastboot_img_bot_width - blank_r)){
			HEADER_PIXEL(p_fastboot_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}
	}
	/*fill fastboot_img_mid pixel*/
	p_mem_img = (unsigned char *)FRAME_ADDR + fastboot_img_bot_width*(fastboot_img_top_height + 30)*4;
	p_fastboot_img = fastboot_mid_image_data;
	blank_l = (fastboot_img_bot_width -fastboot_img_mid_width)/2;
	blank_r = fastboot_img_bot_width -fastboot_img_mid_width - blank_l;
	for (i = 0; i < fastboot_img_bot_width*fastboot_img_mid_height*4; i+=4) {
		pix_pos_in_line = (i/4)%fastboot_img_bot_width;
		if(blank_l <= pix_pos_in_line && pix_pos_in_line < (fastboot_img_bot_width - blank_r)){
			HEADER_PIXEL(p_fastboot_img, pixel);
			*((unsigned char *)(p_mem_img + i)) = pixel[2];
			*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
			*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
			*((unsigned char *)(p_mem_img + i+3)) = 0x00;
		}
	}
	/*fill fastboot_img_bot pixel*/
	p_mem_img = (unsigned char *)FRAME_ADDR + fastboot_img_bot_width*(fastboot_img_top_height + 30 + fastboot_img_mid_height + 27)*4;
	p_fastboot_img = fastboot_bot_image_data;
	for (i = 0; i < fastboot_img_bot_width*fastboot_img_bot_height*4; i+=4) {
		HEADER_PIXEL(p_fastboot_img, pixel);
		*((unsigned char *)(p_mem_img + i)) = pixel[2];
		*((unsigned char *)(p_mem_img + i+1)) = pixel[1];
		*((unsigned char *)(p_mem_img + i+2)) = pixel[0];
		*((unsigned char *)(p_mem_img + i+3)) = 0x00;
	}

	/* Configure GFX pipeline */
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA0);
	__raw_writel(FRAME_ADDR, DISPC_GFX_BA1);
	__raw_writel(0x12000098, DISPC_GFX_ATTRIBUTES);
	__raw_writel(((((PANEL_HEIGHT-fastboot_ui_height)/2) << 16) | ((PANEL_WIDTH-fastboot_ui_width)/2)), DISPC_GFX_POSITION);
	__raw_writel((((fastboot_ui_height-1) << 16) | (fastboot_ui_width-1)), DISPC_GFX_SIZE);
	/* Enable GFX pipeline */
	__raw_writel(__raw_readl(DISPC_GFX_ATTRIBUTES) | 0x1, DISPC_GFX_ATTRIBUTES);
}

#ifdef CONFIG_LONG_PRESSED_PWRON
int backlight_flash(void)
{
	int status;

	if (!board_is_c1_evt1()) {
		//Turn on GB pwr (GPIO_151)
		MV(CP(MCSPI4_CLK),	(PTU | M3))
		__raw_writel(__raw_readl(GPIO5_OE) & ~(1 << (151%32)), GPIO5_OE);
		__raw_writel(__raw_readl(GPIO5_SETDATAOUT) | (1 << (151%32)), GPIO5_SETDATAOUT);
	}

	/* configure I2C bus */
	if ((status = select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return status;
	}

#ifndef CONFIG_COMPACT_I2C_SETTING
	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	if (status) return status;

	status = i2c_read(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
	if (status) return status;

	//enable 3v3
	tca6416_gpio_output(EN3V3_PIN, 1);
	//set pwm high to low
	tca6416_gpio_output(PWM_HIGH, 0);
	//set pwm mid to low
	tca6416_gpio_output(PWM_MID, 0);
	//set pwm low to high
	tca6416_gpio_output(PWM_LOW, 1);
#else
	// reduce i2c accesses to speed up the process
	tca6416.reg_output = (tca6416.reg_output | 0x1008) & ~(0x00c0);
	tca6416.reg_config = tca6416.reg_config & ~(0x10c8);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
	i2c_write(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
#endif

	if (!board_is_c1_evt1()) {
		__raw_writel(__raw_readl(GPIO5_OE) & ~(1 << (151%32)), GPIO5_OE);
		__raw_writel(__raw_readl(GPIO5_SETDATAOUT) | (1 << (151%32)), GPIO5_CLEARDATAOUT);
	} else {
#ifndef CONFIG_COMPACT_I2C_SETTING
		//disable 3v3
		tca6416_gpio_output(EN3V3_PIN, 0);
		//set pwm low to low
		tca6416_gpio_output(PWM_LOW, 0);
#else
		// reduce i2c accesses to speed up the process
		tca6416.reg_output = tca6416.reg_output & ~(0x10c8);
		tca6416.reg_config = tca6416.reg_config & ~(0x10c8);
		i2c_write(TCA6416_I2C_ADDR, TCA6416_OUTPUT, 1, &tca6416.reg_output, 2);
		i2c_write(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416.reg_config, 2);
#endif
	}

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");
}
#endif

int display_init(void)
{
	uint32_t val = 0;
	u32 r;
	printf("display_initialized = %d, ql_is_on = %d\n",display_initialized,ql_is_on);
	if(display_initialized)
		return 0;
#ifdef CONFIG_KEEP_DISPLAY_ALIVE
	if (!ql_is_on)
#endif
	enable_display_power();
	/*enable SYS_CLK*/
	r = __raw_readl(CM_DSS_DSS_CLKCTRL);
	r = FLD_MOD(r, 1, 10, 10);	/* OPTFCLKEN_SYS_CLK */
	r = FLD_MOD(r, 2, 1, 0); /* MODULEMODE */
	__raw_writel(r, CM_DSS_DSS_CLKCTRL);
	
	_omap_dispc_initial_config();
	if (omapdss_dsi_display_enable()) {
		printf("failed to enable DSI\n");
		return 1;
	}
#ifdef CONFIG_KEEP_DISPLAY_ALIVE
	if (!ql_is_on) {
#endif
	panel_eyepiece_init();
	/*Enable auxclk2_ck 19200000 which is needed for QL_ArcticLink chip*/
	sr32(0x4A30A318, 0, 32, 0x10100);
	ql_chip_init();
#ifdef CONFIG_KEEP_DISPLAY_ALIVE
	ql_is_on = 1;
	} else {
		// re-enable GAMMA resistor
		u8 dat = 0x04;
		select_bus(GB_I2C_BUS, CFG_I2C_SPEED);
		i2c_write(PANEL_I2C_ADDR, 0x2, 1, &dat, 1);
		select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);
	}
#endif

	omapdss_dsi_vc_enable_hs(0, 1);
	dsi_vc_set_max_rx_packet_size(0, 0x4);
	display_initialized = 1;
	return 0;
}

void display_uninit(void)
{
	uint32_t val = 0;
	u32 r;
	if(!display_initialized)
		return;

	if (!board_is_c1_evt1())
		disable_backlight();
	dsi_disable_video_output(0);
	omapdss_dsi_display_disable();

#ifndef CONFIG_KEEP_DISPLAY_ALIVE
	/*disable auxclk2_ck*/
	sr32(0x4A30A318, 0, 32, 0x0);
#endif

	/*disable SYS_CLK*/
	r = __raw_readl(CM_DSS_DSS_CLKCTRL);
	r = FLD_MOD(r, 0, 10, 10);	/* OPTFCLKEN_SYS_CLK */
	__raw_writel(r, CM_DSS_DSS_CLKCTRL);

#ifndef CONFIG_KEEP_DISPLAY_ALIVE
	if (board_is_c1_evt1()) {
		disable_display_power();
	}
#else
	{
		// disable GAMMA resistor & power down all internal bias
		u8 dat = 0x0b;
		select_bus(GB_I2C_BUS, CFG_I2C_SPEED);
		i2c_write(PANEL_I2C_ADDR, 0x2, 1, &dat, 1);
		select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);
	}
#endif
	display_initialized = 0;
	ql_is_on = 0;
}

void show_logo(void)
{
	dsi_disable_video_output(0);
	loading_logo();
	printf("Showing LOGO...\n");
	dsi_enable_video_output(0);
	dsi_if_enable(0);
	udelay(3333);
	dsi_if_enable(1);
	if (!board_is_c1_evt1())
		enable_backlight();
}
void show_low_battery_frame(int index)
{
	dsi_disable_video_output(0);
	loading_low_battery_image(index);
	dsi_enable_video_output(0);
	dsi_if_enable(0);
	udelay(3333);
	dsi_if_enable(1);
	if (!board_is_c1_evt1())
		enable_backlight();
}
void show_charging_battery_frame(int bat_index, int percentage_index)
{
	dsi_disable_video_output(0);
	loading_charging_battery_image(bat_index, percentage_index);
	dsi_enable_video_output(0);
	dsi_if_enable(0);
	udelay(3333);
	dsi_if_enable(1);
	if (!board_is_c1_evt1())
		enable_backlight();
}
void show_fastboot_ui(void)
{
	dsi_disable_video_output(0);
	loading_fastboot_ui();
	dsi_enable_video_output(0);
	dsi_if_enable(0);
	udelay(3333);
	dsi_if_enable(1);
	if (!board_is_c1_evt1())
		enable_backlight();
}

#ifdef CONFIG_LEDS_INDICATOR
void led_init(void)
{
	int mask = (1 << (RED_LED%32)) | (1 << (BLUE_LED%32)) |
			(1 << (GREEN_LED%32));

	MV(CP(MCSPI4_SIMO),	(PTD | M3)); /* GPIO_152, G_LED */
	MV(CP(MCSPI4_SOMI),	(PTD | M3)); /* GPIO 153, B_LED */
	MV(CP(UART4_RX),	(PTD | M3)); /* GPIO 155, R_LED */

	__raw_writel(__raw_readl(GPIO5_OE) & ~(mask), GPIO5_OE);
	__raw_writel(mask, GPIO5_CLEARDATAOUT);
}
void set_led(int argb, int onoff)
{
	u8 a = (argb >> 24);
	u8 r = ((argb << 8) >> 24);
	u8 g = ((argb << 16) >> 24);
	u8 b = ((argb << 24) >> 24);

	int data = 0;

	if (a) {
		if (r) data |= (1 << (RED_LED%32));
		if (g) data |= (1 << (GREEN_LED%32));
		if (b) data |= (1 << (BLUE_LED%32));
	}

	__raw_writel(data, (onoff)? GPIO5_SETDATAOUT : GPIO5_CLEARDATAOUT);
}
#endif

/*****************************************
 * Routine: board_late_init
 * Description: Late hardware init.
 *****************************************/
int board_late_init(void)
{
	int status;
#if defined(CONFIG_JORJIN_APM_HACKS)
	/* retrieve the boot device stored in scratchpad */
	int boot_from_sd = (0x05 == (__raw_readl(0x4A326000) & 0xff));
#endif

	/* mmc */
	if( (status = omap4_mmc_init()) != 0) {
#if defined(CONFIG_JORJIN_APM_HACKS)
		if (!boot_from_sd)
#endif
		return status;
	}

#if defined(OMAP44XX_TABLET_CONFIG)
	/* Query display board EEPROM information */
	status = tablet_check_display_boardid();
#endif

#if defined(CONFIG_JORJIN_APM_HACKS)
	if (boot_from_sd) {
		/* boot from SD card */
		char buf[256] = {0};
		strcpy(buf, getenv("bootargs"));
		setenv("emmcbootargs", buf);
		strcpy(buf, getenv("sdbootargs"));
		setenv("bootargs", buf);
		strcpy(buf, getenv("bootcmd"));
		setenv("emmcbootcmd", buf);
		strcpy(buf, getenv("sdbootcmd"));
		setenv("bootcmd", buf);
	}

	/* set APM_HACK_ADDR to 0 for further using */
	__raw_writel(0x00000000, APM_HACK_ADDR);

	/* test if temperature sensor tmp105 is attached */
	{
		select_bus(2, 100);
		u8 dummy_val = 0;
		if (i2c_read(0x48, 0x00, 1, &dummy_val, 1) != 0) {
			/* Can't find tmp105.
			 * It has to be the stand alone mode of raccoonboard */
			__raw_writel((__raw_readl(APM_HACK_ADDR) | 0x0100),
				APM_HACK_ADDR);
		}
	}

	if (CPU_4460 == get_cpu_type()) {
		u16 usb_pid = 0;
		/* retrieve the PID from PMIC */
		select_bus(0, 100);
		i2c_read(0x49, 0x02, 1, &usb_pid, 2);
		if (0xC032 == usb_pid) {
			/* OMAP4460 + TWL6032 */
			__raw_writel((__raw_readl(APM_HACK_ADDR) | 0x01),
				APM_HACK_ADDR);
		}
	}
	select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);
#endif

	return status;
}

/****************************************************************************
 * check_fpga_revision number: the rev number should be a or b
 ***************************************************************************/
inline u16 check_fpga_rev(void)
{
	return __raw_readw(FPGA_REV_REGISTER);
}

/****************************************************************************
 * check_uieeprom_avail: Check FPGA Availability
 * OnBoard DEBUG FPGA registers need to be ready for us to proceed
 * Required to retrieve the bootmode also.
 ***************************************************************************/
int check_uieeprom_avail(void)
{
	volatile unsigned short *ui_brd_name =
	    (volatile unsigned short *)EEPROM_UI_BRD + 8;
	int count = 1000;

	/* Check if UI revision Name is already updated.
	 * if this is not done, we wait a bit to give a chance
	 * to update. This is nice to do as the Main board FPGA
	 * gets a chance to know off all it's components and we can continue
	 * to work normally
	 * Currently taking 269* udelay(1000) to update this on poweron
	 * from flash!
	 */
	while ((*ui_brd_name == 0x00) && count) {
		udelay(200);
		count--;
	}
	/* Timed out count will be 0? */
	return count;
}

/***********************************************************************
 * get_board_type() - get board type based on current production stats.
 *  - NOTE-1-: 2 I2C EEPROMs will someday be populated with proper info.
 *    when they are available we can get info from there.  This should
 *    be correct of all known boards up until today.
 *  - NOTE-2- EEPROMs are populated but they are updated very slowly.  To
 *    avoid waiting on them we will use ES version of the chip to get info.
 *    A later version of the FPGA migth solve their speed issue.
 ************************************************************************/
u32 get_board_type(void)
{
	return SDP_4430_V1;
}

#if defined(OMAP44XX_TABLET_CONFIG)
/***********************************************************************
 * tablet_pass_boardid()
 *  Pass display board id to kernel via setting environment variables.
 ************************************************************************/
static void tablet_pass_boardid(char *sPanel_id)
{
	char buf[256] = { 0 };
	char *cmdline;
	int len;

#if defined(CONFIG_OMAP4_ANDROID_CMD_LINE)
	cmdline = getenv("android.bootargs.extra");
#else
	cmdline = getenv("bootargs");
#endif

	/* existing cmdline? */
	if (cmdline)
		strncpy(buf, cmdline, sizeof(buf) - 1);

	/* does it fit? */
	len = strlen(buf) + strlen(sPanel_id) + 18;
	if (sizeof(buf) < len)
		return;

	strcat(buf, " omapdss.board_id=");
	strcat(buf, sPanel_id);

#if defined(CONFIG_OMAP4_ANDROID_CMD_LINE)
	setenv("android.bootargs.extra", buf);
#else
	setenv("bootargs", buf);
#endif
	return;
}
/***********************************************************************
 * tablet_read_i2c()
 *  Selects required i2c bus
 *  Reads data from i2c device.
 *  Restores default i2c settings
 ************************************************************************/
static int tablet_read_i2c(int bus, uchar chip, int off, char *sBuf, int len)
{
	int status;

	/* configure I2C bus */
	if ((status = select_bus(bus, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", bus);
		return status;
	}

	/* is present? */
	if ((status = i2c_probe(chip)) !=0 ) {
		debug("Probing %x failed\n", (int) chip);
		return status;
	}

	/* read buffer */
	status = i2c_read(chip, off, 1, (uchar *)sBuf, len);

	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0)
		printf("Setting i2c defaults: FAILED\n");

	return status;
}
/***********************************************************************
 * tablet_check_display_boardid() - check display board id.
 *  Reads display board EEPROM information and modifies bootargs to pass
 *  board information to kernel as "omapdss.board_id".
 ************************************************************************/
static int tablet_check_display_boardid(void)
{
	char sBuf[10] = { 0 };
	int status;

	/* Read display board eeprom:
	 *  12-19  board information (XXXX-XXX)
	 */
	status = tablet_read_i2c(CFG_DISP_I2C_BUS,
				 CFG_DISP_EEPROM_I2C_ADDR, 12, sBuf, 8);
	/* pass information to kernel */
	if (!status) {
		debug("board_id: %s\n", sBuf);
		tablet_pass_boardid((char *)sBuf);
	}

	return 0;
}
#endif

