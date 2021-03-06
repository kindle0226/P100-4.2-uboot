/*
 * (C) Copyright 2009
 * Texas Instruments, <www.ti.com>
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

#include <common.h>
#include <i2c.h>

/* I2C chip addresses */
#define TWL6030_CHIP_PM		0x48

#define TWL6030_CHIP_USB	0x49
#define TWL6030_CHIP_ADC	0x49
#define TWL6030_CHIP_CHARGER	0x49
#define TWL6030_CHIP_PWM	0x49
#define TWL6030_CHIP_ID2	0x4A

#define TWL6032_GPSELECT_ISB	0x35

#define MISC1			0xE4
#define VAC_MEAS		(1 << 2)
#define VBAT_MEAS		(1 << 1)
#define BB_MEAS			(1 << 0)

#define TWL6030_GPADC_CTRL		0x2e
#define TWL6032_GPADC_CTRL2		0x2f
#define GPADC_CTRL2_CH18_SCALER_EN	(1 << 2)
#define GPADC_CTRL_SCALER_DIV4		(1 << 3)

/* PM REGISTERS */
#define TWL6030_PHOENIX_DEV_ON		0x25
#define APP_DEVOFF			(1<<0)
#define CON_DEVOFF			(1<<1)
#define MOD_DEVOFF			(1<<2)

#define STS_HW_CONDITIONS		0x21
#define STS_PWRON				(1<<0)
#define STS_PLUG_DET			(1<<3)

/* ADC REGISTERS */
#define GPADC_CTRL_ISOURCE_EN		(1 << 7)
#define GPADC_ISOURCE_22uA		22
#define GPADC_ISOURCE_7uA		7

/* Battery CHARGER REGISTERS */
#define CONTROLLER_INT_MASK	0xE0
#define CONTROLLER_CTRL1	0xE1
#define CONTROLLER_WDG		0xE2
#define CONTROLLER_STAT1	0xE3
#define CHARGERUSB_INT_STATUS	0xE4
#define CHARGERUSB_INT_MASK	0xE5
#define CHARGERUSB_STATUS_INT1	0xE6
#define CHARGERUSB_STATUS_INT2	0xE7
#define CHARGERUSB_CTRL1	0xE8
#define CHARGERUSB_CTRL2	0xE9
#define CHARGERUSB_CTRL3	0xEA
#define CHARGERUSB_STAT1	0xEB
#define CHARGERUSB_VOREG	0xEC
#define CHARGERUSB_VICHRG	0xED
#define CHARGERUSB_CINLIMIT	0xEE
#define CHARGERUSB_CTRLLIMIT1	0xEF

/* CHARGERUSB_VICHRG */
#define CHARGERUSB_VICHRG_500		0x4
#define CHARGERUSB_VICHRG_1500		0xE
/* CHARGERUSB_CINLIMIT */
#define CHARGERUSB_CIN_LIMIT_100	0x1
#define CHARGERUSB_CIN_LIMIT_300	0x5
#define CHARGERUSB_CIN_LIMIT_500	0x9
#define CHARGERUSB_CIN_LIMIT_NONE	0xF
/* CONTROLLER_INT_MASK */
#define MVAC_FAULT		(1 << 6)
#define MAC_EOC			(1 << 5)
#define MBAT_REMOVED		(1 << 4)
#define MFAULT_WDG		(1 << 3)
#define MBAT_TEMP		(1 << 2)
#define MVBUS_DET		(1 << 1)
#define MVAC_DET		(1 << 0)
/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM		(1 << 3)
#define MASK_MCHARGERUSB_STAT		(1 << 2)
#define MASK_MCHARGERUSB_THMREG		(1 << 1)
#define MASK_MCHARGERUSB_FAULT		(1 << 0)
/* CHARGERUSB_VOREG */
#define CHARGERUSB_VOREG_3P52		0x01
#define CHARGERUSB_VOREG_4P0		0x19
#define CHARGERUSB_VOREG_4P2		0x23
#define CHARGERUSB_VOREG_4P76		0x3F
/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50	(0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100	(1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150	(2 << 5)
/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_CHARGER	(1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)

/* CONTROLLER_STAT1 */
#define CHRG_EXTCHRG_STATZ	(1 << 7)
#define CHRG_DET_N		(1 << 5)
#define VAC_DET			(1 << 3)
#define VBUS_DET		(1 << 2)

#define TOGGLE1		0x90
#define FGS		(1 << 5)
#define FGR		(1 << 4)
#define GPADCS		(1 << 1)
#define GPADCR		(1 << 0)

#define TWL6032_CTRL_P1		0x36
#define CTRL_P1_SP1	(1 << 3)

#define CTRL_P2		0x34
#define CTRL_P2_SP2	(1 << 2)
#define CTRL_P2_EOCP2	(1 << 1)
#define CTRL_P2_BUSY	(1 << 0)

#define TWL6030_GPADC_EOC_SW		(1 << 1)
#define TWL6030_GPADC_BUSY		(1 << 0)

#define GPADC_CTRL_ISOURCE_EN		(1 << 7)

#define GPADC_CTRL	0x2e

#define GPCH0_LSB	0x57
#define GPCH0_MSB	0x58

#define TWL6032_GPCH0_LSB	0x3b
#define TWL6032_GPCH0_MSB	0x3c

#define VUSB_CFG_STATE		0xA2
#define MISC2			0xE5

#define SCALE			(1<<15)

#define USB_PRODUCT_ID_LSB	0x02

#define TWL6030_GPADC_VBAT_CHNL	0x07
#define TWL6032_GPADC_VBAT_CHNL	0x12

#define BATTERY_RESISTOR	10000
#define SIMULATOR_RESISTOR	5000
#define BATTERY_DETECT_THRESHOLD	((BATTERY_RESISTOR + SIMULATOR_RESISTOR) / 2)

#define GPADC_ISOURCE_22uA		22
#define GPADC_ISOURCE_7uA		7

typedef enum {
	chip_TWL6030,
	chip_TWL6032,
	chip_TWL603X_cnt
}t_TWL603X_chip_type;

typedef struct{
	t_TWL603X_chip_type twl_chip_type;
	u8 rbase;
	u8 ctrl;
	u8 enable;
}t_twl6030_gpadc_data;

typedef struct {
	int     channel;
	u8      delta_err_reg1;
	u8      delta_err_reg2;
	int     ideal_code1;
	int     ideal_code2;
	int     gain_err;
	int     offset_err;
	int     calibrated;
}t_channel_calibration_info;

typedef enum {
	ADC_CH0 = 0,	/* BATT_PRESENCE */
#ifdef CONFGI_CHRG_ADC1_DEBUG
	ADC_CH1,
#endif
	ADC_CH7,	/* BATT_VOLTAGE */
	ADC_CHANNEL_MAX
}t_adc_channels;


void twl6030_init_battery_charging(void);
void twl6030_usb_device_settings(void);
