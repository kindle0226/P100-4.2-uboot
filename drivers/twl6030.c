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
#include <config.h>
#ifdef CONFIG_TWL6030

#include <twl6030.h>
#include <bq2415x.h>
#include <asm/io.h>
#include <fastboot.h>

#define L_BATT_POLL_PERIOD		20000000	/* 20 seconds */
#define S_BATT_POLL_PERIOD		1600000		/* 1.6 seconds */
#define PWR_ON_POLL_PERIOD		2000000		/* 2 seconds */
#define POLL_INTERVAL			350000		/* 350 ms */
#define L_BATT_POLL_TIMEOUT		(L_BATT_POLL_PERIOD/POLL_INTERVAL)
#define S_BATT_POLL_TIMEOUT		(S_BATT_POLL_PERIOD/POLL_INTERVAL)
#define PWR_ON_POLL_TIMEOUT		(PWR_ON_POLL_PERIOD/POLL_INTERVAL)
#define CHARGE_DELAY			0
#define SHUTDOWN_COUNT			1
#if defined(CONFIG_FLASH_UBOOT)
#define BOOT_VOLTAGE				3539		/* 10% */
#else
#define BOOT_VOLTAGE				3518		/* 3.518V */
#endif
#define MAX_VOLTAGE				4200		/* 4.2V */
// used scaled table based on the voltage table of 1340 mAh battery @ 0.2C


#define BAT_VOLTAGE_100P                        4049
#define BAT_VOLTAGE_95P                         3943
#define BAT_VOLTAGE_90P                         3899
#define BAT_VOLTAGE_85P                         3861
#define BAT_VOLTAGE_80P                         3827
#define BAT_VOLTAGE_75P                         3795
#define BAT_VOLTAGE_70P                         3764
#define BAT_VOLTAGE_65P                         3735
#define BAT_VOLTAGE_60P                         3709
#define BAT_VOLTAGE_55P                         3685
#define BAT_VOLTAGE_50P                         3662
#define BAT_VOLTAGE_45P                         3641
#define BAT_VOLTAGE_40P                         3622
#define BAT_VOLTAGE_35P                         3605
#define BAT_VOLTAGE_30P                         3589
#define BAT_VOLTAGE_25P                         3574
#define BAT_VOLTAGE_20P                         3560
#define BAT_VOLTAGE_15P                         3546
#define BAT_VOLTAGE_10P                         3532
#define BAT_VOLTAGE_5P                          3518
#define BAT_VOLTAGE_0P                          3500



//#define FRAME_DISP_TIME			100000		/* 100 ms */
#define ANIMATION_DISP_TIME		15000000
//#define PWR_BTN_PRS_TIME		100000

#define TWL6032_SCALE		1000
#define TWL6032_TRIM1		0xcd
#define TWL6032_TRIM2		0xce
#define TWL6032_TRIM3		0xcf
#define TWL6032_TRIM4		0xd0
#define TWL6032_TRIM5		0xd1
#define TWL6032_TRIM6		0xd2

#define PHOENIX_START_CONDITION		0x1f
#define RESTART_BB		(1<<6)
#define FIRST_SYS_INS		(1<<5)
#define STRT_ON_RTC		(1<<4)
#define STRT_ON_PLUG_DET	(1<<3)
#define STRT_ON_USB_ID		(1<<2)
#define STRT_ON_RPWRON		(1<<1)
#define STRT_ON_PWRON		(1<<0)

#define PHOENIX_LAST_TURNOFF_STS	0x22
#define FALLBACK		(1<<7)
#define DEVOFF_RPWRON		(1<<6)
#define DEVOFF_SHORT		(1<<5)
#define DEVOFF_WDT		(1<<4)
#define DEVOFF_TSHUT		(1<<3)
#define DEVOFF_BCK		(1<<2)
#define DEVOFF_LPK		(1<<1)


static count = 0;

/* Charging Active */
enum {
	NOT_CHARGING = 0,
	CHARGING_AC,
	CHARGING_USB
};

/* Charger Presence */
enum {
	NO_CHARGER = 0,
	VAC_CHARGER,
	VBUS_CHARGER
};

/* Battery Presence */
enum {
	NO_BATTERY = 0,
	BATTERY
};

static int charging = NOT_CHARGING;

static int battery_level[21] = {
	BAT_VOLTAGE_0P,
	BAT_VOLTAGE_5P,
	BAT_VOLTAGE_10P,
	BAT_VOLTAGE_15P,
	BAT_VOLTAGE_20P,
	BAT_VOLTAGE_25P,
	BAT_VOLTAGE_30P,
	BAT_VOLTAGE_35P,
	BAT_VOLTAGE_40P,
	BAT_VOLTAGE_45P,
	BAT_VOLTAGE_50P,
	BAT_VOLTAGE_55P,
	BAT_VOLTAGE_60P,
	BAT_VOLTAGE_65P,
	BAT_VOLTAGE_70P,
	BAT_VOLTAGE_75P,
	BAT_VOLTAGE_80P,
	BAT_VOLTAGE_85P,
	BAT_VOLTAGE_90P,
	BAT_VOLTAGE_95P,
	BAT_VOLTAGE_100P,
};


static t_channel_calibration_info channel_calib_data[] = {
	{0, 0xCD, 0xCE, 116, 745, 0, 0, 0}, /* BATT_PRESENCE */
#ifdef CONFGI_CHRG_ADC1_DEBUG
	{1, 0xD1, 0xD2, 82, 900, 0, 0, 0},  /* BATT_TEMP     */
#endif
	{7, 0xD3, 0xD4, 614, 941, 0, 0, 0}, /* BATT_VOLTAGE  */
};

/* Functions to read and write from TWL6030 */
static inline int twl6030_i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
	return i2c_write(chip_no, reg, 1, &val, 1);
}

static inline int twl6030_i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	return i2c_read(chip_no, reg, 1, val, 1);
}

static int twl6030_gpadc_sw2_trigger(t_twl6030_gpadc_data * gpadc)
{
	u8 val;
	int ret = 0;

	ret = twl6030_i2c_write_u8(TWL6030_CHIP_ADC, gpadc->enable, gpadc->ctrl);
	if (ret)
		return -1;

	/* Waiting until the SW1 conversion ends*/
	val =  TWL6030_GPADC_BUSY;

	while (!((val & TWL6030_GPADC_EOC_SW) && (!(val & TWL6030_GPADC_BUSY)))) {
		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &val, gpadc->ctrl);
		if (ret)
			return -1;
		udelay(1000);
	}

	return 0;
}

static int twl6030_gpadc_read_channel(t_twl6030_gpadc_data * gpadc, u8 channel_no)
{
	u8 lsb = 0;
	u8 msb = 0;
	int ret;
	u8 channel = channel_no;

	if (gpadc->twl_chip_type == chip_TWL6032) {
		ret = twl6030_i2c_write_u8(TWL6030_CHIP_ADC, channel_no,
				TWL6032_GPSELECT_ISB);
		if (ret)
			return -1;
	}

	ret = twl6030_gpadc_sw2_trigger(gpadc);
	if (ret)
		return ret;

	if (gpadc->twl_chip_type == chip_TWL6032)
		channel = 0;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &lsb,
			gpadc->rbase + channel * 2);
	if (ret)
		return -1;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &msb,
			gpadc->rbase + 1 + channel * 2);
	if (ret)
		return -1;

	return (msb << 8) | lsb;
}
/*
 * Function to read in calibration errors and offset data
 * for later ADC conversion use
 */
static int twl6030_calibration(void)
{
	int i;
	int ret = 1;
	int gain_error_1;
	int offset_error;
	s16 ideal_code1, ideal_code2;
	u8 tmp1, tmp2;
	s8 delta_error1, delta_error2;

	for (i = ADC_CH0; i < ADC_CHANNEL_MAX; i++) {

		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp1,
			channel_calib_data[i].delta_err_reg1);
		if (ret)
			return -1;

		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp2,
			channel_calib_data[i].delta_err_reg2);
		if (ret)
			return -1;

		delta_error1 = ((s8)(tmp1 << 1) >> 1);
		delta_error2 = ((s8)(tmp2 << 1) >> 1);
		ideal_code1 = channel_calib_data[i].ideal_code1;
		ideal_code2 = channel_calib_data[i].ideal_code2;

		gain_error_1 = (delta_error2 - delta_error1) * SCALE
					/ (ideal_code2 - ideal_code1);
		offset_error = delta_error1 * SCALE - gain_error_1
					*  ideal_code1;
		channel_calib_data[i].gain_err = gain_error_1 + SCALE;
		channel_calib_data[i].offset_err = offset_error;

		channel_calib_data[i].calibrated = 1;

	}

	return 0;
}
/*
 * Function to read in calibration errors and offset data
 * for later ADC conversion use
 */
static int twl6032_calibration(void)
{
	int i;
	int ret = 0;
	int gain_error_1;
	int offset_error;
	s16 ideal_code1, ideal_code2;
	u8 tmp;
	s16 delta_error1, delta_error2;

	for (i = ADC_CH0; i < ADC_CHANNEL_MAX; i++) {

		switch (i) {
		/* NOTE: we didn't support CH2,8,9,10,11,14 for now */
		case ADC_CH0:
#ifdef CONFGI_CHRG_ADC1_DEBUG
		case ADC_CH1:
#endif
		//case ADC_CH3:
		//case ADC_CH4:
		//case ADC_CH5:
		//case ADC_CH6:
		//case ADC_CH12:
		//case ADC_CH13:
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM3);
			delta_error1 = ((((s16)tmp) & 0x1f) * 4);
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM1);
			delta_error1 += ((((s16)tmp) & 0x06) >> 1);
			if (tmp & 0x01) delta_error1 = -delta_error1;

			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM4);
			delta_error2 = ((((s16)tmp) & 0x2f) * 4);
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM2);
			delta_error2 += ((((s16)tmp) & 0x06) >> 1);
			if (tmp & 0x01) delta_error2 = -delta_error2;

			break;
		case ADC_CH7:
		//case ADC_CH18:
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM3);
			delta_error1 = ((((s16)tmp) & 0x1f) * 4);
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM1);
			delta_error1 += ((((s16)tmp) & 0x06) >> 1);
			if (tmp & 0x01) delta_error1 = -delta_error1;
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM5);
			delta_error1 += ((((s16)tmp) & 0x7e) >> 1) * ((tmp & 0x01)?-1:1);

			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM4);
			delta_error2 = ((((s16)tmp) & 0x2f) * 4);
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM2);
			delta_error2 += ((((s16)tmp) & 0x06) >> 1);
			if (tmp & 0x01) delta_error2 = -delta_error2;
			ret |= twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp, TWL6032_TRIM6);
			delta_error2 += ((((s16)tmp) & 0xfe) >> 1) * ((tmp & 0x01)?-1:1);

			break;
		default :
			continue;
		}

		if (ret) return -1;

		ideal_code1 = 1441;
		ideal_code2 = 3267;

		gain_error_1 = (delta_error2 - delta_error1) * TWL6032_SCALE
					/ (ideal_code2 - ideal_code1);
		offset_error = delta_error1 * TWL6032_SCALE - gain_error_1
					*  ideal_code1;
		channel_calib_data[i].gain_err = gain_error_1 + TWL6032_SCALE;
		channel_calib_data[i].offset_err = offset_error;

		channel_calib_data[i].calibrated = 1;

	}

	return 0;
}
/*
 * Function to determine if either a PC or Wall USB is attached
 */
static int is_charger_present(void)
{
	u8 val;
	int ret;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_CHARGER, &val,
				  CONTROLLER_STAT1);
	if (ret)
		return NO_CHARGER;
	if (val & VAC_DET)
		return VBUS_CHARGER;
	else if (val & VBUS_DET)
		return VBUS_CHARGER;

	return NO_CHARGER;
}

/*
 * Function to power down TWL and device
 * It should not return as device will be powered off
 */
void twl6030_shutdown(void)
{
	u8 val;

	twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val,
		TWL6030_PHOENIX_DEV_ON);

	/* Write out all 3 bits to shtudown PMIC power */
	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;

	twl6030_i2c_write_u8(TWL6030_CHIP_PM, val,
		TWL6030_PHOENIX_DEV_ON);

	/* TWL should be powered off here */
	while(1) {}
}

/*
 * Function to stop USB charging if charging
 * was enabled. Returns 1 if charging is disabled;
 * and 0 if it is not disabled.
 */
int twl6030_stop_usb_charging(void)
{
	if(charging == NOT_CHARGING)
		return 0;

	/* Disable USB charging */
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, 0, CONTROLLER_CTRL1);

	charging = NOT_CHARGING;

	return 1;
}
/*
 * Function to start USB charging if charging
 * was disabled and charger is now present.
 * Returns 1 if charging is enabled;
 * and 0 if it is not enabled.
 */
int twl6030_start_usb_charging(void)
{
	/*
	 * Only start charging if currently
	 * not charging and there is a charger
	 */
	if(charging != NOT_CHARGING)
		return 0;

	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VICHRG_1500,
							CHARGERUSB_VICHRG);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_CIN_LIMIT_NONE,
							CHARGERUSB_CINLIMIT);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, MBAT_TEMP,
							CONTROLLER_INT_MASK);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, MASK_MCHARGERUSB_THMREG,
							CHARGERUSB_INT_MASK);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VOREG_4P2,
							CHARGERUSB_VOREG);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_CTRL2_VITERM_100,
							CHARGERUSB_CTRL2);
	/* Enable USB charging */
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CONTROLLER_CTRL1_EN_CHARGER,
							CONTROLLER_CTRL1);
	charging = CHARGING_USB;

	return 1;
}

int twl6030_start_ac_charging(void)
{
	/*
	 * Only start charging if currently
	 * not charging and there is a charger
	 */
	if(charging != NOT_CHARGING)
		return 0;

        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, MAX_CHARGE_CURRENT,
                                                BQ2415x_SAFETY_LIMIT_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, NO_INPUT_CURRENT_LIMIT,
                                                BQ2415x_CONTROL_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, RGULATION_VOLTAGE,
                                                BQ2415x_VOLTAGE_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, CHARGE_CURRENT,
                                                BQ2415x_CURRENT_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, NORMAL_CHARGE_CURRENT,
                                                BQ2415x_SPECIAL_REG);
        /* enable AC charging */
        twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CONTROLLER_CTRL1_EN_CHARGER
                                                | CONTROLLER_CTRL1_SEL_CHARGER,
                                                CONTROLLER_CTRL1);
	charging = CHARGING_AC;

	return 1;
}

static int is_battery_present(t_twl6030_gpadc_data * gpadc)
{
	int bat_id_val;
	unsigned int current_src_val;
	u8 reg;
	int ret;

	bat_id_val = twl6030_gpadc_read_channel(gpadc, 0);
	if (bat_id_val < 0) {
		printf("Failed to read GPADC\n");
		return bat_id_val;
	}

	if (channel_calib_data[ADC_CH0].calibrated) {
		bat_id_val = (bat_id_val*
			((gpadc->twl_chip_type==chip_TWL6030)?SCALE:TWL6032_SCALE)-
			channel_calib_data[ADC_CH0].offset_err)/
			channel_calib_data[ADC_CH0].gain_err;
	}

	if (gpadc->twl_chip_type == chip_TWL6030)
		bat_id_val = (bat_id_val* 5 * 1000) >> (10 + 2);
	else
		bat_id_val = (bat_id_val * 5 * 1000) >> (12 + 2);

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &reg, GPADC_CTRL);
	if (ret) {
		printf("Failed to read GPADC\n");
		return -1;
	}

	current_src_val = (reg & GPADC_CTRL_ISOURCE_EN) ?
				GPADC_ISOURCE_22uA :
				GPADC_ISOURCE_7uA;

	bat_id_val = (bat_id_val * 1000) / current_src_val;

	if (bat_id_val < BATTERY_DETECT_THRESHOLD)
		return NO_BATTERY;

	return BATTERY;
}

int twl6030_get_battery_voltage(t_twl6030_gpadc_data * gpadc)
{
	int battery_volt = 0;
	int stopped_charging, saved_charging;
	u8 vbatch = TWL6030_GPADC_VBAT_CHNL;

	if (gpadc->twl_chip_type == chip_TWL6032)
		vbatch = TWL6032_GPADC_VBAT_CHNL;
	saved_charging = charging;
	/* Stop charging to achieve better accuracy */
	stopped_charging = twl6030_stop_usb_charging();

	/* measure Vbat voltage */
	battery_volt = twl6030_gpadc_read_channel(gpadc, vbatch);
	if (battery_volt < 0) {
		printf("Failed to read battery voltage\n");
		return battery_volt;
	}

	if(stopped_charging) {
		if (saved_charging == CHARGING_USB)
			twl6030_start_usb_charging();
		else
			twl6030_start_ac_charging();
	}
	/* Offset calibration data */
	if(channel_calib_data[ADC_CH7].calibrated) {
		battery_volt = (battery_volt*
		((gpadc->twl_chip_type==chip_TWL6030)?SCALE:TWL6032_SCALE)-
		channel_calib_data[ADC_CH7].offset_err)/
		channel_calib_data[ADC_CH7].gain_err;
	}

	if (gpadc->twl_chip_type == chip_TWL6030) {
		/*
		 * multiply by 1000 to convert the unit to milli
		 * division by 1024 (>> 10) for 10 bit ADC
		 * division by 8 (>> 3) for actual scaler gain
		 */
		battery_volt = (battery_volt * 40 * 1000) >> (10 + 3);
	}
	else {
		battery_volt = (battery_volt * 25 * 1000) >> (12 + 2);
	}
	return battery_volt;
}

void twl6030_init_battery_charging(void)
{
	u8 val;
	u8 pwr_btn_pressed = 0;
	u8 show_charging_again = 0;
	int i;
	int bat_level;
	int bat_percentage, bat_percentage_last = -1;
	int animation_disp_time = ANIMATION_DISP_TIME;
	int ret = 0;
	int abort = 0;
	int battery_volt = 0;
	int chargedelay = CHARGE_DELAY;
	int timeout, last_charger_state, charger_state;
	int shutdown_counter = SHUTDOWN_COUNT;
	t_twl6030_gpadc_data gpadc;
	u8 autoboot_disabled = 0;
	int pwrbtn_pressed_time = 0;

	gpadc.twl_chip_type = chip_TWL6030;
	gpadc.rbase = GPCH0_LSB;
	gpadc.ctrl = CTRL_P2;
	gpadc.enable = CTRL_P2_SP2;

	// check the turn on event
	twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val, PHOENIX_START_CONDITION);
	if (!(val & (STRT_ON_PWRON | STRT_ON_PLUG_DET | FIRST_SYS_INS))) {
		// Turn off the device if it is not powered on by PWRON, PLUG_DET or FIRST_SYS_INS events
		printf("PHOENIX_START_CONDITION: 0x%02x\n", val);
		if (val) goto shutdown;
	}

	// check if poweroff charging is needed
	if (__raw_readl(PRM_RSTST) & PRM_RSTST_RESET_WARM_BIT)
		autoboot_disabled = !strcmp(PUBLIC_SAR_RAM_1_FREE, "poweroff");
	else
		autoboot_disabled = !(val & (~STRT_ON_PLUG_DET));
		
#if defined(CONFIG_FLASH_UBOOT) || defined(CONFIG_AUTO_BOOT)
	autoboot_disabled = 0;
#endif

#ifdef CONFIG_LONG_PRESSED_PWRON
	if (!autoboot_disabled) {
		// check if the power button is still pressed
		twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val, STS_HW_CONDITIONS);
		if (!(val & STS_PWRON) || (__raw_readl(PRM_RSTST) & PRM_RSTST_RESET_WARM_BIT)) {
			backlight_flash();
		} else {
			printf("Cannot detect the power button!\n");
			if (NO_CHARGER == (charger_state=is_charger_present()))
				goto shutdown;

			// charger detected, go to poweroff charging process
			autoboot_disabled = 1;
		}
	}
#endif

#ifdef CONFIG_LEDS_INDICATOR
	/*turn off blue led*/
	set_led(0xff0000ff, 0);
#endif

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_USB, &val, USB_PRODUCT_ID_LSB);

	if (ret == 0) {
		if(val == 0x32)
		{
			gpadc.twl_chip_type = chip_TWL6032;
			gpadc.rbase = TWL6032_GPCH0_LSB;
			gpadc.ctrl = TWL6032_CTRL_P1;
			gpadc.enable = CTRL_P1_SP1;
		}
	} else {
		printf("twl6030_init_battery_charging(): "
		       "could not determine chip!\n");
		return;
	}

#ifdef CONFIG_BAT_OVRCURR_HACK
	// NOTE: this is a HACK to deactivate the battery over-current protection
	printf("Raise TWL603X's charge voltage to 4.2V\n");
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VOREG_4P2, CHARGERUSB_VOREG);
#endif

	/* Forced stop charging */
	charging = CHARGING_USB;
	ret = twl6030_stop_usb_charging();
	if (ret == 0 ) {
	        printf("Failed to stop charging\n");
	}

#if defined(CONFIG_SHIPPING_MODE) && defined(CONFIG_OMAP44XX)
	// check if the device is in shipping mode
	{
		int ret = 0;
		int mmc_cont = SHIP_MOD_MMC_SLOT;
		ulong src_addr = SHIP_MOD_MMC_OFFSET;
		ulong dst_addr = SHIP_MOD_DDR_OFFSET;
		ulong size = 8;
		ret = mmc_init(mmc_cont);
		if (!ret) mmc_read(mmc_cont, src_addr, (unsigned char *)dst_addr, size);

		// check if the device is in shipping mode
		if (!strncmp((unsigned char *)dst_addr, "SHIPPING", 8)) {
			if (NO_CHARGER == (charger_state=is_charger_present())) {
				// if the device is not charging, force shutdown
				printf("Shipping mode! Force shutdown!");
				goto shutdown;
			} else {
				// if the device is charging, deactive shipping mode
				printf("Deactivate shipping mode\n");
				mmc_erase(mmc_cont, src_addr, size);

				// clear autoboot_disabled to avoid poweroff charging
				autoboot_disabled = 0;
			}
		}
	}
#endif

	/* Calibration */
	if (gpadc.twl_chip_type == chip_TWL6032)
		ret = twl6032_calibration();
	else
		ret = twl6030_calibration();
	if (ret) {
	        printf("Failed to calibrate\n");
	}

	/* Enable VBAT measurement */
	if (gpadc.twl_chip_type == chip_TWL6030) {
		twl6030_i2c_write_u8(TWL6030_CHIP_PM, VBAT_MEAS, MISC1);
		twl6030_i2c_write_u8(TWL6030_CHIP_ADC, GPADC_CTRL_SCALER_DIV4,
			TWL6030_GPADC_CTRL);
	}
	else
		twl6030_i2c_write_u8(TWL6030_CHIP_ADC,
			GPADC_CTRL2_CH18_SCALER_EN, TWL6032_GPADC_CTRL2);

	/* Enable GPADC module */
	ret = twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, FGS | GPADCS, TOGGLE1);
	if (ret) {
		printf("Failed to enable GPADC\n");
		return;
	}

	/*
	 * Make dummy conversion for the TWL6032
	 * (first conversion may be failed)
	 */
	if (gpadc.twl_chip_type == chip_TWL6032)
		twl6030_gpadc_sw2_trigger(&gpadc);

	/*
	 * In case if battery is absent or error occurred while the battery
	 * detection we will not turn on the battery charging
	 */
	 
#if 1//test charge animation
	if (is_battery_present(&gpadc) <= 0) {
		printf("Battery not detected\n");
		return;
	}

	battery_volt = twl6030_get_battery_voltage(&gpadc);

	if (!autoboot_disabled && (battery_volt < 0 || battery_volt >= BOOT_VOLTAGE))
		return;
#endif
#ifdef CONFIG_SILENT_CONSOLE
	if (gd->flags & GD_FLG_SILENT) {
		/* Restore serial console */
		console_assign (stdout, "serial");
		console_assign (stderr, "serial");
	}
#endif

	if (autoboot_disabled)
		printf("Forced to charge!\n");
	else
		printf("Main battery voltage too low!\n");
	printf("Hit any key to stop charging: %2d ", chargedelay);

	if (tstc()) {	/* we got a key press	*/
		(void) getc();  /* consume input	*/
	}

	while ((chargedelay > 0) && (!abort)) {
		int i;

		--chargedelay;
		/* delay 100 * 10ms */
		for (i=0; !abort && i<100; ++i) {
			if (tstc()) {	/* we got a key press	*/
				abort  = 1;	/* don't auto boot	*/
				chargedelay = 0;	/* no more delay	*/
				(void) getc();  /* consume input	*/
				break;
			}
			udelay (10000);
		}
		printf ("\b\b\b%2d ", chargedelay);
	}
	putc ('\n');

#ifdef CONFIG_SILENT_CONSOLE
	if (gd->flags & GD_FLG_SILENT) {
		/* Restore silent console */
		console_assign (stdout, "nulldev");
		console_assign (stderr, "nulldev");
	}
#endif

	if (abort)
		return;

	charger_state = is_charger_present();
	if (charger_state == VAC_CHARGER)
		ret = twl6030_start_ac_charging();
	else if (charger_state == VBUS_CHARGER)
		ret = twl6030_start_usb_charging();
	//battery_volt = 3550;//test low power battery showing
	if (charger_state == NO_CHARGER || ret==0){
		printf("Charger not detected.");
		if (!autoboot_disabled) {
			//show low_battery image for a while, then shut down.
			for(i = 0; i < sizeof(battery_level)-1;i++){
				if(battery_volt < battery_level[i+1])
					break;
			}
			display_init();
			show_low_battery_frame((i==0)?1:i);
			for(i = 0; i < 100; i++)//delay 5s
				udelay (50000);
		}
		goto shutdown;
	}

	printf("Charging...\n");
	/*
	 * Wait for battery to charge to the level when kernel can boot
	 * During this time, battery voltage is polled periodically and
	 * charger presence is monitored. If charger is detected to be
	 * unplugged for a period of time, the device will proceed
	 * to shutdown to avoid battery drain.
	 */
	do {
#ifdef CONFGI_CHRG_ADC1_DEBUG
		battery_volt = twl6030_gpadc_read_channel(&gpadc, 1);
		if (channel_calib_data[ADC_CH1].calibrated) {
			battery_volt = (battery_volt*
			((gpadc.twl_chip_type==chip_TWL6030)?SCALE:TWL6032_SCALE)-
			channel_calib_data[ADC_CH1].offset_err)/
			channel_calib_data[ADC_CH1].gain_err;
		}

		if (gpadc.twl_chip_type == chip_TWL6030)
			battery_volt = (battery_volt * 10 * 1000) >> 13;
		else
			battery_volt = (battery_volt * 1250) >> 12;
		printf("\rADC1: %d mV", battery_volt);
#endif

		/* Read battery voltage */
		battery_volt = twl6030_get_battery_voltage(&gpadc);

#ifdef CONFGI_CHRG_ADC1_DEBUG
		printf(", Battery Voltage: %d mV", battery_volt);
#else
		printf("\rBattery Voltage: %d mV", battery_volt);
#endif
		/* calculate battery level and percentage*/
		for(i = 0; i < sizeof(battery_level)-1;i++){
			if(battery_volt < battery_level[i+1])
				break;
		}
		bat_level = i;
		bat_percentage = (bat_level == sizeof(battery_level)-1)? 100 :
			(battery_volt - battery_level[i]) * 5 /(battery_level[i+1] - battery_level[i]) + 5 * i;
		// check if the percentage is below 0
		if (bat_percentage < 0) bat_percentage = 0;

		/* reset safety timer */
		twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, 32,
					CONTROLLER_WDG);

		twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER,
                                                        TMR_RST | EN_STAT,
                                                        BQ2415x_STATUS_CTRL_REG);
		timeout = (charger_state == NO_CHARGER)?
			S_BATT_POLL_TIMEOUT : L_BATT_POLL_TIMEOUT;

		pwrbtn_pressed_time=0;

		while (timeout--){
			/*jump out of the loop as soon as charger is unplugged*/
			if(NO_CHARGER == (charger_state=is_charger_present())){
				break;
			}

			if(animation_disp_time > 0){
				if(bat_percentage_last != bat_percentage || show_charging_again){
					display_init();
					show_charging_battery_frame(bat_level, bat_percentage);
					show_charging_again = 0;
				}
				animation_disp_time -= POLL_INTERVAL;
			}else /*if(autoboot_disabled || battery_volt < BOOT_VOLTAGE)*/{
				/*turn off display to save power*/
				display_uninit();
			}

			//udelay(POLL_INTERVAL);

			twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val, STS_HW_CONDITIONS);
#if 0
			if (pwr_btn_pressed != !(val & STS_PWRON)) {
				static int disp_can_be_turned_off = 0;
				if (pwr_btn_pressed) {
					/*when button has been released*/
					disp_can_be_turned_off = !disp_can_be_turned_off;
					if (animation_disp_time > 0) {
						/*turn off the display if available*/
						if (disp_can_be_turned_off) animation_disp_time = 0;
					} else disp_can_be_turned_off = 1;
				}
				pwr_btn_pressed = !(val & STS_PWRON);
			}
			
			if(pwr_btn_pressed){
#else
			if (!(val & STS_PWRON)) {
#endif
				/*detect pressed time*/
				//TODO: use KPD_STS for key press duration check
				if(autoboot_disabled && ++pwrbtn_pressed_time>=PWR_ON_POLL_TIMEOUT) {
					if (battery_volt >= BOOT_VOLTAGE) {
						//display_uninit();
						break;
					}
				}
				/*show charging again*/
				if(animation_disp_time <= 0) {
					show_charging_again = 1;
					animation_disp_time = ANIMATION_DISP_TIME;
				}

			} else pwrbtn_pressed_time = 0;

			bat_percentage_last = bat_percentage;
#ifdef CONFIG_LEDS_INDICATOR

			/*less than boot voltage, blue led flicks*/
			if(battery_volt < BOOT_VOLTAGE){

				/*turn off red leds*/
				set_led(0xffffffff,0);
				//udelay(3000000);
				if(count == 10)
				{
				/*turn on red leds*/
				set_led(0xffff0000,1);
				count = 0;

				}
				count++;


			} else if(autoboot_disabled && bat_percentage < 20) {

				/*turn off red leds*/
				set_led(0xffffffff,0);
				//udelay(3000000);
				if(count == 10)
				{
				/*turn on red leds*/
				set_led(0xffff0000,1);
				count = 0;
				}
				count++;


			} else if(autoboot_disabled && ( bat_percentage >= 20 && bat_percentage < 90)) {
			
				/*turn off orange leds*/
				set_led(0xffffffff,0);
				//udelay(3000000);
				if(count == 10)
				{
				/*turn on orange leds*/
				set_led(0xffffff00,1);
				count = 0;
				}
				count++;
			
			}else if(autoboot_disabled && (bat_percentage >= 90 && bat_percentage <= 95)) {

				/*turn off green leds*/
				set_led(0xffffffff,0);
				//udelay(3000000);
				if(count == 10)
				{
				/*turn on green leds*/
				set_led(0xff00ff00,1);
				count = 0;
				}
				count++;
				
			}else if(autoboot_disabled && bat_percentage >95) {

				/*turn on green leds*/
				set_led(0xff00ff00, 1);
			}
#endif
		}

		if(autoboot_disabled && pwrbtn_pressed_time>=PWR_ON_POLL_TIMEOUT) {
			printf("\nStop charging then boot...\n");
			break;
		}

		/* Charger plug or unplug action detected*/
		if (charger_state != last_charger_state) {

			last_charger_state = charger_state;

			if (charging != NOT_CHARGING){
				ret = twl6030_stop_usb_charging();
			}

			if (charging == NOT_CHARGING) {
				if (charger_state == VAC_CHARGER) {
					/* VAC Charger plugged */
					twl6030_start_ac_charging();
				} else if (charger_state == VBUS_CHARGER) {
					/* VBUS Charger plugged */
					twl6030_start_usb_charging();
				}
			} else if (charger_state == NO_CHARGER) {

					printf("\rCharger Unplugged!       ");
					/* Charger unplugged */
					twl6030_stop_usb_charging();

					/* Will count down before shutdown */
					shutdown_counter = SHUTDOWN_COUNT;
				}

		} else if (charging == NOT_CHARGING &&
				charger_state == NO_CHARGER) {
			/*
			 * Charger continues to be unplugged.
			 * Countdown until 0 and shut off device
			 */

			printf("\rCharger Unplugged! (%d)   ",
				--shutdown_counter);

			if (shutdown_counter == 0)
				goto shutdown;
		}


	} while (autoboot_disabled || battery_volt < BOOT_VOLTAGE);

	printf("\n");
#ifdef CONFIG_LEDS_INDICATOR
	/*turn off rgb leds*/
	set_led(0xffffffff, 0);
#endif

	return;

shutdown:

	printf("\nShutdown!\n");
	twl6030_shutdown();
	return;	/*Should never get here */

}

#define TCA6416_I2C_ADDR     0x20
#define GB_I2C_BUS      2
#define TCA6416_INPUT	0
#define TCA6416_CONFIG	6//0 output, 1 input
#define CAM_SHOT_BTN 0

static u16 tca6416_chip2_reg_config;

static int tca6416_chip2_gpio_config_as_input(u8 off)
{
	u8 val_to_write;
	u16 reg_val;
	uint reg_addr;
	int ret;

	reg_val = tca6416_chip2_reg_config | (1u << off);
	if(off < 8){
		val_to_write = reg_val & 0xff;
		reg_addr = TCA6416_CONFIG;
	}else{
		val_to_write = (reg_val >> 8) & 0xff;
		reg_addr = TCA6416_CONFIG + 1;
	}
	if (ret = i2c_write(TCA6416_I2C_ADDR, reg_addr, 1, &val_to_write, 1))
		goto exit;
	tca6416_chip2_reg_config = reg_val;

	ret = 0;
exit:
	return ret;
}

static u8 tca6416_chip2_gpio_read_input(u8 off)
{
	uchar input_data_r;
	uint reg_addr;

	if(off < 8)
		reg_addr = TCA6416_INPUT;
	else
		reg_addr = TCA6416_INPUT + 1;

	if (i2c_read(TCA6416_I2C_ADDR, reg_addr, 1, &input_data_r, 1))
		return -1;
	if(off < 8)
		return input_data_r & (1u << off);
	else
		return input_data_r << 8 & (1u << off);
}

int check_fastboot_mode()
{
	//u8 val;
	//u8 pwr_btn_pressed = 0;
	u8 cam_shot_input = 0;
	int detect_count = 4;

	/*twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val, STS_HW_CONDITIONS);
	pwr_btn_pressed = !(val & STS_PWRON);
	if(!pwr_btn_pressed)
		return 0;*/

	/* configure I2C bus */
	if ((select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return 0;
	}
	if(i2c_read(TCA6416_I2C_ADDR, TCA6416_CONFIG, 1, &tca6416_chip2_reg_config, 2)){
		printf("Failed to read tca6416 config register\n");
		return 0;
	}
	tca6416_chip2_gpio_config_as_input(CAM_SHOT_BTN);
	/* restore default settings */
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0){
		printf("Setting i2c defaults: FAILED\n");
		return 0;
	}

	do{
		if ((select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
			printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
			return 0;
		}
		cam_shot_input = tca6416_chip2_gpio_read_input(CAM_SHOT_BTN);
		if(cam_shot_input == -1 || cam_shot_input != 0)
			return 0;

		if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0){
			printf("Setting i2c defaults: FAILED\n");
			return 0;
		}

		/*twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val, STS_HW_CONDITIONS);
		pwr_btn_pressed = !(val & STS_PWRON);
		if(!pwr_btn_pressed)
			return 0;*/
		udelay(10000);
	}while(--detect_count > 0);
	return 1;
}

u8 is_cam_shot_pressed(void)
{
	u8 cam_shot_input = 0;

	if ((select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return 0;
	}

	cam_shot_input = tca6416_chip2_gpio_read_input(CAM_SHOT_BTN);

	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0){
		printf("Setting i2c defaults: FAILED\n");
	}
	if(cam_shot_input == -1 || cam_shot_input != 0)
		return 0;
	else
		return 1;
}

u8 is_cam_shot_released(void)
{
	u8 cam_shot_input = 0;

	if ((select_bus(GB_I2C_BUS, CFG_I2C_SPEED)) != 0 ) {
		printf("Setting bus[%d]: FAILED", GB_I2C_BUS);
		return 0;
	}
	cam_shot_input = tca6416_chip2_gpio_read_input(CAM_SHOT_BTN);
	if (select_bus(CFG_I2C_BUS, CFG_I2C_SPEED) != 0){
		printf("Setting i2c defaults: FAILED\n");
	}
	if(cam_shot_input == -1 || cam_shot_input == 0)
		return 0;
	else
		return 1;
}

void twl6030_usb_device_settings()
{
	u8 data = 0;

	/* Select APP Group and set state to ON */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, 0x21, VUSB_CFG_STATE);

	twl6030_i2c_read_u8(TWL6030_CHIP_PM, &data, MISC2);
	data |= 0x10;

	/* Select the input supply for VBUS regulator */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, data, MISC2);
}
#endif
