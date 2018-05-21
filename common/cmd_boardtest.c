/*
 * Board test functions
 */
#include <common.h>
#include <command.h>
#include <i2c.h>
#if defined(CONFIG_BOARD_TEST)

#ifdef CONFIG_TWL6030
#include <twl6030.h>

#define PW_BTN_CHIP TWL6030_CHIP_PM
#define PW BTN_ADDR STS_HW_CONDITIONS
#endif

#ifndef PW_BTN_CHIP
#define PW_BTN_CHIP 0x48
#endif
#ifndef BTN_ADDR
#define BTN_ADDR 0x21
#endif
#define BMM050_U16 unsigned short
#define BMM050_S16 signed short
#define BMM050_S32 signed int
#define abs(value) (((value) < 0) ? ((value)*-1) : (value))


#define set_i2c_bus(a,b) 	if(select_bus(a, b)){ \
								printf("ibus [%d] set FAILED\n",a); \
								return 1; \
							}

typedef enum{
	Acceleration = 1,
	Gyroscope,
	Magnetic_normal,
	Magnetic_advanced,
	ALS_static,
	ALS_dynamic,
	Proximity_static,
	Proximity_dynamic,
}sensor_case;

static int ALS_Up = 150;
static int ALS_Low = 50;

int do_ledset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong input;
	int delaycnt = 0;
	int flickertime = 60;

	if (argc != 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	input = simple_strtoul(argv[1], NULL, 10);

	switch(input){
		case 0:/*All Off*/
			run_command("mw 0x4805b134 ffffffff", 0);
			run_command("mw 0x4805b13c 00000000", 0);
			run_command("mw 0x4805b190 00000000", 0);
			run_command("mw 0x4a10015c 00000100", 0);
			break;
		case 1:
			run_command("mw 0x4a10015c 00000103", 0);
			run_command("mw 0x4805b134 f7ffffff", 0);
			run_command("mw 0x4805b13c 08000000", 0);
			break;
#if 0
		case 2:/*LED flickering*/
			printf ("Blue LED flickering...\n");
			run_command("mw 0x4a10015c 00000103", 0);
			run_command("mw 0x4805b134 f7ffffff", 0);
			while(flickertime){
				if(flickertime & 0x01)
					run_command("mw 0x4805b13c 08000000", 0);
				else
					run_command("mw 0x4805b13c 00000000", 0);
				flickertime--;
				for(delaycnt=0; delaycnt<100; ++delaycnt){ udelay(10000);}
			}
			run_command("mw 0x4805b134 ffffffff", 0);
			run_command("mw 0x4805b13c 00000000", 0);
			run_command("mw 0x4805b190 00000000", 0);
			run_command("mw 0x4a10015c 00000100", 0);
			break;
#endif
	}

	return 0;
}


int do_st (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned char	linebuf[8];
	unsigned char	delaycnt = 0;
	unsigned char	delaytime = 0;
	unsigned char	byte;
	unsigned char	testflag = 0;
	int ch0_adc, ch1_adc, luxval;
	int ch0_coeff = 0;
	int ch1_coeff = 0;
	int ratio;
	int jt1 = 0,jt2 = 0,rangexy = 0,rangez = 0;
	short ax1 = 0,ax2 = 0,ax3 = 0,ax4 = 0;
	short ay1 = 0,ay2 = 0,ay3 = 0,ay4 = 0;
	short az1 = 0,az2 = 0,az3 = 0,az4 = 0;
	unsigned long result = 0;
	short value1 = 0,value2 = 0,diff = 0;
	unsigned char range=0;
	unsigned char t1 = 0,t2 = 0,t3 = 0,t4 = 0,MZ1 = 0,MZ2 = 0;
	BMM050_U16 dz1,ddz1,R1,R2;
	BMM050_S16 dz2,ddz2,z5,z0,z1,z2;
	BMM050_S16 dz3,ddz3;
	BMM050_S16 dz4,ddz4;
	BMM050_U16 dxyz1,ddxyz1;
	BMM050_S32 retval = 0,retval2 = 0;
	ulong input;

	if (argc != 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	input = simple_strtoul(argv[1], NULL, 10);

	switch(input){
		case Acceleration:
			result = 0;
			range=0x08;
			rangexy=204;
			rangez=102;
			//run_command("ibus 3 0x64", 0 );
			set_i2c_bus(0x03, 0x64);
			byte = 0x00;
			if(i2c_write(0x18, 0x11, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x11 0x00)\n");
			}
			byte = 0x80;
			if(i2c_write(0x18, 0x11, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x11 0x80)\n");
			}
			byte = 0xb6;
			if(i2c_write(0x18, 0x14, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x14 0xb6)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x02, 0x01, linebuf, 0x06) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x02 0x06)\n");
			}
			byte = range;
			if(i2c_write(0x18, 0x0f, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x0f 0x08)\n");
				return 1;
			}
			byte = 0x11;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x32 0x11)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x02, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x02 0x02)\n");
				return 0;
			}
			ax1 = ((linebuf[0]&0xf0)>>4)|((linebuf[1]&0xff)<<4);
			ax2 = (ax1<<(sizeof(short)*8-12));
			value1= (ax2>>(sizeof(short)*8-12));
			//printf("ax1=%d\nax2=%d\nvaule1=%d\n",ax1,ax2,value1);
			byte = 0x15;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x32 0x15)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x02, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x02 0x02)\n");
				return 0;
			}
			ax3 = ((linebuf[0]&0xf0)>>4)|((linebuf[1]&0xff)<<4);
			ax4 = (ax3<<(sizeof(short)*8-12));
			value2 = (ax4>>(sizeof(short)*8-12));
			//printf("ax3=%d\nax4=%d\nvaule2=%d\n",ax3,ax4,value2);
			diff = value1 - value2;
			//printf("8gx=%d\n",diff);
			if (abs(diff) < rangexy)
				result |= 1;
			byte = 0x12;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x32 0x12)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x04, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x04 0x02)\n");
				return 0;
			}
			ay1 = ((linebuf[0]&0xf0)>>4)|((linebuf[1]&0xff)<<4);
			ay2 = (ay1<<(sizeof(short)*8-12));
			value1 = (ay2>>(sizeof(short)*8-12));
			//printf("ay1=%d\nay2=%d\naule1=%d\n",ay1,ay2,value1);
			byte = 0x16;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x32 0x16)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x04, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x04 0x02)\n");
				return 0;
			}
			ay3 = ((linebuf[0]&0xf0)>>4)|((linebuf[1]&0xff)<<4);
			ay4 = (ay3<<(sizeof(short)*8-12));
			value2 = (ay4>>(sizeof(short)*8-12));
			//printf("ay3=%d\nay4=%d\nvaule2=%d\n",ay3,ay4,value2);
			diff = value1 - value2;
			//printf("8gy=%d\n",diff);
			if (abs(diff) < rangexy)
				result |= 2;
			byte = 0x13;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x32 0x13)\n");
				return 1;
			}
			if(i2c_read(0x18, 0x06, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x06 0x02)\n");
				return 0;
			}
			az1 = ((linebuf[0]&0xf0)>>4)|((linebuf[1]&0xff)<<4);
			az2 = (az1<<(sizeof(short)*8-12));
			value1 = (az2>>(sizeof(short)*8-12));
			//printf("az1=%d\naz2=%d\nvaule1=%d\n",az1,az2,value1);
			byte = 0x17;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Acceleration sensor) chip.(0x18 0x32 0x17)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x06, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Acceleration sensor) chip.(0x18 0x06 0x02)\n");
				return 0;
			}
			az3 = ((linebuf[0]&0xf0)>>4)|((linebuf[1]&0xff)<<4);
			az4 = (az3<<(sizeof(short)*8-12));
			value2 = (az4>>(sizeof(short)*8-12));
			//printf("az3=%d\naz4=%d\nvaule2=%d\n",az3,az4,value2);
			diff = value1 - value2;
			//printf("8gz=%d\n",diff);
			if (abs(diff) < rangez)
				result |= 4;
			if(result == 0x00){
				printf("PASS:Acceleration sensor test\n");
			}else{
				printf("\nFAIL:Acceleration sensor test fall!!!\n");
			}
			break;
		case Gyroscope:
			//run_command("ibus 3 0x64", 0 );
			set_i2c_bus(0x03, 0x64);
			byte = 0x00;
			if(i2c_write(0x68, 0x11, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Gyro sensor) chip.(0x68 0x11 0x00)\n");
			}
			byte = 0x80;
			if(i2c_write(0x68, 0x11, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Gyro sensor) chip.(0x68 0x11 0x80)\n");
			}
			byte = 0xb6;
			if(i2c_write(0x68, 0x14, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Gyro sensor) chip.(0x68 0x14 0xb6)\n");
			}
			if(i2c_read(0x68, 0x3c, 0x01, linebuf, 0x01) != 0) {
			puts ("Error reading the (Gyro sensor) chip.(0x68 0x11)\n");
			} else {
				if(linebuf[0]==0x10){
					byte = 0x01;
					if(i2c_write(0x68, 0x3c, 0x01, &byte, 1) != 0) {
						puts ("Error writing the (Gyro sensor) chip.(0x68 0x3c 0x01)\n");
					}
					if(i2c_read(0x68, 0x3c, 0x01, linebuf, 0x01) != 0) {
						puts ("Error reading the (Gyro sensor) chip.(0x68 0x3c)\n");
					}else {
						if(linebuf[0]==0x12){
							printf("PASS:Gyro sensor test\n");
						}
					}
				}
			}
			break;
		case Magnetic_normal:
			set_i2c_bus(0x03, 0x64);
			byte = 0x80;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic normal) chip.(0x10 0x32 0x80)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x42, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x42 0x04)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x46 0x02)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x4c, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x4c)\n");
				return 0;
			}
			byte = 0x01;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic normal) chip.(0x10 0x4b 0x01)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x4e, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x4e)\n");
				return 0;
			}			
			if(linebuf[0]==0x07){
				if(i2c_read(0x10, 0x42, 0x01, linebuf, 0x01) != 0) {
					puts ("Error reading the (Magnetic normal) chip.(0x10 0x42)\n");
				}else{
					if(linebuf[0]==0x01){
						if(i2c_read(0x10, 0x44, 0x01, linebuf, 0x01) != 0) {
							puts ("Error reading the (Magnetic normal) chip.(0x10 0x44)\n");
						}else{
							if(linebuf[0]==0x01){
								if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x01) != 0) {
									puts ("Error reading the (Magnetic normal) chip.(0x10 0x46)\n");
								}else{
									if(linebuf[0]==0x01){
										printf("PASS:Magnetic sensor normal test\n");
									}
								}
							}
						}
					}
				}
			}
			if(i2c_read(0x10, 0x4b, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x4b 0x01)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x4c, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x4c 0x01)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x43, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x43 0x01)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x45, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x45 0x01)\n");
				return 0;
			}
			if(i2c_read(0x10, 0x47, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the (Magnetic normal) chip.(0x10 0x47 0x01)\n");
				return 0;
			}
			break;
		case Magnetic_advanced:
			//run_command("ibus 3 0x64", 0 );
			set_i2c_bus(0x03, 0x64);
			byte = 0x80;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x23 0x81 0x80)\n");
				return 1;
			}

			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x04)\n");
				return 0;
			}
			byte = 0x01;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x23 0x4b 0x01)\n");
				return 1;
			}
			byte = 0x06;
			if(i2c_write(0x10, 0x4c, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x23 0x4b 0x01)\n");
				return 1;
			}
			byte = 0x1f;
			if(i2c_write(0x10, 0x4e, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x23 0x4e 0x1f)\n");
				return 1;
			}
			byte = 0x02;
			if(i2c_write(0x10, 0x52, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x23 0x4e 0x1f)\n");
				return 1;
			}
			byte = 0xc2;
			if(i2c_write(0x10, 0x4c, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x10 0x4c 0xc2)\n");
				return 1;
			}
			if(i2c_read(0x10, 0x68, 0x01, linebuf, 0x08) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x08)\n");
				return 0;
			}
			dz2 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[1])) <<8) | linebuf[0]);
			dz1 = (BMM050_U16)((((BMM050_U16)((unsigned char)linebuf[3]))<<8)|linebuf[2]);
			MZ1 = (((linebuf[5])&0x7f)>>0);
			dxyz1 = (BMM050_U16)(((BMM050_U16)((unsigned char)MZ1) <<8) | linebuf[4]);
			dz3 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[7])) <<8) | linebuf[6]);
			if(i2c_read(0x10, 0x62, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x02)\n");
				return 0;
			}
			dz4 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[1])) <<8) | linebuf[0]);
			//printf("dz4=%d\n",dz4);
			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x04)\n");
				return 0;
			}
			t1 = (((linebuf[0]&0xfe)>>1)&0x7f);
			z1 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[1]))<<7)|t1);
			t2 = ((linebuf[2]&0xfc)>>2);
			R1 = (BMM050_U16)((((BMM050_U16)linebuf[3])<<6)|t2);
			retval = (((((BMM050_S32)(z1-dz4))<<15)-((((BMM050_S32)dz3)*((BMM050_S32)(((BMM050_S16)R1)-((BMM050_S16)dxyz1))))>>2))/(dz2 +((BMM050_S16)(((((BMM050_S32)dz1)*((((BMM050_S16)R1)<<1)))+(1<<15))>>16))));
			//printf("z1=%d\nR1=%d\nretval=%d\n",z1,R1,retval);
			byte = 0x82;
			if(i2c_write(0x10, 0x4c, 0x01, &byte, 1) != 0) {
				puts ("Error writing the (Magnetic advanced) chip.(0x10 0x4c 0x82)\n");
				return 1;
			}
			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x04)\n");
				return 0;
			}
			t3 = (((linebuf[0]&0xfe)>>1)&0x7f);
			z2 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[1]))<<7)|t3);
			t4 = ((linebuf[2]&0xfc)>>2);
			R2 = (BMM050_U16)((((BMM050_U16)linebuf[3])<<6)|t4);
			if(i2c_read(0x10, 0x68, 0x01, linebuf, 0x08) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x08)\n");
				return 0;
			}
			ddz2 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[1])) <<8) | linebuf[0]);
			ddz1 = (BMM050_U16)((((BMM050_U16)((unsigned char)linebuf[3]))<<8) | linebuf[2]);
			MZ2 = (((linebuf[5])&0x7f)>>0);
			ddxyz1 = (BMM050_U16)(((BMM050_U16)((unsigned char)MZ2) <<8) | linebuf[4]);
			ddz3 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[7])) <<8) | linebuf[6]);
			if(i2c_read(0x10, 0x62, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x01)\n");
				return 0;
			}
			ddz4 = (BMM050_S16)((((BMM050_S16)((signed char)linebuf[1])) <<8) | linebuf[0]);
			retval2 = (((((BMM050_S32)(z2-dz4))<<15)-((((BMM050_S32)dz3)*((BMM050_S32)(((BMM050_S16)R2)-((BMM050_S16)dxyz1))))>>2))/(dz2+((BMM050_S16)(((((BMM050_S32)dz1)*((((BMM050_S16)R2)<<1)))+(1<<15))>>16))));
			//printf("ddz4=%d\nz2=%d\nR2=%d\nretval2=%d\n",ddz4,z2,R2,retval2);
			if(i2c_read(0x10, 0x48, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Magnetic advanced) chip.(0x10 0x46 0x02)\n");
				return 0;
			}
			z0 = retval-retval2;
			//printf("z0=%d\n",z0);
			if(2880 <= z0 <= 3840){
				printf("PASS:Magnetic sensor advanced test\n");
			}
			break;

		case ALS_static:

			set_i2c_bus(0x03, 0x64);

			byte = 0x09;
			if(i2c_write(0x29, 0x85, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(set Light sensor)\n");
				return 1;
			}
			byte = 0x1d;
			if(i2c_write(0x29, 0x80, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Enable Light sensor)\n");
				return 1;
			}
			if(i2c_read(0x29, 0x88, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the (Light sensor) chip.\n");
				return 0;
			}
			ch1_adc = (linebuf[1]<<8) | linebuf[0];
			ch0_adc = (linebuf[3]<<8) | linebuf[2];

			byte = 0x1c;
			if(i2c_write(0x29, 0x80, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Disable Light sensor)\n");
				return 1;
			}

			if ((ch1_adc + ch0_adc) == 0) {
				ratio = 100;
			} else {
				ratio = (100 * ch1_adc)/(ch1_adc + ch0_adc);
			}

			if (ratio < 45) {
				ch0_coeff = 17743;
				ch1_coeff = 11059;
			} else if ((ratio >= 45) && (ratio < 64)) {
				ch0_coeff = 42785;
				ch1_coeff = -19548;
			} else if ((ratio >= 64) && (ratio < 85)) {
				ch0_coeff = 5926;
				ch1_coeff = 1185;
			} else if (ratio >= 85) {
				ch0_coeff = 0;
				ch1_coeff = 0;
			}

			luxval = (((ch0_adc * ch0_coeff) + (ch1_adc * ch1_coeff))/10000)/48;
			printf("PASS : Light sensor %01dLux (Ch1:0x%04x,Ch0:0x%04x)\n", luxval, ch1_adc, ch0_adc);

			break;
		case ALS_dynamic:
			set_i2c_bus(0x03, 0x64);
			byte = 0x09;
			if(i2c_write(0x29, 0x85, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Set Light sensor)\n");
				return 1;
			}
			byte = 0x1d;
			if(i2c_write(0x29, 0x80, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Enable Light sensor)\n");
				return 1;
			}

			delaytime = 120;
			testflag = 0;
			printf("Light sensor test.");
			while(delaytime){
				if(i2c_read(0x29, 0x88, 0x01, linebuf, 0x04) != 0) {
					puts ("Error reading the (Light sensor) chip.\n");
					return 0;
				}
				ch1_adc = (linebuf[1]<<8) | linebuf[0];
				ch0_adc = (linebuf[3]<<8) | linebuf[2];

				if ((ch1_adc + ch0_adc) == 0) {
					ratio = 100;
				} else {
					ratio = (100 * ch1_adc)/(ch1_adc + ch0_adc);
				}

				if (ratio < 45) {
					ch0_coeff = 17743;
					ch1_coeff = 11059;
				} else if ((ratio >= 45) && (ratio < 64)) {
					ch0_coeff = 42785;
					ch1_coeff = -19548;
				} else if ((ratio >= 64) && (ratio < 85)) {
					ch0_coeff = 5926;
					ch1_coeff = 1185;
				} else if (ratio >= 85) {
					ch0_coeff = 0;
					ch1_coeff = 0;
				}

				luxval = (((ch0_adc * ch0_coeff) + (ch1_adc * ch1_coeff))/10000)/48;
				if(luxval > ALS_Up)
					testflag |= 0x02;

				if(luxval < ALS_Low)
					testflag |= 0x01;

				if(testflag == 0x03){
					delaytime = 1;
				}

				for(delaycnt=0; delaycnt<25; ++delaycnt){ udelay(10000);}
				delaytime--;
				puts(".");
			}

			byte = 0x1c;
			if(i2c_write(0x29, 0x80, 0x01, &byte, 1) != 0) {
				puts ("\nError writing the chip.(Disable Light sensor)\n");
				return 1;
			}

			if(testflag == 0x03){
				printf("\nPASS:Light sensor test\n");
			}else{
				printf("\nFAIL:Light sensor test time out!!!\n");
			}
			break;
		case Proximity_static:
			set_i2c_bus(0x03, 0x64);
			byte = 0x23;
			if(i2c_write(0x23, 0x81, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Enable Proximity sensor)\n");
				return 1;
			}

			if(i2c_read(0x23, 0x8d, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the (Proximity sensor) chip.\n");
				return 0;
			}

			byte = 0x0;
			if(i2c_write(0x23, 0x81, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Disable Proximity sensor)\n");
				return 1;
			}

			printf("PASS : Proximity sensor ");
			if(linebuf[0] || linebuf[1]){
				printf("Near(0x%02x%02x)\n",linebuf[1], linebuf[0]);
			}else{
				printf("Far\n");
			}

			break;
		case Proximity_dynamic:
			set_i2c_bus(0x03, 0x64);
			byte = 0x23;
			if(i2c_write(0x23, 0x81, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(Enable Proximity sensor)\n");
				return 1;
			}
			delaytime = 120;
			testflag = 0;
			printf("Proximity sensor test.");
			while(delaytime){
				if(i2c_read(0x23, 0x8d, 0x01, linebuf, 0x02) != 0) {
					puts ("Error reading the (Proximity sensor) chip.\n");
					return 0;
				}

				if(linebuf[0] || linebuf[1]){
					testflag |= 0x02;
				}else{
					testflag |= 0x01;
				}

				if(testflag == 0x03){
					delaytime = 1;
				}
				for(delaycnt=0; delaycnt<25; ++delaycnt){ udelay(10000);}
				delaytime--;
				puts(".");
			}

			byte = 0x0;
			if(i2c_write(0x23, 0x81, 0x01, &byte, 1) != 0) {
				puts ("\nError writing the chip.(Disable Proximity sensor)\n");
				return 1;
			}

			if(testflag == 0x03){
				printf("\nPASS:Proximity sensor test\n");
			}else{
				printf("\nFAIL:Proximity sensor test time out!!!\n");
			}
			break;
	}
	return 0;
}

int do_dctpwbtn (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned char	linebuf[2];
	unsigned char	delaycnt = 0;
	unsigned char	ledflag = 1;

	run_command("ledset 1", 0);
	set_i2c_bus(0x00, 0x64);
	while(1){
		if(i2c_read(PW_BTN_CHIP, BTN_ADDR, 0x01, linebuf, 0x01) != 0) {
			puts ("Error reading the chip.\n");
		} else {
		if(linebuf[0]&0x01)
				printf(".");
			else
				break;
		}
		for(delaycnt=0; delaycnt<20; ++delaycnt){ udelay(10000);}

		ledflag ^= 1;
		if(ledflag) {
			run_command("mw 0x4805b13c 08000000", 0);
		}
		else{
			run_command("mw 0x4805b13c 00000000", 0);
		}
	}
	printf("\nPASS:Power Button test\n");
	run_command("ledset 0", 0);

	return 0;
}

int do_dctibus (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned char	linebuf[2];
	unsigned char	byte;
	unsigned char	channel;
	int j;

	set_i2c_bus(0x02, 0x64);
	if(i2c_read(0x20, 0x06, 0x01, linebuf, 0x01) != 0) {
		puts ("Error reading the (Proximity sensor) chip.\n");
		return 0;
	}

	byte = 0x00;
	if(i2c_write(0x21, 0x07, 0x01, &byte, 1) != 0) {
		puts ("Error writing the chip.(0x21 0x07 0x00)\n");
		return 1;
	}

	byte = 0xcd;
	if(i2c_write(0x21, 0x02, 0x01, &byte, 1) != 0) {
		puts ("Error writing the chip.(0x21 0x02 0xcd)\n");
		return 1;
	}

	byte = 0x00;
	if(i2c_write(0x21, 0x06, 0x01, &byte, 1) != 0) {
		puts ("Error writing the chip.(0x21 0x06 0x00)\n");
		return 1;
	}
	
	run_command("mw 4a008154 00000108", 0);
	run_command("mw 4a30a314 00070104", 0);
	byte = 0xfb;
	if(i2c_write(0x21, 0x03, 0x01, &byte, 1) != 0) {
		puts ("Error writing the chip.(0x21 0x03 0xfb)\n");
		return 1;
	}

	channel = 0x00;
	set_i2c_bus(channel, 0x64);
	for(j = 0; j < 128; j++) {
		if(i2c_probe(j) == 0) {
			switch(j){
				case 0x48:
					printf("[%01x.%02X]:Power chip.\n", channel, j);
					break;
				case 0x49:
					printf("[%01x.%02X]:Power chip.\n", channel, j);
					break;
				case 0x4A:
					printf("[%01x.%02X]:Power chip.\n", channel, j);
					break;
				case 0x4B:
					printf("[%01x.%02X]:Power chip.\n", channel, j);
					break;
				case 0x6A:
					printf("[%01x.%02X]:Charging chip.\n", channel, j);
					break;
				default:
					printf("[%01x.%02X]:Unknow chip.\n", channel, j);
					break;
			}
		}
	}

	channel = 0x01;
	set_i2c_bus(channel, 0x64);
	for(j = 0; j < 128; j++) {
		if(i2c_probe(j) == 0) {
			switch(j){
				case 0x10:
					printf("[%01x.%02X]:Camera chip.\n", channel, j);
					break;
				default:
					printf("[%01x.%02X]:unknow chip.\n", channel, j);
					break;
			}
		}
	}

	channel = 0x02;
	set_i2c_bus(channel, 0x64);
	for(j = 0; j < 128; j++) {
		if(i2c_probe(j) == 0) {
			switch(j){
				case 0x20:
					printf("[%01x.%02X]:I/O Expander.\n", channel, j);
					break;
				case 0x21:
					printf("[%01x.%02X]:I/O Expander.\n", channel, j);
					break;
				case 0x46:
					printf("[%01x.%02X]:Touch-IC.\n", channel, j);
					break;
				default:
					printf("[%01x.%02X]:Unknow chip.\n", channel, j);
					break;
			}
		}
	}

	channel = 0x03;
	set_i2c_bus(channel, 0x64);
	for(j = 0; j < 128; j++) {
		if(i2c_probe(j) == 0) {
			switch(j){
				case 0x10:
					printf("[%01x.%02X]:Magnetic sensor.\n", channel, j);
					break;
				case 0x18:
					printf("[%01x.%02X]:Acceleration sensor.\n", channel, j);
					break;
				case 0x23:
					printf("[%01x.%02X]:Proximity sensor.\n", channel, j);
					break;
				case 0x29:
					printf("[%01x.%02X]:Light sensor.\n", channel, j);
					break;
				case 0x48:
					printf("[%01x.%02X]:Temperature sensor.\n", channel, j);
					break;
				case 0x68:
					printf("[%01x.%02X]:Gyroscope sensor.\n", channel, j);
					break;
				default:
					printf("[%01x.%02X]:Unknow chip.\n", channel, j);
					break;
			}
		}
	}
	printf("End:decect i2c devices.\n");
}

int do_setals (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if (argc == 1) {
		printf ("Light sensor test value Up=%d, Low=%d\n", ALS_Up, ALS_Low);
	} else if (argc == 3) {
		ALS_Low = simple_strtoul(argv[1], NULL, 10);
		ALS_Up = simple_strtoul(argv[2], NULL, 10);
	} else {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	return 0;
}


U_BOOT_CMD(
	ledset ,	2,	2,	 do_ledset,
	"ledset  - set LED status\n",
	"N\n"
	"    - set RGB LED status (N is _decimal_ !!!)\n"
	"    - 0:LED Off\n"
	"    - 1:LED On\n"
#if 0
	"    - 2:LED flickering\n"
#endif
);

U_BOOT_CMD(
	st,	2,	2,	 do_st,
	"st      - sensor test\n",
	"N\n"
	"    - set sensor test (N is _decimal_ !!!)\n"
	"    - 1 Acceleration test \n"
	"    - 2 Gyro test\n"
	"    - 3 Magnetic Normal test \n"
	"    - 4 Magnetic Advanced test \n"
	"    - 5 sensor test all \n"
);


U_BOOT_CMD(
	dctpwbtn,	1,	2,	do_dctpwbtn,
	"dctpwbtn- decect Power Button status\n",
	"    -decect Power Button status\n"
);

U_BOOT_CMD(
	dctibus,	1,	2,	do_dctibus,
	"dctibus - decect i2c devices\n",
	"    -decect i2c devices\n"
);

U_BOOT_CMD(
	setals,	3,	2,	 do_setals,
	"setals  - set light sensor test value\n",
	" low_value up_value\n"
);
#endif	/* CONFIG_BOARD_TEST */
