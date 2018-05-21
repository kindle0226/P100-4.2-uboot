/*
 * (C) Copyright 2001
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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

/*
 * Misc functions
 */
#include <common.h>
#include <command.h>

#if (CONFIG_COMMANDS & CFG_CMD_MISC)

int do_sleep (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong start = get_timer(0);
	ulong delay;

	if (argc != 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	delay = simple_strtoul(argv[1], NULL, 10) * CFG_HZ;

	while (get_timer(start) < delay) {
		if (ctrlc ()) {
			return (-1);
		}
		udelay (100);
	}

	return 0;
}

#if 0
int do_st (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    unsigned char	linebuf[8];
    unsigned char	byte;
    ulong input;
    
	if (argc != 2) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	input = simple_strtoul(argv[1], NULL, 10);
  
	switch(input){
		case 1:
			printf("Acceleration test\n");
			run_command("ibus 3 0x64", 0 );
			//run_command("ibus 3 0x64", 0 );
			//run_command("imw 0x18 0x14 0xb6", 0 );
			//run_command("imd 0x18 0x02 0x01", 0 );
			//run_command("imd 0x18 0x03 0x01", 0 );
			//run_command("imd 0x18 0x04 0x01", 0 );
			//run_command("imd 0x18 0x05 0x01", 0 );
			//run_command("imd 0x18 0x06 0x01", 0 );
			//run_command("imd 0x18 0x07 0x01", 0 );
			//run_command("imw 0x18 0x0f 0x08", 0 );
			//run_command("imd 0x18 0x0f 0x01", 0 );
			byte = 0xb6;
			if(i2c_write(0x18, 0x14, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x14 0xb6)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x02, 0x01, linebuf, 0x06) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x02)\n");
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
				printf("0x%02x%02x\n",linebuf[3],linebuf[2]);
				printf("0x%02x%02x\n",linebuf[5],linebuf[4]);
			}
			byte = 0x08;
			if(i2c_write(0x18, 0x0f, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x0f 0x08)\n");
				return 1;
			}
			//run_command("imw 0x18 0x32 0x11", 0 );
			//run_command("imd 0x18 0x32 0x01", 0 );
			//run_command("imd 0x18 0x02 0x01", 0 );
			//run_command("imd 0x18 0x03 0x01", 0 );
			//run_command("imw 0x18 0x32 0x15", 0 );
			//run_command("imd 0x18 0x32 0x01", 0 );
			//run_command("imd 0x18 0x02 0x01", 0 );
			//run_command("imd 0x18 0x03 0x01", 0 );
			byte = 0x11;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x32 0x11)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x02, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x02)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
			}
			byte = 0x15;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x32 0x15)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x02, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x02)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
			}
			//run_command("imw 0x18 0x32 0x12", 0 );
			//run_command("imd 0x18 0x32 0x01", 0 );
			//run_command("imd 0x18 0x04 0x01", 0 );
			//run_command("imd 0x18 0x05 0x01", 0 );
			//run_command("imw 0x18 0x32 0x16", 0 );
			//run_command("imd 0x18 0x32 0x01", 0 );
			//run_command("imd 0x18 0x04 0x01", 0 );
			//run_command("imd 0x18 0x05 0x01", 0 );
			byte = 0x12;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x32 0x12)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x04, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x04)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
			}
			byte = 0x16;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x32 0x16)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x04, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x04)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
			}		
			//run_command("imw 0x18 0x32 0x13", 0 );
			//run_command("imd 0x18 0x32 0x01", 0 );
			//run_command("imd 0x18 0x06 0x01", 0 );
			//run_command("imd 0x18 0x07 0x01", 0 );
			//run_command("imw 0x18 0x32 0x17", 0 );
			//run_command("imd 0x18 0x32 0x01", 0 );
			//run_command("imd 0x18 0x06 0x01", 0 );
			//run_command("imd 0x18 0x07 0x01", 0 );
			
			byte = 0x13;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x32 0x13)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x06, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x06)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
			}
			
			byte = 0x17;
			if(i2c_write(0x18, 0x32, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x18 0x32 0x17)\n");
				return 1;
			}

			if(i2c_read(0x18, 0x06, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.(imd 0x18 0x06)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\nPass\n",linebuf[1],linebuf[0]);
			}	
			break;
		case 2:
			printf("Gyro test\n");
			run_command("ibus 3 0x64", 0 );
			//run_command("imw 0x68 0x11 0x00", 0 );
			//printf("1\n");
			byte = 0x00;
			if(i2c_write(0x68, 0x11, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.\n");
			}
			//udelay(11000);
			
			//run_command("imw 0x68 0x11 0x80", 0 );
			//printf("2\n");
			byte = 0x80;
			if(i2c_write(0x68, 0x11, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.\n");
			}
			//udelay(11000);
			
			//run_command("imw 0x68 0x14 0xb6", 0 );
			//printf("3\n");
			byte = 0xb6;
			if(i2c_write(0x68, 0x14, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.\n");
			}
			//udelay(11000);
			//printf("4\n");
			
			//run_command("imd 0x68 0x3c 0x01", 0 );
			if(i2c_read(0x68, 0x3c, 0x01, linebuf, 0x01) != 0) {
			puts ("Error reading the chip.\n");
			} else {
				if(linebuf[0]==0x10){
					//run_command("imw 0x68 0x3c 0x01", 0 );
					byte = 0x01;
					if(i2c_write(0x68, 0x3c, 0x01, &byte, 1) != 0) {
					puts ("Error writing the chip.\n");
					}
						if(i2c_read(0x68, 0x3c, 0x01, linebuf, 0x01) != 0) {
						puts ("Error reading the chip.\n");
						}else {
							if(linebuf[0]==0x12){
							printf("PASS\n");
							}
						}
				}
			}
		
			//run_command("imw 0x68 0x3c 0x01", 0 );
			//run_command("imd 0x68 0x3c 0x01", 0 );
			break;
		case 3:
			printf("Magnetic Normal test\n");
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x10 0x4b 0x80", 0 );

			run_command("imd 0x10 0x42 0x01", 0 );
			run_command("imd 0x10 0x43 0x01", 0 );
			run_command("imd 0x10 0x44 0x01", 0 );
			run_command("imd 0x10 0x45 0x01", 0 );

			run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );
			run_command("imd 0x10 0x4c 0x01", 0 );

			//run_command("imw 0x10 0x4b 0x01", 0 );
			run_command("imd 0x10 0x4b 0x01", 0 );
			//run_command("imd 0x10 0x4e 0x01", 0 );
			run_command("imd 0x10 0x4c 0x01", 0 );

			//run_command("imd 0x10 0x42 0x01", 0 );
			run_command("imd 0x10 0x43 0x01", 0 );
			//run_command("imd 0x10 0x44 0x01", 0 );
			run_command("imd 0x10 0x45 0x01", 0 );
			//run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );
			byte = 0x01;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.\n");
				return 0;
			}
			
			if(i2c_read(0x10, 0x4e, 0x01, linebuf, 0x01) != 0) {
				puts ("Error reading the chip.\n");
				return 0;
			}
			
			if(linebuf[0]==0x07){
				if(i2c_read(0x10, 0x42, 0x01, linebuf, 0x01) != 0) {
					puts ("Error reading the chip.\n");
				}else{
					if(linebuf[0]==0x01){
						if(i2c_read(0x10, 0x44, 0x01, linebuf, 0x01) != 0) {
							puts ("Error reading the chip.\n");
						}else{
							if(linebuf[0]==0x01){
								if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x01) != 0) {
									puts ("Error reading the chip.\n");
								}else{
									//printf("R:0x%02x\n",linebuf[0]);
									if(linebuf[0]==0x01){
										printf("PASS\n");
									}	
								}
							}
						}
					}
				}
			}
							
			
			
			
			break;
		case 4:
			printf("Magnetic Advanced test\n");
			run_command("ibus 3 0x64", 0 );
			//run_command("imw 0x10 0x4b 0x80", 0 );
			//run_command("imd 0x10 0x46 0x01", 0 );
			//run_command("imd 0x10 0x47 0x01", 0 );
			//run_command("imd 0x10 0x48 0x01", 0 );
			//run_command("imd 0x10 0x49 0x01", 0 );
			byte = 0x80;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x23 0x81 0x80)\n");
				return 1;
			}

			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the chip.(imd 0x10 0x46)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
				printf("0x%02x%02x\n",linebuf[3],linebuf[2]);
			}
	
			//run_command("imw 0x10 0x4b 0x01", 0 );
			//run_command("imd 0x10 0x4b 0x01", 0 );
			//run_command("imw 0x10 0x4e 0x1f", 0 );
			//run_command("imw 0x10 0x4c 0xc2", 0 );
			byte = 0x01;
			if(i2c_write(0x10, 0x4b, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x23 0x4b 0x01)\n");
				return 1;
			}
			byte = 0x1f;
			if(i2c_write(0x10, 0x4e, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x23 0x4e 0x1f)\n");
				return 1;
			}
			byte = 0xc2;
			if(i2c_write(0x10, 0x4c, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x10 0x4c 0xc2)\n");
				return 1;
			}
	
			//run_command("imd 0x10 0x46 0x01", 0 );
			//run_command("imd 0x10 0x47 0x01", 0 );
			//run_command("imd 0x10 0x48 0x01", 0 );
			//run_command("imd 0x10 0x49 0x01", 0 );
			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the chip.(imd 0x10 0x46)\n");
				return 0;
			}	
			//run_command("imw 0x10 0x4c 0x82", 0 );
			//run_command("imd 0x10 0x46 0x01", 0 );
			//run_command("imd 0x10 0x47 0x01", 0 );
			//run_command("imd 0x10 0x48 0x01", 0 );
			//run_command("imd 0x10 0x49 0x01", 0 );
			byte = 0x82;
			if(i2c_write(0x10, 0x4c, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x10 0x4c 0x82)\n");
				return 1;
			}
			if(i2c_read(0x10, 0x46, 0x01, linebuf, 0x04) != 0) {
				puts ("Error reading the chip.(imd 0x10 0x46)\n");
				return 0;
			}
			else{
				printf("0x%02x%02x\n",linebuf[1],linebuf[0]);
				printf("0x%02x%02x\nPass\n",linebuf[3],linebuf[2]);
			}			
			break;
		case 5:
			printf("Acceleration test\n");
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x18 0x14 0xb6", 0 );
			run_command("imd 0x18 0x02 0x01", 0 );
			run_command("imd 0x18 0x03 0x01", 0 );
			run_command("imd 0x18 0x04 0x01", 0 );
			run_command("imd 0x18 0x05 0x01", 0 );
			run_command("imd 0x18 0x06 0x01", 0 );
			run_command("imd 0x18 0x07 0x01", 0 );
			run_command("imw 0x18 0x0f 0x08", 0 );
			run_command("imd 0x18 0x0f 0x01", 0 );

			run_command("imw 0x18 0x32 0x11", 0 );
			run_command("imd 0x18 0x32 0x01", 0 );
			run_command("imd 0x18 0x02 0x01", 0 );
			run_command("imd 0x18 0x03 0x01", 0 );
			run_command("imw 0x18 0x32 0x15", 0 );
			run_command("imd 0x18 0x32 0x01", 0 );
			run_command("imd 0x18 0x02 0x01", 0 );
			run_command("imd 0x18 0x03 0x01", 0 );

			run_command("imw 0x18 0x32 0x12", 0 );
			run_command("imd 0x18 0x32 0x01", 0 );
			run_command("imd 0x18 0x04 0x01", 0 );
			run_command("imd 0x18 0x05 0x01", 0 );
			run_command("imw 0x18 0x32 0x16", 0 );
			run_command("imd 0x18 0x32 0x01", 0 );
			run_command("imd 0x18 0x04 0x01", 0 );
			run_command("imd 0x18 0x05 0x01", 0 );
	
			run_command("imw 0x18 0x32 0x13", 0 );
			run_command("imd 0x18 0x32 0x01", 0 );
			run_command("imd 0x18 0x06 0x01", 0 );
			run_command("imd 0x18 0x07 0x01", 0 );
			run_command("imw 0x18 0x32 0x17", 0 );
			run_command("imd 0x18 0x32 0x01", 0 );
			run_command("imd 0x18 0x06 0x01", 0 );
			run_command("imd 0x18 0x07 0x01", 0 );
		
			printf("Gyro test\n");
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x68 0x11 0x00", 0 );
			run_command("imw 0x68 0x11 0x80", 0 );
			run_command("imw 0x68 0x14 0xb6", 0 );
			run_command("imd 0x68 0x3c 0x01", 0 );
			run_command("imw 0x68 0x3c 0x01", 0 );
			run_command("imd 0x68 0x3c 0x01", 0 );
		
			printf("Magnetic Normal test\n");
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x10 0x4b 0x80", 0 );
	
			run_command("imd 0x10 0x42 0x01", 0 );
			run_command("imd 0x10 0x43 0x01", 0 );
			run_command("imd 0x10 0x44 0x01", 0 );
			run_command("imd 0x10 0x45 0x01", 0 );
	
			run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );
			run_command("imd 0x10 0x4c 0x01", 0 );
	
			run_command("imw 0x10 0x4b 0x01", 0 );
			run_command("imd 0x10 0x4b 0x01", 0 );
			run_command("imd 0x10 0x4e 0x01", 0 );
			run_command("imd 0x10 0x4c 0x01", 0 );
	
			run_command("imd 0x10 0x42 0x01", 0 );
			run_command("imd 0x10 0x43 0x01", 0 );
			run_command("imd 0x10 0x44 0x01", 0 );
			run_command("imd 0x10 0x45 0x01", 0 );
			run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );	
		
			printf("Magnetic Advanced test\n");
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x10 0x4b 0x80", 0 );
			run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );
			run_command("imd 0x10 0x48 0x01", 0 );
			run_command("imd 0x10 0x49 0x01", 0 );
	
			run_command("imw 0x10 0x4b 0x01", 0 );
			run_command("imd 0x10 0x4b 0x01", 0 );
			run_command("imw 0x10 0x4e 0x1f", 0 );
			run_command("imw 0x10 0x4c 0xc2", 0 );
	
			run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );
			run_command("imd 0x10 0x48 0x01", 0 );
			run_command("imd 0x10 0x49 0x01", 0 );
	
			run_command("imw 0x10 0x4c 0x82", 0 );
			run_command("imd 0x10 0x46 0x01", 0 );
			run_command("imd 0x10 0x47 0x01", 0 );
			run_command("imd 0x10 0x48 0x01", 0 );
			run_command("imd 0x10 0x49 0x01", 0 );
			break;
		case 6:
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x29 0x80 0x01", 0 ); //enable
			run_command("imd 0x29 0x88 0x01", 0 ); //ALS  data0 ch1 low byte address
			run_command("imd 0x29 0x89 0x01", 0 ); //ALS  data1 ch1 high byte address
			run_command("imd 0x29 0x8a 0x01", 0 ); //ALS  data2 ch0 low byte address
			run_command("imd 0x29 0x8b 0x01", 0 ); //ALS  data3 ch0 high byte address
			
			//ALS_CH1_ADC_Data = (Data1 << 8) | Data0 // Combining lower and upper bytes to give 16-bit Ch1 data
			//ALS_CH0_ADC_Data = (Data3 << 8) | Data2 // Combining lower and upper bytes to give 16-bit Ch0 data
			break;
		case 7:
#if 0		
			run_command("ibus 3 0x64", 0 );
			run_command("imw 0x23 0x81 0x23", 0 ); //enable
			run_command("imd 0x23 0x8d 0x01", 0 ); //PS  data0 low byte address
			run_command("imd 0x23 0x8e 0x01", 0 ); //PS  data1 high byte address
#else			
			//PS_ADC_Data = (Data1 << 8) | Data0 // Combining lower and upper bytes to give 16-bit PS data

			if(select_bus(0x03, 0x64)){
				printf("ibus set FAILED\n");
				return 1;
			}

			byte = 0x23;
			if(i2c_write(0x23, 0x81, 0x01, &byte, 1) != 0) {
				puts ("Error writing the chip.(imw 0x23 0x81 0x23)\n");
				return 1;
			}

			if(i2c_read(0x23, 0x8d, 0x01, linebuf, 0x02) != 0) {
				puts ("Error reading the chip.\n");
				return 0;
			}
			if(linebuf[0]||linebuf[1]){
				printf("0x%02x%02x Pass\n",linebuf[1],linebuf[0]);
			}
#endif
			break;
	}
	return 0;

}
#endif


/* Implemented in $(CPU)/interrupts.c */
#if (CONFIG_COMMANDS & CFG_CMD_IRQ)
int do_irqinfo (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

U_BOOT_CMD(
	irqinfo,    1,    1,     do_irqinfo,
	"irqinfo - print information about IRQs\n",
	NULL
);
#endif  /* CONFIG_COMMANDS & CFG_CMD_IRQ */

U_BOOT_CMD(
	sleep ,    2,    2,     do_sleep,
	"sleep   - delay execution for some time\n",
	"N\n"
	"    - delay execution for N seconds (N is _decimal_ !!!)\n"
);

/*U_BOOT_CMD(
	st,    2,    2,     do_st,
	"st      - sensor test\n",
	"N\n"
	"	 - set sensor test (N is _decimal_ !!!)\n"
	"	 - 1 Acceleration test \n"
	"	 - 2 Gyro test\n"
	"	 - 3 Magnetic Normal test \n"
	"	 - 4 Magnetic Advanced test \n"
	"	 - 5 sensor test all \n"
);*/


#endif	/* CFG_CMD_MISC */
