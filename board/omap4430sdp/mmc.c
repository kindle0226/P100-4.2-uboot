/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
#include <asm/io.h>

#include <common.h>
#include <mmc.h>
#include <fastboot.h>

#define EFI_VERSION 0x00010000
#define EFI_ENTRIES 128
#define EFI_NAMELEN 36

/* Default to eMMC slot for omap4430sdp Blaze and Blaze Tablet */
int mmc_slot = 1;

static const u8 partition_type[16] = {
	0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,
	0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7,
};

static const u8 random_uuid[16] = {
	0xff, 0x1f, 0xf2, 0xf9, 0xd4, 0xa8, 0x0e, 0x5f,
	0x97, 0x46, 0x59, 0x48, 0x69, 0xae, 0xc3, 0x4e,
};
	
struct efi_entry {
	u8 type_uuid[16];
	u8 uniq_uuid[16];
	u64 first_lba;
	u64 last_lba;
	u64 attr;
	u16 name[EFI_NAMELEN];
};

struct efi_header {
	u8 magic[8];

	u32 version;
	u32 header_sz;

	u32 crc32;
	u32 reserved;

	u64 header_lba;
	u64 backup_lba;
	u64 first_lba;
	u64 last_lba;

	u8 volume_uuid[16];

	u64 entries_lba;

	u32 entries_count;
	u32 entries_size;
	u32 entries_crc32;
} __attribute__((packed));

struct ptable {
	u8 mbr[512];
	union {
		struct efi_header header;
		u8 block[512];
	};
	struct efi_entry entry[EFI_ENTRIES];	
};

int board_set_flash_slot(char * slot_name)
{
	int ret = 0;
	if (!strcmp(slot_name, "SD"))
		mmc_slot = 0;
	else if (!strcmp(slot_name, "EMMC"))
		mmc_slot = 1;
	else
		ret = -1;

	return(ret);
}

static void init_mbr(u8 *mbr, u32 blocks)
{
	mbr[0x1be] = 0x00; // nonbootable
	mbr[0x1bf] = 0xFF; // bogus CHS
	mbr[0x1c0] = 0xFF;
	mbr[0x1c1] = 0xFF;

	mbr[0x1c2] = 0xEE; // GPT partition
	mbr[0x1c3] = 0xFF; // bogus CHS
	mbr[0x1c4] = 0xFF;
	mbr[0x1c5] = 0xFF;

	mbr[0x1c6] = 0x01; // start
	mbr[0x1c7] = 0x00;
	mbr[0x1c8] = 0x00;
	mbr[0x1c9] = 0x00;

	memcpy(mbr + 0x1ca, &blocks, sizeof(u32));

	mbr[0x1fe] = 0x55;
	mbr[0x1ff] = 0xaa;
}

static void start_ptbl(struct ptable *ptbl, unsigned blocks)
{
	struct efi_header *hdr = &ptbl->header;

	memset(ptbl, 0, sizeof(*ptbl));

	init_mbr(ptbl->mbr, blocks - 1);

	memcpy(hdr->magic, "EFI PART", 8);
	hdr->version = EFI_VERSION;
	hdr->header_sz = sizeof(struct efi_header);
	hdr->header_lba = 1;
	hdr->backup_lba = blocks - 1;
	hdr->first_lba = 34;
	hdr->last_lba = blocks - 1;
	memcpy(hdr->volume_uuid, random_uuid, 16);
	hdr->entries_lba = 2;
	hdr->entries_count = EFI_ENTRIES;
	hdr->entries_size = sizeof(struct efi_entry);
}

static void end_ptbl(struct ptable *ptbl)
{
	struct efi_header *hdr = &ptbl->header;
	u32 n;

	n = crc32(0, 0, 0);
	n = crc32(n, (void*) ptbl->entry, sizeof(ptbl->entry));
	hdr->entries_crc32 = n;

	n = crc32(0, 0, 0);
	n = crc32(0, (void*) &ptbl->header, sizeof(ptbl->header));
	hdr->crc32 = n;
}

int add_ptn(struct ptable *ptbl, u64 first, u64 last, const char *name)
{
	struct efi_header *hdr = &ptbl->header;
	struct efi_entry *entry = ptbl->entry;
	unsigned n;

	if (first < 34) {
		printf("partition '%s' overlaps partition table\n", name);
		return -1;
	}

	if (last > hdr->last_lba) {
		printf("partition '%s' does not fit\n", name);
		return -1;
	}
	for (n = 0; n < EFI_ENTRIES; n++, entry++) {
		if (entry->last_lba)
			continue;
		memcpy(entry->type_uuid, partition_type, 16);
		memcpy(entry->uniq_uuid, random_uuid, 16);
		entry->uniq_uuid[0] = n;
		entry->first_lba = first;
		entry->last_lba = last;
		for (n = 0; (n < EFI_NAMELEN) && *name; n++)
			entry->name[n] = *name++;
		return 0;
	}
	printf("out of partition table entries\n");
	return -1;
}

void import_efi_partition(struct efi_entry *entry)
{
	struct fastboot_ptentry e;
	int n;
	if (memcmp(entry->type_uuid, partition_type, sizeof(partition_type)))
		return;
	for (n = 0; n < (sizeof(e.name)-1); n++)
		e.name[n] = entry->name[n];
	e.name[n] = 0;
	e.start = entry->first_lba;
	e.length = (entry->last_lba - entry->first_lba + 1) * 512;
	e.flags = 0;

	if (!strcmp(e.name,"environment"))
		e.flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_ENV;
	fastboot_flash_add_ptn(&e);

	if (e.length > 0x100000)
		printf("%8d %7dM %s\n", e.start,
			(u32)(e.length/0x100000), e.name);
	else
		printf("%8d %7dK %s\n", e.start,
			(u32)(e.length/0x400), e.name);
}

static int load_ptbl(void)
{
	static unsigned char data[512];
	static struct efi_entry entry[4];
	int n,m,r;
	printf("ptbl slot: %s:(%d).\n",
			mmc_slot?"EMMC":"SD", mmc_slot);
	r = mmc_read(mmc_slot, 1, data, 512);
	if (r != 1) {
		printf("error reading partition table\n");
		return -1;
	}
	if (memcmp(data, "EFI PART", 8)) {
		printf("efi partition table not found\n");
		return -1;
	}
	for (n = 0; n < (128/4); n++) {
		r = mmc_read(mmc_slot, 1 + n, (void*) entry, 512);
		if (r != 1) {
			printf("partition read failed\n");
			return 1;
		}
		for (m = 0; m < 4; m ++)
			import_efi_partition(entry + m);
	}
	return 0;
}

struct partition {
	const char *name;
	unsigned size_kb;
};

static struct partition partitions[] = {
	{ "-", 128 },
	{ "xloader", 128 },
	{ "bootloader", 2304 },
	/* "misc" partition is required for recovery */
	{ "misc", 128 },
	{ "-", 384 },
	{ "efs", 16384 },
	{ "firmware", 1024},
	{ "crypto", 16 },
	{ "recovery", 8*1024 },
	{ "boot", 8*1024 },
	{ "system", 512*1024 },
	{ "cache", 256*1024 },
	{ "userdata", 0},
	{ 0, 0 },
};

static struct ptable the_ptable;

static int do_format(void)
{
	struct ptable *ptbl = &the_ptable;
	unsigned sector_sz, blocks;
	unsigned next;
	int n;

	printf("Formatting %s(%d) slot.\n", mmc_slot?"EMMC":"SD", mmc_slot);
	if (mmc_init(mmc_slot)) {
		printf("mmc init failed?\n");
		return -1;
	}

	mmc_info(mmc_slot, &sector_sz, &blocks);
	printf("blocks %d\n", blocks);

	start_ptbl(ptbl, blocks);
	n = 0;
	next = 0;
	for (n = 0, next = 0; partitions[n].name; n++) {
		unsigned sz = partitions[n].size_kb * 2;
		if (!strcmp(partitions[n].name,"-")) {
			next += sz;
			continue;
		}
		if (sz == 0)
			sz = blocks - next;
		if (add_ptn(ptbl, next, next + sz - 1, partitions[n].name))
			return -1;
		next += sz;
	}
	end_ptbl(ptbl);

	fastboot_flash_reset_ptn();
	if (mmc_write(mmc_slot, (void*) ptbl, 0, sizeof(struct ptable)) != 1)
		return -1;

	printf("\nnew partition table:\n");
	load_ptbl();

	return 0;
}

#ifdef CONFIG_SHIPPING_MODE
static int do_set_ship_mod()
{
	int ret = 0;
	int mmc_cont = SHIP_MOD_MMC_SLOT;
	ulong src_addr = SHIP_MOD_DDR_OFFSET;
	ulong dst_addr = SHIP_MOD_MMC_OFFSET;
	ulong size = 8;
	char *tmp = (char *)src_addr;
	tmp[0]='S';
	tmp[1]='H';
	tmp[2]='I';
	tmp[3]='P';
	tmp[4]='P';
	tmp[5]='I';
	tmp[6]='N';
	tmp[7]='G';
	tmp[8]=0;
	ret = mmc_init(mmc_cont);
	if (!ret) {
		mmc_write(mmc_cont, (unsigned char *)src_addr, dst_addr, size);
		printf("Shipping mode activated\n");
		//sprintf(response, "OKAY");
	} else {
		printf("Failed to set device to shipping mod\n");
		//sprintf(response, "FAIL");
	}

	return ret;
}

int setup_shipping_mode() {
	return do_set_ship_mod();
}
#endif

#ifdef CONFIG_CUSTOM_SN_FLASH
static int do_set_sn(const char *sn)
{
	int ret;
	unsigned char i, tmp;
	unsigned char databuf[3+SN_SIZE+SN_CHKSUM_SIZE+SN_ENCKEY_SIZE];
	unsigned char keybuf[SN_ENCKEY_SIZE];
	unsigned long chksum = 0;

	sprintf(keybuf, "%s", SN_ENCKEY);
	databuf[0] = SN_SIZE;
	databuf[1] = SN_CHKSUM_SIZE;
	databuf[2] = SN_ENCKEY_SIZE;

	/* write the SN to buffer */
	for (i = 0; i < SN_SIZE; i++) {
		tmp = sn[i];
		if (((tmp < '0') || (tmp > '9'))
			&& ((tmp < 'A') || (tmp > 'Z'))
			&& ((tmp < 'a') || (tmp > 'z'))) {
			printf("\"%c(0x%X)\" is not allowed\n", tmp, tmp);
			break;
		}
		databuf[3+i] = tmp ^ keybuf[i%SN_ENCKEY_SIZE];
		if ((i%(SN_CHKSUM_BIT/8)) == ((SN_CHKSUM_BIT/8)-1)) {
		for (tmp = 0; tmp < (SN_CHKSUM_BIT/8) && tmp <= i; tmp++) {
				chksum += (unsigned long)(databuf[3+i-tmp])
					<< (8*((SN_CHKSUM_BIT/8)-1-tmp));
			}
		}
	}
	if (i != SN_SIZE) {
		printf("set_sn: Invalid serial number\n");
		//sprintf(response, "FAILinvalid serial number");
		return -1;
	}

	/* write the checksum to buffer */
	for (i = 0; i < SN_CHKSUM_SIZE; i++)
		databuf[3+SN_SIZE+i] = (char)(chksum >> (8*(SN_CHKSUM_SIZE-1-i)));

	/* write the encryption key to buffer */
	for (i = 0; i < SN_ENCKEY_SIZE; i++)
		databuf[3+SN_SIZE+SN_CHKSUM_SIZE+i] = keybuf[i] ^ databuf[3+SN_SIZE+(i%SN_CHKSUM_SIZE)];

	struct fastboot_ptentry *ptn;

	/* find the parition */
	ptn = fastboot_flash_find_ptn(SN_MMC_PTN);
	if (ptn == 0) {
		//sprintf(response, "FAILpartition does not exist");
		return -2;
	}

	/* write the buffer to partition */
	ret = mmc_init(SN_MMC_SLOT);
	if (!ret) {
		mmc_write(SN_MMC_SLOT, databuf, ptn->start,
			3+SN_SIZE+SN_CHKSUM_SIZE+SN_ENCKEY_SIZE);
		printf("set_sn: Updated SN to \"%s\"\n", sn);
		//sprintf(response, "OKAY");
	} else {
		printf("set_sn: Failed to init mmc%d\n",SN_MMC_SLOT);
		//sprintf(response, "FAILinit of MMC card");
		return -3;
	}
	return 0;
}
#endif

#ifdef CONFIG_CUSTOM_LANGUAGE_FLASH
static int do_set_lang(const char *lang)
{
	// The following array was captured from /mydroid/build/target/product/languages_full.mk
	char locales[49][LANG_SIZE] = {"af_ZA", "am_ET", "ar_EG", "bg_BG", "ca_ES", "cs_CZ", "da_DK", "de_DE", "el_GR", "en_GB", "en_US", "es_ES", "es_US", "fa_IR", "fi_FI", "fr_FR", "hi_IN", "hi_IN", "hr_HR", "hu_HU", "in_ID", "it_IT", "iw_IL", "ja_JP", "ko_KR", "lt_LT", "lv_LV", "ms_MY", "nb_NO", "nl_NL", "pl_PL", "pt_BR", "pt_PT", "rm_CH", "ro_RO", "ru_RU", "sk_SK", "sl_SI", "sr_RS", "sv_SE", "sw_TZ", "th_TH", "tl_PH", "tr_TR", "uk_UA", "vi_VN", "zh_CN", "zh_TW", "zu_ZA"};
	int ret, i;
	unsigned char j=0;
	char locale[6];
	char *ptr;

	for (i=0; i<49; i++, ptr=lang) {
		for (; j<LANG_SIZE; j++) {
			if (ptr[j] != locales[i][j]) break;
			locale[j] = ptr[j];
		}
		if (j==LANG_SIZE) break;
	}

	if (i==49) {
		printf("set_lang: Invalid language\n");
		//sprintf(response, "FAILinvalid language");
		return -1;
	}

	struct fastboot_ptentry *ptn;

	/* find the parition */
	ptn = fastboot_flash_find_ptn(LANG_MMC_PTN);
	if (ptn == 0) {
		//sprintf(response, "FAILpartition does not exist");
		return -2;
	}

	/* write the buffer to partition */
	ret = mmc_init(LANG_MMC_SLOT);
	if (!ret) {
		mmc_write(LANG_MMC_SLOT, locale, ptn->start+LANG_MMC_OFFSET,
			LANG_SIZE);
		printf("set_lang: Updated language to \"%s\"\n", locale);
		//sprintf(response, "OKAY");
	} else {
		printf("set_lang: Failed to init mmc%d\n",LANG_MMC_SLOT);
		//sprintf(response, "FAILinit of MMC card");
		return -3;
	}
	return 0;
}
#endif

#ifdef CONFIG_CUSTOM_TIMEZONE_FLASH
static int do_set_time(const char *time)
{
	char *ptr = time;
	int offset = 0, sign = 0, ret;
	unsigned char i;
	char timezone[TIME_SIZE];

	if (*(ptr++) != 'G' || *(ptr++) != 'M' || *(ptr++) != 'T')
		offset = 0x1;

	if (*ptr == '+') sign = 1;
	else if (*ptr == '-') sign = -1;

	for (i=1; i<5 && ptr[i]>='0' && ptr[i]<='9'; i++) {
		offset *= 10;
		offset += sign * (ptr[i] - '0');
	}

	if (i != 5 || sign == 0 || offset > 1400 || offset < -1200) {
		printf("set_time: Invalid timezone\n");
		//sprintf(response, "FAILinvalid timezone");
		return -1;
	}

	if (offset<0) offset = -offset;
	sprintf(timezone,"GMT%c%02d:%02d",(sign==1)?'+':'-', offset/100, offset%100);

	struct fastboot_ptentry *ptn;

	/* find the parition */
	ptn = fastboot_flash_find_ptn(TIME_MMC_PTN);
	if (ptn == 0) {
		//sprintf(response, "FAILpartition does not exist");
		return -2;
	}

	/* write the buffer to partition */
	ret = mmc_init(TIME_MMC_SLOT);
	if (!ret) {
		mmc_write(TIME_MMC_SLOT, timezone, ptn->start+TIME_MMC_OFFSET,
			TIME_SIZE);
		printf("set_time: Updated timezone to \"%s\"\n", timezone);
		//sprintf(response, "OKAY");
	} else {
		printf("set_time: Failed to init mmc%d\n",TIME_MMC_SLOT);
		//sprintf(response, "FAILinit of MMC card");
		return -3;
	}
	return 0;
}
#endif
/**
*  get_partition_sz: Returns size of requested paritition in eMMC
* @buf: Caller buffer pointer the size will be returned in
* @partname: partittion name being requested
*/
char * get_partition_sz(char *buf, const char *partname)
{
	struct ptable *ptbl = &the_ptable;
	struct efi_header *hdr = &ptbl->header;
	struct efi_entry *entry = ptbl->entry;
	unsigned n, i;
	char curr_partname[EFI_NAMELEN];
	u64 fist_lba, last_lba, sz;
	u32 crc_orig;
	u32 crc;
	u32 *szptr = (u32 *) &sz;

	if (mmc_read(mmc_slot, 0,  (void *)ptbl, sizeof(struct ptable)) != 1){
		printf("\n ERROR Reading Partition Table \n");
		return buf;
	}

	/*Make sure there is a legit partition table*/
	if (load_ptbl()){
		printf("\n INVALID PARTITION TABLE \n");
		return buf;
	}

	/*crc needs to be computed with crc zeroed out.*/
	crc_orig = hdr->crc32;
	hdr->crc32 = 0;
	crc = crc32(0,0,0);
	crc = crc32(0, (void *) &ptbl->header, sizeof(ptbl->header));
	if (crc != crc_orig){
		printf("\n INVALID HEADER CRC!!\n");
		return buf;
	}

	for (n=0; n < EFI_ENTRIES; n++, entry++) {
		for (i = 0; i < EFI_NAMELEN; i++)
			curr_partname[i] = (char) entry->name[i];
		if (!strcmp(curr_partname, partname)){
			if (entry->last_lba < entry->first_lba){
				printf("\n REDICULOUS LENGTH!! \n");
				break;
			}
			sz = (entry->last_lba - entry->first_lba)/2;
			if (sz >= 0xFFFFFFFF)
				sprintf(buf, "0x%08x , %08x KB", szptr[1], szptr[0]);
			else
				sprintf(buf, "%d KB", szptr[0]);

			break;
		}
	}
	return buf;
}

int fastboot_oem(const char *cmd)
{
	if (!strcmp(cmd,"format"))
		return do_format();
#ifdef CONFIG_SHIPPING_MODE
	/* fastboot oem set_ship_mod */

	/* This command is used to avoid device being turned on by the power button during shipping.
	 * Once the shipping mode is on, the device will be forced shutdown unless a charger is detected.
	 */
	if (!strcmp(cmd,"set_ship_mod"))
		return do_set_ship_mod();
#endif
#ifdef CONFIG_CUSTOM_SN_FLASH
	/* fastboot oem set_sn [xxxxxxxxxxxxxxxxxx] */
	if (!memcmp(cmd,"set_sn ",7))
		return do_set_sn(cmd+7);
#endif
#ifdef CONFIG_CUSTOM_LANGUAGE_FLASH
	/* fastboot oem set_lang [xx_XX] */
	if (!memcmp(cmd, "set_lang ",9))
		return do_set_lang(cmd+9);
#endif
#ifdef CONFIG_CUSTOM_TIMEZONE_FLASH
	/* fastboot oem set_time [GMT-1200~GMT+1400] */
	if (!memcmp(cmd, "set_time ",9))
		return do_set_time(cmd+9);
#endif

	return -1;
}

void board_mmc_init(void)
{
	/* nothing to do this early */
}

/**
*  get_boot_slot: Returns boot from SD or eMMC
* @ret: 0:SD	1:eMMC
*/
static int get_boot_slot(void)
{
	u32 control_register = __raw_readl(OMAP44xx_BOOT_DEVICE);

	if ((control_register & 0xff) == 0x5)
		return 0;
	else
		return 1;
}

int omap4_mmc_init(void)
{
	int i;
	char booticmd[20];
	int boot_slot = get_boot_slot();

	/* If we booted off of SD slot, initialize SD card as well. */
	if (boot_slot == 0) {
		printf("Initializing SD(0) Slot.\n");
		/* Temporarily set mmc_slot to SD */
		mmc_slot = 0;
		if (mmc_init(boot_slot)) {
			printf("mmc init failed?\n");
			return 1;
		}
		load_ptbl();
	}

	/* Default back to eMMC(1) slot
	  * If someone wants to flash all partitions to SD slot
	  * they need to explicty give "fastboot oem set_boot_slot:SD"
	  */
	mmc_slot = 1;

	if (mmc_init(mmc_slot)) {
		printf("mmc init failed?\n");
		return 1;
	}
	sprintf(booticmd, "booti mmc%d", boot_slot);
	setenv("bootcmd", booticmd);
	printf("efi partition table:\n");
	return load_ptbl();
}


