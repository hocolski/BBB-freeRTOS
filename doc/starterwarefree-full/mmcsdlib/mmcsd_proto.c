/**
 *  \file mmcsd_proto.c
 *
 *  \brief this file defines the MMC/SD standard operations
 *
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include "mmcsd_proto.h"
#include "string.h"
#include "uartStdio.h"
#include "cache.h"
#include "hs_mmcsd.h"

#define DATA_RESPONSE_WIDTH       (512)

/*
 * EXT_CSD switch cmd macros
 */

#define EXT_CSD_FLUSH_CACHE             32      /* W */
#define EXT_CSD_CACHE_CTRL              33      /* R/W */
#define EXT_CSD_POWER_OFF_NOTIFICATION  34      /* R/W */
#define EXT_CSD_PACKED_FAILURE_INDEX    35      /* RO */
#define EXT_CSD_PACKED_CMD_STATUS       36      /* RO */
#define EXT_CSD_EXP_EVENTS_STATUS       54      /* RO, 2 bytes */
#define EXT_CSD_EXP_EVENTS_CTRL         56      /* R/W, 2 bytes */
#define EXT_CSD_DATA_SECTOR_SIZE        61      /* R */
#define EXT_CSD_GP_SIZE_MULT            143     /* R/W */
#define EXT_CSD_PARTITION_ATTRIBUTE     156     /* R/W */
#define EXT_CSD_PARTITION_SUPPORT       160     /* RO */
#define EXT_CSD_HPI_MGMT                161     /* R/W */
#define EXT_CSD_RST_N_FUNCTION          162     /* R/W */
#define EXT_CSD_BKOPS_EN                163     /* R/W */
#define EXT_CSD_BKOPS_START             164     /* W */
#define EXT_CSD_SANITIZE_START          165     /* W */
#define EXT_CSD_WR_REL_PARAM            166     /* RO */
#define EXT_CSD_RPMB_MULT               168     /* RO */
#define EXT_CSD_BOOT_WP                 173     /* R/W */
#define EXT_CSD_ERASE_GROUP_DEF         175     /* R/W */
#define EXT_CSD_PART_CONFIG             179     /* R/W */
#define EXT_CSD_ERASED_MEM_CONT         181     /* RO */
#define EXT_CSD_BUS_WIDTH               183     /* R/W */
#define EXT_CSD_HS_TIMING               185     /* R/W */
#define EXT_CSD_POWER_CLASS             187     /* R/W */
#define EXT_CSD_REV                     192     /* RO */
#define EXT_CSD_STRUCTURE               194     /* RO */
#define EXT_CSD_CARD_TYPE               196     /* RO */
#define EXT_CSD_OUT_OF_INTERRUPT_TIME   198     /* RO */
#define EXT_CSD_PART_SWITCH_TIME        199     /* RO */
#define EXT_CSD_PWR_CL_52_195           200     /* RO */
#define EXT_CSD_PWR_CL_26_195           201     /* RO */
#define EXT_CSD_PWR_CL_52_360           202     /* RO */
#define EXT_CSD_PWR_CL_26_360           203     /* RO */
#define EXT_CSD_SEC_CNT                 212     /* RO, 4 bytes */
#define EXT_CSD_S_A_TIMEOUT             217     /* RO */
#define EXT_CSD_REL_WR_SEC_C            222     /* RO */
#define EXT_CSD_HC_WP_GRP_SIZE          221     /* RO */
#define EXT_CSD_ERASE_TIMEOUT_MULT      223     /* RO */
#define EXT_CSD_HC_ERASE_GRP_SIZE       224     /* RO */
#define EXT_CSD_BOOT_MULT               226     /* RO */
#define EXT_CSD_SEC_TRIM_MULT           229     /* RO */
#define EXT_CSD_SEC_ERASE_MULT          230     /* RO */
#define EXT_CSD_SEC_FEATURE_SUPPORT     231     /* RO */
#define EXT_CSD_TRIM_MULT               232     /* RO */
#define EXT_CSD_PWR_CL_200_195          236     /* RO */
#define EXT_CSD_PWR_CL_200_360          237     /* RO */
#define EXT_CSD_PWR_CL_DDR_52_195       238     /* RO */
#define EXT_CSD_PWR_CL_DDR_52_360       239     /* RO */
#define EXT_CSD_BKOPS_STATUS            246     /* RO */
#define EXT_CSD_POWER_OFF_LONG_TIME     247     /* RO */
#define EXT_CSD_GENERIC_CMD6_TIME       248     /* RO */
#define EXT_CSD_CACHE_SIZE              249     /* RO, 4 bytes */
#define EXT_CSD_TAG_UNIT_SIZE           498     /* RO */
#define EXT_CSD_DATA_TAG_SUPPORT        499     /* RO */
#define EXT_CSD_MAX_PACKED_WRITES       500     /* RO */
#define EXT_CSD_MAX_PACKED_READS        501     /* RO */
#define EXT_CSD_BKOPS_SUPPORT           502     /* RO */
#define EXT_CSD_HPI_FEATURES            503     /* RO */

/*
 * EXT_CSD field definitions
 */

#define EXT_CSD_WR_REL_PARAM_EN         (1<<2)

#define EXT_CSD_BOOT_WP_B_PWR_WP_DIS    (0x40)
#define EXT_CSD_BOOT_WP_B_PERM_WP_DIS   (0x10)
#define EXT_CSD_BOOT_WP_B_PERM_WP_EN    (0x04)
#define EXT_CSD_BOOT_WP_B_PWR_WP_EN     (0x01)

#define EXT_CSD_PART_CONFIG_ACC_MASK    (0x7)
#define EXT_CSD_PART_CONFIG_ACC_BOOT0   (0x1)
#define EXT_CSD_PART_CONFIG_ACC_RPMB    (0x3)
#define EXT_CSD_PART_CONFIG_ACC_GP0     (0x4)

#define EXT_CSD_PART_SUPPORT_PART_EN    (0x1)

#define EXT_CSD_CMD_SET_NORMAL          (1<<0)
#define EXT_CSD_CMD_SET_SECURE          (1<<1)
#define EXT_CSD_CMD_SET_CPSECURE        (1<<2)

#define EXT_CSD_CARD_TYPE_26    (1<<0)  /* Card can run at 26MHz */
#define EXT_CSD_CARD_TYPE_52    (1<<1)  /* Card can run at 52MHz */
#define EXT_CSD_CARD_TYPE_MASK  0x3F    /* Mask out reserved bits */
#define EXT_CSD_CARD_TYPE_DDR_1_8V  (1<<2)   /* Card can run at 52MHz */
                                              /* DDR mode @1.8V or 3V I/O */
#define EXT_CSD_CARD_TYPE_DDR_1_2V  (1<<3)   /* Card can run at 52MHz */
                                              /* DDR mode @1.2V I/O */
#define EXT_CSD_CARD_TYPE_DDR_52       (EXT_CSD_CARD_TYPE_DDR_1_8V  \
                                         | EXT_CSD_CARD_TYPE_DDR_1_2V)
 #define EXT_CSD_CARD_TYPE_SDR_1_8V      (1<<4)  /* Card can run at 200MHz */
#define EXT_CSD_CARD_TYPE_SDR_1_2V      (1<<5)  /* Card can run at 200MHz */
                                                 /* SDR mode @1.2V I/O */

#define EXT_CSD_BUS_WIDTH_1     0       /* Card is in 1 bit mode */
#define EXT_CSD_BUS_WIDTH_4     1       /* Card is in 4 bit mode */
#define EXT_CSD_BUS_WIDTH_8     2       /* Card is in 8 bit mode */
#define EXT_CSD_DDR_BUS_WIDTH_4 5       /* Card is in 4 bit DDR mode */
#define EXT_CSD_DDR_BUS_WIDTH_8 6       /* Card is in 8 bit DDR mode */

#define EXT_CSD_SEC_ER_EN       BIT(0)
#define EXT_CSD_SEC_BD_BLK_EN   BIT(2)
#define EXT_CSD_SEC_GB_CL_EN    BIT(4)
#define EXT_CSD_SEC_SANITIZE    BIT(6)  /* v4.5 only */

#define EXT_CSD_RST_N_EN_MASK   0x3
#define EXT_CSD_RST_N_ENABLED   1       /* RST_n is enabled on card */

#define EXT_CSD_NO_POWER_NOTIFICATION   0
#define EXT_CSD_POWER_ON                1
#define EXT_CSD_POWER_OFF_SHORT         2
#define EXT_CSD_POWER_OFF_LONG          3

#define EXT_CSD_PWR_CL_8BIT_MASK        0xF0    /* 8 bit PWR CLS */
#define EXT_CSD_PWR_CL_4BIT_MASK        0x0F    /* 8 bit PWR CLS */
#define EXT_CSD_PWR_CL_8BIT_SHIFT       4
#define EXT_CSD_PWR_CL_4BIT_SHIFT       0

#define EXT_CSD_PACKED_EVENT_EN BIT(3)

/*
 * EXCEPTION_EVENT_STATUS field
 */
#define EXT_CSD_URGENT_BKOPS            BIT(0)
#define EXT_CSD_DYNCAP_NEEDED           BIT(1)
#define EXT_CSD_SYSPOOL_EXHAUSTED       BIT(2)
#define EXT_CSD_PACKED_FAILURE          BIT(3)

#define EXT_CSD_PACKED_GENERIC_ERROR    BIT(0)
#define EXT_CSD_PACKED_INDEXED_ERROR    BIT(1)


/*
 * MMC_SWITCH access modes
 *
 * The SWITCH command response is of type R1b, therefore, the host should read the card status, using
 * SEND_STATUS command, after the busy signal is de-asserted, to check the result of the SWITCH
 * operation.
 */
#define MMC_SWITCH_MODE_CMD_SET         0x00    /* The command set is changed according to the Cmd Set field of the argument */
#define MMC_SWITCH_MODE_SET_BITS        0x01    /* The bits in the pointed byte are set, according to the �1� bits in the Value field. */
#define MMC_SWITCH_MODE_CLEAR_BITS      0x02    /* The bits in the pointed byte are cleared, according to the �1� bits in the Value field. */
#define MMC_SWITCH_MODE_WRITE_BYTE      0x03    /* The Value field is written into the pointed byte. */


unsigned int unstuffBits(unsigned int *resp,unsigned int start,unsigned int size)
{
	const int __size = size;
	const unsigned int __mask = (__size < 32 ? 1 << __size : 0) - 1;
	const int __off = 3 - ((start) / 32);
	const int __shft = (start) & 31;
	unsigned int __res;

	__res = resp[__off] >> __shft;
	if (__size + __shft > 32)
	{
		__res |= resp[__off-1] << ((32 - __shft) % 32);
	}

	return(__res & __mask);
}

/*
 * EXT_CSD struct
 */
struct extCsd {
	// [511:505] Reserved
	char s_cmd_set;		 					// [504] Supported Command Sets
	char hpi_features;						// [503] HPI features
	char bkops_support; 					// [502] Background operations support
	// [501:247] Reserved
	char bkops_status;						// [246] Background operations status
	unsigned int correctly_prg_sectors_num;	// [245:242] Number of correctly programmed sectors
	char ini_timeout_ap;					// [241] 1st initialization time after partitioning
	// [240] Reserved
	char pwr_cl_ddr_52_360;					// [239] Power class for 52MHz, DDR at 3.6V
	char pwr_cl_ddr_52_195;					// [238] Power class for 52MHz, DDR at 1.95V
	// [237:236]
	char min_perf_ddr_w_8_52;				// [235] Minimum Write Performance for 8bit at 52MHz in DDR mode
	char min_perf_ddr_r_8_52;				// [234] Minimum Read Performance for 8bit at 52MHz in DDR mode
	// [233]
	char trim_mult;							// [232] TRIM Multiplier
	char sec_feature_support;				// [231] Secure Feature support
	char sec_erase_mult;					// [230] Secure Erase Multiplier
	char sec_trim_mult;						// [229] Secure TRIM Multiplier
	char boot_info;							// [228] Boot information
	// [227] Reserved
	char boot_size_multi;					// [226] Boot partition size
	char acc_size;							// [225] Access size;
	char hc_erase_grp_size;					// [224] High-capacity erase unit size
	char erase_timeout_mult;				// [223] High-capacity erase timeout
	char rel_wr_sec_c; 						// [222] Reliable write sector count
	char hc_wp_grp_size;					// [221] High-capacity write protect group size
	char s_c_vcc;							// [220] Sleep current (VCC)
	char s_c_vccq;							// [219] Sleep current (VCCQ)
	// [218] Reserved
	char s_a_timeout;						// [217] Sleep/awake timeout
	// [216] Reserved
	unsigned int sec_count;					// [215:212] Sector Count
	// [211] Reserved
	char min_perf_w_8_52;					// [210] Minimum Write Performance for 8bit at 52MHz
	char min_perf_r_8_52;					// [209] Minimum Read Performance for 8bit at 52MHz
	char min_perf_w_8_26_4_52;				// [208] Minimum Write Performance for 8bit at 26MHz, for 4bit at 52MHz
	char min_perf_r_8_26_4_52;				// [207] Minimum Read Performance for 8bit at 26MHz, for 4bit at 52MHz
	char min_perf_w_4_26;					// [206] Minimum Write Performance for 4bit at 26MHz
	char min_perf_r_4_26;					// [205] Minimum Read Performance for 4bit at 26MHz
	// [211] Reserved
	char pwr_cl_26_360;						// [203] Power class for 26MHz at 3.6V
	char pwr_cl_52_360;						// [202] Power class for 52MHz at 3.6V
	char pwr_cl_26_195;						// [201] Power class for 26MHz at 1.95V
	char pwr_cl_52_195;						// [200] Power class for 52MHz at 1.95V
	char partition_switch_time;				// [199] Partition switching timing
	char out_of_interrupt_time;				// [198] Out-of-interrupt busy timing
	// [197] Reserved
	char card_type;							// [196] Card type
	// [195] Reserved
	char csd_structure;						// [194] CSD structure version
	// [193] Reserved
	char ext_csd_rev;						// [192] Extended CSD revision
	char cmd_set;							// [191] Command set
	// [190] Reserved
	char cmd_set_rev;						// [189] Command set revision
	// [188] Reserved
	char power_class;						// [187] Power class
	// [186] Reserved
	char hs_timing;							// [185] High-speed interface timing
	// [184] Reserved
	char bus_width;							// [183] Bus width mode
	// [182] Reserved
	char erased_mem_cont;					// [181] Erased memory content
	// [180] Reserved
	char partition_config;					// [179] Partition configuration
	char boot_config_prot;					// [178] Boot config protection
	char boot_bus_width;					// [177] Boot bus width1
	// [176] Reserved
	char erase_group_def;					// [175] High-density erase group definition
	// [174] Reserved;
	char boot_wp;							// [173] Boot area write protection register
	// [172] Reserved;
	char user_wp;							// [171] User area write protection register
	// [170] Reserved;
	char fw_config;							// [169] FW configuration
	char rpmb_size_mult;					// [168] RPMB Size
	char wr_rel_set; 						// [167] Write reliability setting register
	char wr_rel_param;						// [166] Write reliability parameter register
	// [165] Reserved;
	char bkops_start;						// [164] Manually start background operations
	char bkops_en;							// [163] Enable background operations handshake
	char rst_n_function;					// [162] H/W reset function
	char hpi_mgmt;							// [161] HPI management
	char partitioning_support;				// [160] Partitioning Support
	unsigned char max_enh_size_mult; 		// [159:157] Max Enhanced Area Size
	char partitions_attribute;				// [156] Partitions attribute
	char partition_setting_completed; 		// [155] Paritioning Setting
	unsigned int gp_size_mult[4];			// [154:143] General Purpose Partition Size
	unsigned int enh_size_mult; 			// [142:140] Enhanced User Data Area Size
	unsigned int enh_start_addr;			// [139:136] Enhanced User Data Start Address
	// [135] Reserved;
	char sec_bad_blk_mgmnt;			// [134] Bad Block Management mode
	// [133:0] Reserved
}extCsd;



struct cardCSD
{
	char csd_structure;
	char spec_vers;
	char taac;
	char nsac;
	char tran_speed;
	unsigned short ccc;
	char read_bl_len;
	char read_bl_partial;
	char write_blk_misalign;
	char read_blk_misalign;
	char dsr_imp;
	unsigned short c_size;
	char vdd_r_curr_min;
	char vdd_r_curr_max;
	char vdd_w_curr_min;
	char vdd_w_curr_max;
	char c_size_mult;
	char erase_grp_size;
	char erase_grp_mult;
	char wp_grp_size;
	char wp_grp_enable;
	char default_ecc;
	char r2w_factor;
	char write_bl_len;
	char write_bl_partial;
	char content_prot_app;
	char file_format_grp;
	char copy;
	char perm_write_protect;
	char tmp_write_protect;
	char file_format;
	char ecc;
	char crc;
} CSDInfo;


struct cardCid {
	char mid; 					/* Manufacturer ID */
	char cbx;					/* Device/BGA */
	char oid;					/* OEM/Application ID */
	unsigned long int pnm;		/* Product name */
	char prv;					/* Product revision  */
	unsigned int psn;			/* Product serial number */
	char mdt;					/* Manufacturing date */
	char crc;					/* crc7 checksum */
}cardCid;

//#define DATA_RESPONSE_WIDTH       (SOC_CACHELINE_SIZE)

/* Cache size aligned data buffer (minimum of 64 bytes) for command response */
#ifdef __TMS470__
#pragma DATA_ALIGN(dataBuffer, SOC_CACHELINE_SIZE);
static unsigned char dataBuffer[DATA_RESPONSE_WIDTH];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = SOC_CACHELINE_SIZE
static unsigned char dataBuffer[DATA_RESPONSE_WIDTH];

#elif defined(gcc)
static unsigned char dataBuffer[DATA_RESPONSE_WIDTH]
                               __attribute__((aligned(SOC_CACHELINE_SIZE)));

#else
#error "Unsupported compiler\n\r"
#endif

int mmc_switch(mmcsdCtrlInfo *ctrl, char set, char index, char value);
int mmc_send_status(mmcsdCtrlInfo *ctrl, unsigned int *status);
unsigned int MMCSDCardTypeCheck(mmcsdCtrlInfo * ctrl);


void decodeExtCsd(mmcsdCtrlInfo *ctrl, unsigned char *buffer)
{
	extCsd.s_cmd_set		 			= buffer[504];
	extCsd.hpi_features					= buffer[503];// [503] HPI features
	extCsd.bkops_support 				= buffer[502];// [502] Background operations support
	// [501:247] Reserved
	extCsd.bkops_status					= buffer[246]; // [246] Background operations status
	extCsd.correctly_prg_sectors_num	=(buffer[245] << 24) | (buffer[244] << 16) | (buffer[243] << 8) | buffer[242];	// [245:242] Number of correctly programmed sectors
	extCsd.ini_timeout_ap				= buffer[241]; // [241] 1st initialization time after partitioning
	// [240] Reserved
	extCsd.pwr_cl_ddr_52_360			= buffer[239];	// [239] Power class for 52MHz, DDR at 3.6V
	extCsd.pwr_cl_ddr_52_195			= buffer[238];	// [238] Power class for 52MHz, DDR at 1.95V
	// [237:236]
	extCsd.min_perf_ddr_w_8_52			= buffer[235];// [235] Minimum Write Performance for 8bit at 52MHz in DDR mode
	extCsd.min_perf_ddr_r_8_52			= buffer[234];// [234] Minimum Read Performance for 8bit at 52MHz in DDR mode
	// [233]
	extCsd.trim_mult					= buffer[232];	// [232] TRIM Multiplier
	extCsd.sec_feature_support			= buffer[231];  // [231] Secure Feature support
	extCsd.sec_erase_mult				= buffer[230];// [230] Secure Erase Multiplier
	extCsd.sec_trim_mult				= buffer[229];	// [229] Secure TRIM Multiplier
	extCsd.boot_info					= buffer[228];	// [228] Boot information
	// [227] Reserved
	extCsd.boot_size_multi				= buffer[226];// [226] Boot partition size
	extCsd.acc_size						= buffer[225];// [225] Access size;
	extCsd.hc_erase_grp_size			= buffer[224];	// [224] High-capacity erase unit size
	extCsd.erase_timeout_mult			= buffer[223];// [223] High-capacity erase timeout
	extCsd.rel_wr_sec_c 				= buffer[222];	// [222] Reliable write sector count
	extCsd.hc_wp_grp_size				= buffer[221];// [221] High-capacity write protect group size
	extCsd.s_c_vcc						= buffer[220];// [220] Sleep current (VCC)
	extCsd.s_c_vccq						= buffer[219];// [219] Sleep current (VCCQ)
	// [218] Reserved
	extCsd.s_a_timeout					= buffer[217];// [217] Sleep/awake timeout
	// [216] Reserved
//	extCsd.sec_count					= (buffer[215] << 24) | (buffer[124] << 16) | (buffer[213] << 8) | buffer[213];	// [215:212] Sector Count
    extCsd.sec_count                    = (buffer[215] << 24) | (buffer[214] << 16) | (buffer[213] << 8) | buffer[212]; // [215:212] Sector Count, http://e2e.ti.com/support/embedded/starterware/f/790/p/272577/1348464.aspx#1348464
	// [211] Reserved
	extCsd.min_perf_w_8_52				= buffer[210];// [210] Minimum Write Performance for 8bit at 52MHz
	extCsd.min_perf_r_8_52				= buffer[209];// [209] Minimum Read Performance for 8bit at 52MHz
	extCsd.min_perf_w_8_26_4_52			= buffer[208];// [208] Minimum Write Performance for 8bit at 26MHz, for 4bit at 52MHz
	extCsd.min_perf_r_8_26_4_52			= buffer[207];// [207] Minimum Read Performance for 8bit at 26MHz, for 4bit at 52MHz
	extCsd.min_perf_w_4_26				= buffer[206];// [206] Minimum Write Performance for 4bit at 26MHz
	extCsd.min_perf_r_4_26				= buffer[205];// [205] Minimum Read Performance for 4bit at 26MHz
	// [211] Reserved
	extCsd.pwr_cl_26_360				= buffer[203];	// [203] Power class for 26MHz at 3.6V
	extCsd.pwr_cl_52_360				= buffer[202];	// [202] Power class for 52MHz at 3.6V
	extCsd.pwr_cl_26_195				= buffer[201];	// [201] Power class for 26MHz at 1.95V
	extCsd.pwr_cl_52_195				= buffer[200];	// [200] Power class for 52MHz at 1.95V
	extCsd.partition_switch_time		= buffer[199];	// [199] Partition switching timing
	extCsd.out_of_interrupt_time		= buffer[198];	// [198] Out-of-interrupt busy timing
	// [197] Reserved
	extCsd.card_type					= buffer[196];	// [196] Card type
	// [195] Reserved
	extCsd.csd_structure				= buffer[194];	// [194] CSD structure version
	// [193] Reserved
	extCsd.ext_csd_rev					= buffer[192];// [192] Extended CSD revision
	extCsd.cmd_set						= buffer[191];// [191] Command set
	// [190] Reserved
	extCsd.cmd_set_rev					= buffer[189];// [189] Command set revision
	// [188] Reserved
	extCsd.power_class					= buffer[187];// [187] Power class
	// [186] Reserved
	extCsd.hs_timing					= buffer[185];	// [185] High-speed interface timing
	// [184] Reserved
	extCsd.bus_width					= buffer[183];	// [183] Bus width mode
	// [182] Reserved
	extCsd.erased_mem_cont				= buffer[181];// [181] Erased memory content
	// [180] Reserved
	extCsd.partition_config				= buffer[179];// [179] Partition configuration
	extCsd.boot_config_prot				= buffer[178];// [178] Boot config protection
	extCsd.boot_bus_width				= buffer[177];// [177] Boot bus width1
	// [176] Reserved
	extCsd.erase_group_def				= buffer[175];// [175] High-density erase group definition
	// [174] Reserved
	extCsd.boot_wp						= buffer[173];// [173] Boot area write protection register
	// [172] Reserved
	extCsd.user_wp						= buffer[171];// [171] User area write protection register
	// [170] Reserved
	extCsd.fw_config					= buffer[169];	// [169] FW configuration
	extCsd.rpmb_size_mult				= buffer[168];// [168] RPMB Size
	extCsd.wr_rel_set 					= buffer[167];// [167] Write reliability setting register
	extCsd.wr_rel_param					= buffer[166];// [166] Write reliability parameter register
	// [165] Reserved
	extCsd.bkops_start					= buffer[164];// [164] Manually start background operations
	extCsd.bkops_en						= buffer[163];// [163] Enable background operations handshake
	extCsd.rst_n_function				= buffer[162];// [162] H/W reset function
	extCsd.hpi_mgmt						= buffer[161];// [161] HPI management
	extCsd.partitioning_support			= buffer[160];// [160] Partitioning Support
	extCsd.max_enh_size_mult 			= (buffer[159] << 16) | (buffer[158] << 8) | buffer[157]; // [159:157] Max Enhanced Area Size
	extCsd.partitions_attribute			= buffer[156];// [156] Partitions attribute
	extCsd.partition_setting_completed 	= buffer[155];// [155] Paritioning Setting
	extCsd.gp_size_mult[0]				= (buffer[154] << 24) | (buffer[153] << 16) | (buffer[152] << 8) | buffer[151];// [154:143] General Purpose Partition Size
	extCsd.gp_size_mult[1]				= (buffer[150] << 24) | (buffer[149] << 16) | (buffer[148] << 8) | buffer[147];// [154:143] General Purpose Partition Size
	extCsd.gp_size_mult[2]				= (buffer[146] << 24) | (buffer[145] << 16) | (buffer[144] << 8) | buffer[143];// [154:143] General Purpose Partition Size
	extCsd.enh_size_mult 				= (buffer[142] << 16) | (buffer[141] << 8) | buffer[140];// [142:140] Enhanced User Data Area Size
	extCsd.enh_start_addr				= (buffer[139] << 24) | (buffer[138] << 16) | (buffer[137] << 8) | buffer[136];// [139:136] Enhanced User Data Start Address
	// [135] Reserved;
	extCsd.sec_bad_blk_mgmnt			= buffer[134];	// [134] Bad Block Management mode
	// [133:0] Reserved

	ctrl->card->cardType = extCsd.card_type;
	ctrl->card->busWidth = extCsd.bus_width;
}



void GetCSDParameters(mmcsdCtrlInfo *ctrl)
{
	unsigned int blockNr = 0;
	unsigned int mult = 0;

	/* Describes the version of the CSD structure. */
	CSDInfo.csd_structure = ((ctrl->card->raw_csd[3] & 0xC0000000) >> 30); // [127:126]
	CSDInfo.spec_vers = ((ctrl->card->raw_csd[3] & 0x3C000000) >> 26); // [125:122] everything above 4 is reserved
	// [121:120] reserved
	CSDInfo.taac = ((ctrl->card->raw_csd[3] & 0x00FF0000) >> 16); // [119:112] Data read access-time 1
	CSDInfo.nsac = ((ctrl->card->raw_csd[3] & 0x0000FF00) >> 8); // [111:104] Data read access-time 2 in CLK cycles (NSAC*100)
	CSDInfo.tran_speed = (ctrl->card->raw_csd[3] & 0x000000FF); // [103:96] Max. bus clock frequency

	CSDInfo.ccc = ((ctrl->card->raw_csd[2] & 0xFF000000) >> 20) | ((ctrl->card->raw_csd[2] & 0x00F00000) >> 20); // [95:84] Card command classes
	CSDInfo.read_bl_len = ((ctrl->card->raw_csd[2] & 0x000F0000) >> 16); // [83:80] Max. read data block length
	CSDInfo.read_bl_partial = ((ctrl->card->raw_csd[2] & 0x00008000) >> 15); // [79:79] Partial blocks for read allowed
	CSDInfo.write_blk_misalign = ((ctrl->card->raw_csd[2] & 0x00004000) >> 14); // [78:78] WRITE_BLK_MISALIGN
	CSDInfo.read_blk_misalign = ((ctrl->card->raw_csd[2] & 0x00002000) >> 13); // [77:77] READ_BLK_MISALIGN
	CSDInfo.dsr_imp = ((ctrl->card->raw_csd[2] & 0x00001000) >> 12); // [76:76] DSR implemented
	// [75:74] reserved
	CSDInfo.c_size = ((ctrl->card->raw_csd[2] & 0x000003FF) << 2) | ((ctrl->card->raw_csd[1] & 0xC0000000) >> 30); // [73:62] Device size
	CSDInfo.vdd_r_curr_min = ((ctrl->card->raw_csd[1] & 0x38000000) >> 27); // [61:59] Max. read current @ VDD min
	CSDInfo.vdd_r_curr_max = ((ctrl->card->raw_csd[1] & 0x07000000) >> 24); // [58:56] Max. read current @ VDD max
	CSDInfo.vdd_w_curr_min = ((ctrl->card->raw_csd[1] & 0x00E00000) >> 21); // [55:53] Max. write current @ VDD min
	CSDInfo.vdd_w_curr_max = ((ctrl->card->raw_csd[1] & 0x001C0000) >> 18); // [52:50] Max. write current @ VDD max
	CSDInfo.c_size_mult = ((ctrl->card->raw_csd[1] & 0x00038000) >> 15); // [49:47] Device size multiplier
	CSDInfo.erase_grp_size = ((ctrl->card->raw_csd[1] & 0x00007C00) >> 10); // [46:42] Erase group size
	CSDInfo.erase_grp_mult = ((ctrl->card->raw_csd[1] & 0x000003E0) >> 5); // [41:37] Erase group size multiplier
	CSDInfo.wp_grp_size = (ctrl->card->raw_csd[1] & 0x0000001F); // [36:32] Write protect group size
	CSDInfo.wp_grp_enable = ((ctrl->card->raw_csd[0] & 0x80000000) >> 31); // [31:31] WP_GRP_ENABLE
	CSDInfo.default_ecc = ((ctrl->card->raw_csd[0] & 0x60000000) >> 29); // [30:29] Manufacturer default ECC
	CSDInfo.r2w_factor = ((ctrl->card->raw_csd[0] & 0x1C000000) >> 26); // [28:26] Write speed factor
	CSDInfo.write_bl_len = ((ctrl->card->raw_csd[0] & 0x03C00000) >> 22); // [25:22] Max. write data block length
	CSDInfo.write_bl_partial = ((ctrl->card->raw_csd[0] & 0x00200000) >> 21); // [21:21] Partial blocks for write allowed
	// [20:17]
	CSDInfo.content_prot_app = ((ctrl->card->raw_csd[0] & 0x00010000) >> 16); // [16:16] Content protection application
	CSDInfo.file_format_grp = ((ctrl->card->raw_csd[0] & 0x00008000) >> 13); // [15:15] File format group
	CSDInfo.copy = ((ctrl->card->raw_csd[0] & 0x00004000) >> 12); // [14:14] Copy flag (OTP)
	CSDInfo.perm_write_protect = ((ctrl->card->raw_csd[0] & 0x00002000) >> 11); // [13:13] Permanent write protection
	CSDInfo.tmp_write_protect = ((ctrl->card->raw_csd[0] & 0x00001000) >> 10); // [12:12] Temporary write protection
	CSDInfo.file_format = ((ctrl->card->raw_csd[0] & 0x00000C00) >> 10); // [11:10] File format
	CSDInfo.ecc = ((ctrl->card->raw_csd[0] & 0x00000300) >> 8); // [9:8] ECC code
	CSDInfo.crc = (ctrl->card->raw_csd[0] & 0x000000FE); // [7:1] CRC
	// [0:0] Not used, always�1�


	mult = 2^(CSDInfo.c_size + 2);
	blockNr = (CSDInfo.c_size + 1) * mult;
	ctrl->card->size = blockNr * mult;


	//ctrl->card->blkLen = 1 << (CSDInfo.read_bl_len);
	ctrl->card->blkLen = 1 << (CSDInfo.read_bl_len - 1); // Set it to 512 /////////////////////////////////////////////////////////

	ctrl->card->nBlks = ctrl->card->size/ctrl->card->blkLen;

	ctrl->card->tranSpeed = CSDInfo.tran_speed ;
}


void getCID(mmcsdCtrlInfo *ctrl)
{
	unsigned int temp[4];
	int i;

	// UNSTUFF_BITS() read in a reverse order so use a temp buffer
	for(i=0; i<4; i++)
	{
		temp[3-i] = ctrl->card->raw_cid[i];
	}

	unsigned int *resp = temp;

	cardCid.mid	= (char)unstuffBits(resp,120,8);
	cardCid.cbx	= (char)unstuffBits(resp,112,2);
	cardCid.oid	= (char)unstuffBits(resp,102,8);
	cardCid.pnm	= unstuffBits(resp,56,48); // This value is not correct!
	cardCid.prv	= (char)unstuffBits(resp,48,8);
	cardCid.psn	= unstuffBits(resp,16,32);
	cardCid.mdt	= (char)unstuffBits(resp,8,8);
	cardCid.crc	= (char)unstuffBits(resp,1,7);
}




/**
 * \brief   This function sends the command to MMCSD.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \param    mmcsdCmd It determines the mmcsd cmd
 *
 * \return   status of the command.
 *
 **/
unsigned int MMCSDCmdSend(mmcsdCtrlInfo *ctrl, mmcsdCmd *c)
{
    return ctrl->cmdSend(ctrl, c);
}

/**
 * \brief   This function sends the application command to MMCSD.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \param    mmcsdCmd It determines the mmcsd cmd
 *
 * \return   status of the command.
 *
 **/
unsigned int MMCSDAppCmdSend(mmcsdCtrlInfo *ctrl, mmcsdCmd *c)
{
    unsigned int status = 0;
    mmcsdCmd capp;


    /* APP cmd should be preceeded by a CMD55 */
    capp.idx = SD_CMD(55);
    capp.flags = 0;
    capp.arg = ctrl->card->rca << 16;
    status = MMCSDCmdSend(ctrl, &capp);

    if (status == 0)
    {
        /* return safely, since we cannot continue if CMD55 fails */
        return 0;
    }

    status = MMCSDCmdSend(ctrl, c);

    return status;
}

/**
 * \brief   Configure the MMC/SD bus width
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \param   buswidth   SD/MMC bus width.
 * 
 *  buswidth can take the values.
 *     HS_MMCSD_BUS_WIDTH_8BIT.
 *     HS_MMCSD_BUS_WIDTH_4BIT.
 *     HS_MMCSD_BUS_WIDTH_1BIT.
 *
 * \return  None.
 *
 **/
unsigned int MMCSDBusWidthSet(mmcsdCtrlInfo *ctrl)
{
    mmcsdCardInfo *card = ctrl->card;
    unsigned int status = 0;
    mmcsdCmd capp;

    capp.idx = SD_CMD(6);
    capp.arg = SD_BUS_WIDTH_1BIT;
    capp.flags = 0;

    if (ctrl->busWidth & SD_BUS_WIDTH_4BIT)
    {
        if (card->busWidth & SD_BUS_WIDTH_4BIT)
        {
            capp.arg = SD_BUS_WIDTH_4BIT;
        }
    }

    else if(ctrl->busWidth & SD_BUS_WIDTH_8BIT)
    {
    	if (card->busWidth & SD_BUS_WIDTH_8BIT)
		{
			capp.arg = SD_BUS_WIDTH_8BIT;
		}
    }


    capp.arg = capp.arg >> 1;

    status = MMCSDAppCmdSend(ctrl, &capp);

    if (1 == status)
    {
        if (capp.arg == 0)
        {
            ctrl->busWidthConfig(ctrl, SD_BUS_WIDTH_1BIT);
        }
        else if (capp.arg == SD_BUS_WIDTH_8BIT)
        {
        	ctrl->busWidthConfig(ctrl, SD_BUS_WIDTH_8BIT);
        }
        else
        {
            ctrl->busWidthConfig(ctrl, SD_BUS_WIDTH_4BIT);
        }
    }
    return status;
}

/**
 * \brief    This function configures the transmission speed in MMCSD.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  1 - successfull.
 *           0 - failed.
 **/
unsigned int MMCSDTranSpeedSet(mmcsdCtrlInfo *ctrl)
{
    mmcsdCardInfo *card = ctrl->card;
    unsigned int speed;
    int status;
    unsigned int cmdStatus = 0;
    mmcsdCmd cmd;

    /* Returns 1 for a SD card, 0 for a non-SD card */
	status = MMCSDCardTypeCheck(ctrl);

	if (status == 1)
	/* SD Card */
	{
		ctrl->xferSetup(ctrl, 1, dataBuffer, 64, 1);

		cmd.idx = SD_CMD(6);
		cmd.arg = ((SD_SWITCH_MODE & SD_CMD6_GRP1_SEL) | (SD_CMD6_GRP1_HS));
		cmd.flags = SD_CMDRSP_READ | SD_CMDRSP_DATA;
		cmd.nblks = 1;
		cmd.data = (unsigned char*)dataBuffer;

		cmdStatus = MMCSDCmdSend(ctrl, &cmd);

		if (cmdStatus == 0)
		{
			return 0;
		}

		cmdStatus = ctrl->xferStatusGet(ctrl);

		if (cmdStatus == 0)
		{
			return 0;
		}

		/* Invalidate the data cache. */
		CacheDataInvalidateBuff((unsigned int) dataBuffer, DATA_RESPONSE_WIDTH);

		speed = card->tranSpeed;

		if ((dataBuffer[16] & 0xF) == SD_CMD6_GRP1_HS)
		{
			card->tranSpeed = SD_TRANSPEED_50MBPS;
		}

		if (speed == SD_TRANSPEED_50MBPS)
		{
			status = ctrl->busFreqConfig(ctrl, 50000000);
			ctrl->opClk = 50000000;
		}
		else
		{
			status = ctrl->busFreqConfig(ctrl, 25000000);
			ctrl->opClk = 25000000;
		}

		if (status != 0)
		{
			return 0;
		}

		return 1;
	}
	else
	{
		/* eMMC */

		ctrl->xferSetup(ctrl, 1, dataBuffer, 512, 1);

		cmd.idx = SD_CMD(8);
		cmd.flags = SD_CMDRSP_DATA | SD_CMDRSP_READ;
		cmd.arg = 0;
		cmd.nblks = 1;
		cmd.data = (unsigned char*)dataBuffer;

		status = MMCSDCmdSend(ctrl,&cmd);
		if (status == 0)
		{
			UARTPuts("error CMD8 \n", -1);
			return 0;
		}


		status = ctrl->xferStatusGet(ctrl);

		if (status == 0)
		{
			UARTPuts("error xferStatusGet \n", -1);
			return 0;
		}

		/* Invalidate the data cache. */
		CacheDataInvalidateBuff((unsigned int)dataBuffer, 512);

		speed = card->tranSpeed;

		// If high speed is enabled
		if ((dataBuffer[185]) == SD_CMD6_GRP1_HS)
		{
			card->tranSpeed = SD_TRANSPEED_50MBPS;
			status = ctrl->busFreqConfig(ctrl, 52000000);
			ctrl->opClk = 52000000;
		}

		else if(card->tranSpeed == SD_TRANSPEED_25MBPS)
		{
			status = ctrl->busFreqConfig(ctrl, 26000000);
			ctrl->opClk = 26000000;
		}

		else
		{
			// unknown bus speed
			return 0;
		}

		if (status != 0)
		{
			return 0;
		}
	}
    return 1;
}

/**
 * \brief   This function resets the MMCSD card.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  1 - successfull reset of card.
 *           0 - fails to reset the card.
 **/
unsigned int MMCSDCardReset(mmcsdCtrlInfo *ctrl)
{
    unsigned int status = 0;
    mmcsdCmd cmd;

    cmd.idx = SD_CMD(0);
    cmd.flags = SD_CMDRSP_NONE;
    cmd.arg = 0x0;

    status = MMCSDCmdSend(ctrl, &cmd);

    return status;
}

/**
 * \brief   This function sends the stop command to MMCSD card.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  1 - successfully sends stop command to card.
 *           0 - fails to send stop command to card.
 **/
unsigned int MMCSDStopCmdSend(mmcsdCtrlInfo *ctrl)
{
    unsigned int status = 0;
    mmcsdCmd cmd;

    cmd.idx  = SD_CMD(12);
    cmd.flags = SD_CMDRSP_BUSY;
    cmd.arg = 0;

    MMCSDCmdSend(ctrl, &cmd);

    /* Get transfer status */
    status = ctrl->xferStatusGet(ctrl);

    return status;
}

/**
 * \brief   This function determines the type of MMCSD card.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  type of the MMCSD card
 *         
 **/
unsigned int MMCSDCardTypeCheck(mmcsdCtrlInfo * ctrl)
{
    unsigned int status;
    mmcsdCmd cmd;

    /* 
     * Card type can be found by sending CMD55. If the card responds,
     * it is a SD card. Else, we assume it is a MMC Card
     */

    cmd.idx = SD_CMD(55);
    cmd.flags = 0;
    cmd.arg = 0;
    status = MMCSDAppCmdSend(ctrl, &cmd);

    return status;
}

/**
 * \brief   This function intializes the mmcsdcontroller.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  1 - Intialization is successfull.
 *           0 - Intialization is failed.
 **/
unsigned int MMCSDCtrlInit(mmcsdCtrlInfo *ctrl)
{
    return ctrl->ctrlInit(ctrl);
}

/**
 * \brief   This function determines whether card is persent or not.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  1 - Card is present.
 *           0 - Card is not present.
 **/
unsigned int MMCSDCardPresent(mmcsdCtrlInfo *ctrl)
{
    return ctrl->cardPresent(ctrl);
}

/**
 * \brief   Enables the controller events to generate a h/w interrupt request
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \return   none
 *
 **/
void MMCSDIntEnable(mmcsdCtrlInfo *ctrl)
{
    ctrl->intrEnable(ctrl);

    return;
}

/**
 * \brief   This function intializes the MMCSD Card.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 *
 * \returns  1 - Intialization is successfull.
 *           0 - Intialization is failed.
 **/
unsigned int MMCSDCardInit(mmcsdCtrlInfo *ctrl)
{

    mmcsdCardInfo *card = ctrl->card;
    unsigned int retry = 0xFFFF;
    unsigned int status = 0;
    mmcsdCmd cmd;

    memset(ctrl->card, 0, sizeof(mmcsdCardInfo));

    card->ctrl = ctrl;

    /* CMD0 - reset card */
    status = MMCSDCardReset(ctrl);

    if (status == 0)
    {
        return 0;
    }

    /* Returns 1 for a SD card, 0 for a non-SD card */
//    status = MMCSDCardTypeCheck(ctrl);

    if (status == 1)
    /* SD Card */
    {
        ctrl->card->cardType = MMCSD_CARD_SD;

        /* CMD0 - reset card */
        status = MMCSDCardReset(ctrl);

        if (status == 0)
        {
            return 0;
        }

        /* CMD8 - send oper voltage */
        cmd.idx = SD_CMD(8);
        cmd.flags = 0;
        cmd.arg = (SD_CHECK_PATTERN | SD_VOLT_2P7_3P6);

        status = MMCSDCmdSend(ctrl, &cmd);

        if (status == 0)
        {
            /* If the cmd fails, it can be due to version < 2.0, since
             * we are currently supporting high voltage cards only
             */
        }

        /* Go ahead and send ACMD41, with host capabilities */
        cmd.idx = SD_CMD(41);
        cmd.flags = 0;
        cmd.arg = SD_OCR_HIGH_CAPACITY | SD_OCR_VDD_WILDCARD;

        status = MMCSDAppCmdSend(ctrl,&cmd);

        if (status == 0)
        {
            return 0;
        }

        /* Poll until we get the card status (BIT31 of OCR) is powered up */
        do {
                cmd.idx = SD_CMD(41);
                cmd.flags = 0;
                cmd.arg = SD_OCR_HIGH_CAPACITY | SD_OCR_VDD_WILDCARD;

                MMCSDAppCmdSend(ctrl,&cmd);

        } while (!(cmd.rsp[0] & ((unsigned int)BIT(31))) && retry--);

        if (retry == 0)
        {
            /* No point in continuing */
            return 0;
        }

        card->ocr = cmd.rsp[0];

        card->highCap = (card->ocr & SD_OCR_HIGH_CAPACITY) ? 1 : 0;

        /* Send CMD2, to get the card identification register */
        cmd.idx = SD_CMD(2);
        cmd.flags = SD_CMDRSP_136BITS;
        cmd.arg = 0;

        status = MMCSDCmdSend(ctrl,&cmd);

        memcpy(card->raw_cid, cmd.rsp, 16);

        if (status == 0)
        {
            return 0;
        }

        /* Send CMD3, to get the card relative address */
        cmd.idx = SD_CMD(3);
        cmd.flags = 0;
        cmd.arg = 0;

        status = MMCSDCmdSend(ctrl,&cmd);

        card->rca = SD_RCA_ADDR(cmd.rsp[0]);

        if (status == 0)
        {
            return 0;
        }

        /* Send CMD9, to get the card specific data */
        cmd.idx = SD_CMD(9);
        cmd.flags = SD_CMDRSP_136BITS;
        cmd.arg = card->rca << 16;

        status = MMCSDCmdSend(ctrl,&cmd);

        memcpy(card->raw_csd, cmd.rsp, 16);

        if (status == 0)
        {
            return 0;
        }

        if (SD_CARD_CSD_VERSION(card))
        {
            card->tranSpeed = SD_CARD1_TRANSPEED(card);
            card->blkLen = 1 << (SD_CARD1_RDBLKLEN(card));
            card->size = SD_CARD1_SIZE(card);
            card->nBlks = card->size / card->blkLen;
        }
        else
        {
            card->tranSpeed = SD_CARD0_TRANSPEED(card);
            card->blkLen = 1 << (SD_CARD0_RDBLKLEN(card));
            card->nBlks = SD_CARD0_NUMBLK(card);
            card->size = SD_CARD0_SIZE(card);

        }

        /* Set data block length to 512 (for byte addressing cards) */
        if( !(card->highCap) )
        {
            cmd.idx = SD_CMD(16);
            cmd.flags = SD_CMDRSP_NONE;
            cmd.arg = 512;
            status = MMCSDCmdSend(ctrl,&cmd);

            if (status == 0)
            {
                return 0;
            }
            else
            {
                card->blkLen = 512;
            }
        }

        /* Select the card */
        cmd.idx = SD_CMD(7);
        cmd.flags = SD_CMDRSP_BUSY;
        cmd.arg = card->rca << 16;

        status = MMCSDCmdSend(ctrl,&cmd);

        if (status == 0)
        {
            return 0;
        }

        /*
         * Send ACMD51, to get the SD Configuration register details.
         * Note, this needs data transfer (on data lines).
         */
        cmd.idx = SD_CMD(55);
        cmd.flags = 0;
        cmd.arg = card->rca << 16;

        status = MMCSDCmdSend(ctrl,&cmd);
        if (status == 0)
        {
            return 0;
        }

        ctrl->xferSetup(ctrl, 1, dataBuffer, 8, 1);

        cmd.idx = SD_CMD(51);
        cmd.flags = SD_CMDRSP_READ | SD_CMDRSP_DATA;
        cmd.arg = card->rca << 16;
        cmd.nblks = 1;
        cmd.data = (unsigned char*)dataBuffer;

        status = MMCSDCmdSend(ctrl,&cmd);
        if (status == 0)
        {
            return 0;
        }

        status = ctrl->xferStatusGet(ctrl);

        if (status == 0)
        {
            return 0;
        }

        /* Invalidate the data cache. */
        CacheDataInvalidateBuff((unsigned int)dataBuffer, DATA_RESPONSE_WIDTH);

        card->raw_scr[0] = (dataBuffer[3] << 24) | (dataBuffer[2] << 16) | \
		                   (dataBuffer[1] << 8) | (dataBuffer[0]);
        card->raw_scr[1] = (dataBuffer[7] << 24) | (dataBuffer[6] << 16) | \
                                   (dataBuffer[5] << 8) | (dataBuffer[4]);

        card->sd_ver = SD_CARD_VERSION(card);
        card->busWidth = SD_CARD_BUSWIDTH(card);
    }
    else
    /* onboard mmc */
    {
    	ctrl->card->cardType = MMCSD_CARD_MMC;

    	/* eMMC reconfigure the controller */
    	ctrl->busWidthConfig(ctrl, HS_MMCSD_BUS_WIDTH_1BIT);
    	ctrl->card->busWidth = 1;




		/*
		 * After receiving Command GO_IDLE_STATE (CMD0 with argument of 0x00000000),
		 * the cards go to Idle State.
		 */
		status = MMCSDCardReset(ctrl);

		if (status == 0)
		{
			UARTPuts("error GO_IDLE_STATE \n", -1);
			return 0;
		}

		/*
		 * The SEND_OP_COND (CMD1) command is designed to provide MultiMediaCard hosts with a
		 * mechanism to identify and reject cards which do not match the VDD range desired by the host. This is
		 * accomplished by the host sending the required VDD voltage window as the operand of this command
		 *
		 * For e�MMC devices, the voltage range in CMD1 is no longer valid. Regardless of the voltage range
		 * indicated by the host, the e�MMC devices shall respond with a fixed pattern of either 0x00FF 8080
		 * (capacity less than or equal to 2GB) or 0x40FF 8080 (capacity greater than 2GB) if device is busy, and
		 * they shall not move into Inactive state.
		 *
		 * For e�MMC devices, the host shall still send the correct Access mode in CMD1 argument.
		 */



		/* loop until the card is not busy */
		do {

			cmd.idx = SD_CMD(1);
			cmd.arg = 0x40FF8000;
			cmd.flags = SD_CMDRSP_R3;

			status = MMCSDCmdSend(ctrl, &cmd);

			if (status == 0)
			{
				UARTPuts("error CMD1 \n", -1);
				return 0;
			}
			int delay = 0xFF;
			while(delay--); // wait for it to response

		} while (!(cmd.rsp[0] & ((unsigned int)BIT(31))) && retry--);

		if(retry == 0)
		{
			UARTPuts("Timeout\n", -1);
			return 0;
		}

		card->ocr = cmd.rsp[0];
		/* The extra bit indicates if we support high capacity */
		card->highCap = (card->ocr & SD_OCR_HIGH_CAPACITY) ? 1 : 0;


		/*
		 * Fetch CID from card.
		 * ALL_SEND_CID (CMD2), asking all cards for their unique card identification (CID) number
		 */

		/* Send CMD2, to get the card identification register */
		cmd.idx = SD_CMD(2);
		cmd.flags = SD_CMDRSP_R2;
		cmd.arg = 0;

		status = MMCSDCmdSend(ctrl,&cmd);

		if (status == 0)
		{
			UARTPuts("error CMD2 \n", -1);
			return 0;
		}


		/*
		 * Save response in the allocate card structure.
		 */
		memcpy(card->raw_cid, cmd.rsp, 16);


		/* save the raw data in a structure */
		getCID(ctrl);

		/* Send CMD3, to set the card relative address
		 *
		 * The writable 16-bit relative card address (RCA) register carries the card address assigned by the host during
		 * the card identification. This address is used for the addressed host-card communication after the card
		 * identification procedure. The default value of the RCA register is 0x0001. The value 0x0000 is reserved to
		 * set all cards into the Stand-by State with CMD7.
		 *
		 */

		/* Set the default value of the RCA register */
		card->rca = 1;

		cmd.idx = SD_CMD(3);
		cmd.flags = SD_CMDRSP_R1;
		cmd.arg = card->rca << 16;

		status = MMCSDCmdSend(ctrl,&cmd);

		if (status == 0)
		{
			UARTPuts("error CMD3 \n", -1);
			return 0;
		}

		/* Send CMD9, to get the card specific data */
		cmd.idx = SD_CMD(9);
		cmd.flags = SD_CMDRSP_R2;
		cmd.arg = card->rca << 16;

		status = MMCSDCmdSend(ctrl,&cmd);

		memcpy(card->raw_csd, cmd.rsp, 16);

		if (status == 0)
		{
			UARTPuts("error CMD9 \n", -1);
			return 0;
		}

		GetCSDParameters(ctrl);


		/*
		 * Select card, as all following commands rely on that.
		 */
		cmd.idx = SD_CMD(7);
		cmd.flags = SD_CMDRSP_R1;
		cmd.arg = card->rca << 16;

		status = MMCSDCmdSend(ctrl,&cmd);

		if (status == 0)
		{
			UARTPuts("error CMD7 \n", -1);
			return 0;
		}

		/*
		 * Now that the card is selected we can read the extended CSD.
		 */

		//    xferSetup(mmcsdCtrlInfo, rwFlag,*ptr,blkSize,nBlks)
		ctrl->xferSetup(ctrl, 1, dataBuffer, 512, 1);

		cmd.idx = SD_CMD(8);
		cmd.flags = SD_CMDRSP_DATA | SD_CMDRSP_READ;
		cmd.arg = 0;
		cmd.nblks = 1;
		cmd.data = (unsigned char*)dataBuffer;

		status = MMCSDCmdSend(ctrl,&cmd);
		if (status == 0)
		{
			UARTPuts("error CMD8 \n", -1);
			return 0;
		}


		status = ctrl->xferStatusGet(ctrl);

		if (status == 0)
		{
			UARTPuts("error xferStatusGet \n", -1);
			return 0;
		}

		/* Invalidate the data cache. */
		CacheDataInvalidateBuff((unsigned int)dataBuffer, 512);

		decodeExtCsd(ctrl,dataBuffer);

		/* enable High speed */
		if(mmc_switch(ctrl,MMC_SWITCH_MODE_WRITE_BYTE,EXT_CSD_HS_TIMING,0x1))
		{
			ctrl->highspeed = 1;
		}

		/* Set the bus width to 8 */
		if(mmc_switch(ctrl,MMC_SWITCH_MODE_WRITE_BYTE,EXT_CSD_BUS_WIDTH,EXT_CSD_BUS_WIDTH_8))
		{
			card->busWidth = 8;
			ctrl->busWidthConfig(ctrl, HS_MMCSD_BUS_WIDTH_8BIT);
		}

    }

    return 1;
}

/**
 * \brief   This function sends the write command to MMCSD card.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 * \param    ptr           It determines the address from where data has to written
 * \param    block         It determines to which block data to be written
 * \param    nblks         It determines the number of blocks to be written
 *
 * \returns  1 - successfull written of data.
 *           0 - failure to write the data.
 **/
unsigned int MMCSDWriteCmdSend(mmcsdCtrlInfo *ctrl, void *ptr, unsigned int block,
                               unsigned int nblks)
{
    mmcsdCardInfo *card = ctrl->card;
    unsigned int status = 0;
    unsigned int address;
    mmcsdCmd cmd;

    /*
     * Address is in blks for high cap cards and in actual bytes
     * for standard capacity cards
     */
    if (card->highCap)
    {
        address = block;
    }
    else
    {
        address = block * card->blkLen;
    }

    /* Clean the data cache. */
    CacheDataCleanBuff((unsigned int) ptr, (512 * nblks));

    ctrl->xferSetup(ctrl, 0, ptr, 512, nblks);

    cmd.flags = SD_CMDRSP_WRITE | SD_CMDRSP_DATA;
    cmd.arg = address;
    cmd.nblks = nblks;

    if (nblks > 1)
    {
        cmd.idx = SD_CMD(25);
        cmd.flags |= SD_CMDRSP_ABORT;
    }
    else
    {
        cmd.idx = SD_CMD(24);
    }


    status = MMCSDCmdSend(ctrl, &cmd);

    if (status == 0)
    {
        return 0;
    }

    status = ctrl->xferStatusGet(ctrl);

    if (status == 0)
    {
        return 0;
    }

    /* Send a STOP */
    if (cmd.nblks > 1)
    {
        status = MMCSDStopCmdSend(ctrl);

        if (status == 0)
        {
            return 0;
        }
    }

    return 1;
}

/**
 * \brief   This function sends the write command to MMCSD card.
 *
 * \param    mmcsdCtrlInfo It holds the mmcsd control information.
 * \param    ptr           It determines the address to where data has to read
 * \param    block         It determines from which block data to be read
 * \param    nblks         It determines the number of blocks to be read
 *
 * \returns  1 - successfull reading of data.
 *           0 - failure to the data.
 **/
unsigned int MMCSDReadCmdSend(mmcsdCtrlInfo *ctrl, void *ptr, unsigned int block,
                              unsigned int nblks)
{
    mmcsdCardInfo *card = ctrl->card;
    unsigned int status = 0;
    unsigned int address;
    mmcsdCmd cmd;

    /*
     * Address is in blks for high cap cards and in actual bytes
     * for standard capacity cards
     */
    if (card->highCap)
    {
        address = block;
    }
    else
    {
        address = block * card->blkLen;
    }

    ctrl->xferSetup(ctrl, 1, ptr, 512, nblks);

    cmd.flags = SD_CMDRSP_READ | SD_CMDRSP_DATA;
    cmd.arg = address;
    cmd.nblks = nblks;

    if (nblks > 1)
    {
        cmd.flags |= SD_CMDRSP_ABORT;
        cmd.idx = SD_CMD(18);
    }
    else
    {
        cmd.idx = SD_CMD(17);
    }

    status = MMCSDCmdSend(ctrl, &cmd);
    if (status == 0)
    {
        return 0;
    }

    status = ctrl->xferStatusGet(ctrl);

    if (status == 0)
    {
        return 0;
    }

    /* Send a STOP */
    if (cmd.nblks > 1)
    {
        status = MMCSDStopCmdSend(ctrl);

        if (status == 0)
        {
            return 0;
        }
    }

    /* Invalidate the data cache. */
    CacheDataInvalidateBuff((unsigned int) ptr, (512 * nblks));

    return 1;
}


/**
 *    mmc_switch - modify EXT_CSD register
 *    card: the MMC card associated with the data transfer
 *    set: cmd set values
 *    index: EXT_CSD register index
 *    value: value to program into EXT_CSD register
 *
 *
 *    Modifies the EXT_CSD register for selected card.
 */
int mmc_switch(mmcsdCtrlInfo *ctrl, char set, char index, char value)
{
	mmcsdCmd cmd;
	unsigned int status = 0;
	int err;
    cmd.idx = SD_CMD(6);
    cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
          (index << 16) |
          (value << 8) |
          set;
    cmd.flags = SD_CMDRSP_R1b;

    err = MMCSDCmdSend(ctrl,&cmd);

	if (err == 0)
	{
		UARTPuts("error CMD6 \n", -1);
		return 0;
	}

    /* Must check status to be sure of no errors */
    do
    {
        err = mmc_send_status(ctrl, &status);
        if (err)
        {
        	UARTPuts("error status failed\n", -1);
            //return 0;
        }

        if(status & (BIT(7)))
        {
        	// switch error
        	UARTPuts("switch error\n", -1);
        	return 0;
        }

        if(status & (BIT(22)))
	   {
        	// Illegal command
        	UARTPuts("Illegal command\n", -1);
        	return 0;
	   }

    } while (!(status & BIT(8)) & (!err)); // run while the card is not ready


    return 1;
}


int mmc_send_status(mmcsdCtrlInfo *ctrl, unsigned int *status)
{
    int err;
    mmcsdCmd cmd;

    cmd.idx = SD_CMD(13);
    cmd.arg = ctrl->card->rca << 16;
    cmd.flags = SD_CMDRSP_R1;

    err = MMCSDCmdSend(ctrl, &cmd);
	if (err == 0)
	{
		return 0;
	}

    *status = cmd.rsp[0];

    return 1;
}
