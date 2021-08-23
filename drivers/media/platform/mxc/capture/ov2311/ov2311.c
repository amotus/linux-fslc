/*
 * ov2311.c - ov2311 sensor driver
 * Copyright (c) 2018-2019, e-con Systems.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "ov2311.h"
#include "cam_firmware.h"

#define ov2311_DEBUG 0

/*
 * General TODO:
 *
 * Improve consistency: use either u8, u16 etc. (OR) uint8_t, uint16_t etc.
 */

/*
 * Note about retries for MCU functions:
 *
 * We faced some spurious I2C failures in various places. To avoid that,
 * we retry all those operations a few times as it is possible that the
 * I2C communication would succeed in one of the other attempts.
 */

/*!
 * Maintains the information on the current state of the sensor.
 */
static struct ov2311 ov2311_data;

/*
 * TODO: Shouldn't the gpio pins be moved to the struct that holds
 * sensor state
 */
static int pwdn_gpio, reset_gpio;

/**********************************************************************
 *
 * START of ov2311 related code
 *
 **********************************************************************
 */

static int retries_for_i2c_commands = 5;

static int ov2311_write(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register.\n");
		dev_err(&client->dev, "addr: %x; val = %hhu, ret = %d!\n",
			client->addr, *val, ret);
		return ret;
	}

	return 0;
}

static int ov2311_read(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = val,
	};

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

 err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}

/*
 * The MCU specific functions below depend on the sensor-specific
 * functions above.
 */

/*
 * ---------------------------------------------------------
 *  START of MCU realed functions
 * ---------------------------------------------------------
 */

/*
 * NOTE about modularizing this MCU related function.
 *
 * - The functions:
 *
 *  	x mcu_get_fw_version
 *      x mcu_bload_get_version
 *      x mcu_bload_erase_flash
 *      x mcu_bload_parse_send_cmd
 *      x mcu_bload_go
 *      x mcu_bload_read
 *      x mcu_count_or_list_ctrls
 *      x mcu_count_or_list_fmts
 *      x mcu_get_sensor_id
 *      x mcu_get_ctrl_ui
 *      x mcu_stream_config
 *      x mcu_isp_power_down
 *      x mcu_isp_power_wakeup
 *      x mcu_set_ctrl
 *      x mcu_get_ctrl
 *
 *   seem to directly use platform specific functions:
 *
 *   	x ov2311_write
 *   	x ov2311_read
 *
 *   This could be passed in as a function pointer.
 */

static unsigned short int mcu_bload_calc_crc16(unsigned char *buf, int len)
{
	unsigned short int crc = 0;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		crc ^= buf[i];
	}

	return crc;
}

static int mcu_bload_ascii2hex(unsigned char ascii)
{
	if (ascii <= '9') {
		return (ascii - '0');
	} else if ((ascii >= 'a') && (ascii <= 'f')) {
		return (0xA + (ascii - 'a'));
	} else if ((ascii >= 'A') && (ascii <= 'F')) {
		return (0xA + (ascii - 'A'));
	}
	return -1;
}

static unsigned char errorcheck(char *data, unsigned int len)
{
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
	}

	return crc;
}

/*
 * mcu_get_fw_version:
 *
 * Read the firmware version from the MCU.
 *
 * A success value (0) is returned when the MCU version could be successfully read.
 * else a negative value indicating error is returned.
 */
static int mcu_get_fw_version(struct i2c_client *client, unsigned char *fw_version)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop;
	/* Query firmware version from MCU */

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_VERSION;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = ov2311_write(client, mc_data, TX_LEN_PKT);
	if (err != 0)
	{
		dev_err(&client->dev, "MCU CMD ID version Error-  %d\n", err);
		ret = -EIO;
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_VERSION;
	err = ov2311_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) MCU CMD ID Write PKT fw Version Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ov2311_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) MCU CMD ID Read PKT fw Version Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) MCU CMD ID fw Errcode - 0x%02x \n", __func__,
				__LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	/* Read the actual version from MCU*/
	payload_len =
	    ((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	memset(mc_ret_data, 0x00, payload_len);
	err = ov2311_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Version Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 32);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x \n", __func__,
				__LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	for (loop = 0 ; loop < VERSION_SIZE ; loop++ )
		*(fw_version+loop) = mc_ret_data[2+loop];

	ret = ERRCODE_SUCCESS;
exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

/**
 * mcu_verify_fw_version:
 *
 * Verify the firmware version obtained from the MCU and that found in the
 * firmware file present in our driver.
 *
 * The return value after verification is as follows:
 *
 *   - If the version number matches a success value (0) is returned.
 *
 *   - In case the version number mismatches, a negative value indicating error
 *     is returned.
 *
 *   - In case the  force update bit is set in firmware version in the firmware
 *     file, a positive value is returned.
 */
static int mcu_verify_fw_version(const unsigned char *const fw_version)
{
	int loop, i = 0, ret;
	char fw_version_in_file[32] = {0};
	unsigned long file_fw_pos = ARRAY_SIZE(g_mcu_fw_buf)-VERSION_FILE_OFFSET;

	/* Get Firmware version from the firmware file */
	for(loop = file_fw_pos; loop < (file_fw_pos+64); loop=loop+2) {
		*(fw_version_in_file+i) = (mcu_bload_ascii2hex(g_mcu_fw_buf[loop]) << 4 |
				mcu_bload_ascii2hex(g_mcu_fw_buf[loop+1]));
		i++;
	}

	/* Check for forced/always update field in the firmware version present in the firmware file */
	if(fw_version_in_file[17] == '1') {

#ifdef ov2311_DEBUG
		pr_info("Forced Update Enabled - Firmware Version - (%.32s) \n",
			fw_version);
#endif

		ret = 2;
	}
	else {
		for(i = 0; i < VERSION_SIZE; i++) {
			if(fw_version_in_file[i] != fw_version[i]) {

				pr_info("Previous Firmware Version - (%.32s)\n", fw_version);

				ret = -1;
				break;
			}
		}

		if (i == VERSION_SIZE)
			ret = ERRCODE_SUCCESS;
	}

	return ret;
}

static int mcu_bload_get_version(struct i2c_client *client)
{
	int ret = 0;

	/*----------------------------- GET VERSION -------------------- */

	/*   Write Get Version CMD */
	g_bload_buf[0] = BL_GET_VERSION;
	g_bload_buf[1] = ~(BL_GET_VERSION);

	ret = ov2311_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {	
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != 'y') {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed\n");
		return -1;
	}

	/* ---------------- GET VERSION END ------------------- */

	return 0;
}

static int mcu_bload_erase_flash(struct i2c_client *client)
{
	unsigned short int pagenum = 0x0000;
	int ret = 0, i = 0, checksum = 0;

	/* --------------- ERASE FLASH --------------------- */

	for (i = 0; i < NUM_ERASE_CYCLES; i++) {

		checksum = 0x00;
		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_ERASE_MEM_NS;
		g_bload_buf[1] = ~(BL_ERASE_MEM_NS);

		ret = ov2311_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ov2311_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf[0] = (MAX_PAGES - 1) >> 8;
		g_bload_buf[1] = (MAX_PAGES - 1) & 0xFF;
		g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];

		ret = ov2311_write(client, g_bload_buf, 3);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ov2311_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
			g_bload_buf[(2 * pagenum)] =
			    (pagenum + (i * MAX_PAGES)) >> 8;
			g_bload_buf[(2 * pagenum) + 1] =
			    (pagenum + (i * MAX_PAGES)) & 0xFF;
			checksum =
			    checksum ^ g_bload_buf[(2 * pagenum)] ^
			    g_bload_buf[(2 * pagenum) + 1];
		}
		g_bload_buf[2 * MAX_PAGES] = checksum;

		ret = ov2311_write(client, g_bload_buf, (2 * MAX_PAGES) + 1);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

 poll_busy:
		/*   Wait for ACK or NACK */
		ret = ov2311_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

#ifdef ov2311_DEBUG
		pr_info(" ERASE Sector %d success !! \n", i + 1);
#endif
	}

	/* ------------ ERASE FLASH END ----------------------- */

	return 0;
}

static unsigned char mcu_bload_inv_checksum(unsigned char *buf, int len)
{
	unsigned int checksum = 0x00;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		checksum = (checksum + buf[i]);
	}

	checksum &= (0xFF);
	return (~(checksum) + 1);
}

static int mcu_bload_parse_send_cmd(struct i2c_client *client,
			     unsigned char *bytearray, int rec_len)
{
	IHEX_RECORD *ihex_rec = NULL;
	unsigned char checksum = 0, calc_checksum = 0;
	int i = 0, ret = 0;

	if (!bytearray)
		return -1;

	ihex_rec = (IHEX_RECORD *) bytearray;
	ihex_rec->addr = htons(ihex_rec->addr);

	checksum = bytearray[rec_len - 1];

	calc_checksum = mcu_bload_inv_checksum(bytearray, rec_len - 1);
	if (checksum != calc_checksum) {
		dev_err(&client->dev," Invalid Checksum 0x%02x != 0x%02x !! \n",
		       checksum, calc_checksum);
		return -1;
	}

	if ((ihex_rec->rectype == REC_TYPE_ELA)
	    && (ihex_rec->addr == 0x0000)
	    && (ihex_rec->datasize = 0x02)) {
		/*   Upper 32-bit configuration */
		g_bload_flashaddr = (ihex_rec->recdata[0] <<
				     24) | (ihex_rec->recdata[1]
					    << 16);
#ifdef ov2311_DEBUG
		pr_info("Updated Flash Addr = 0x%08x \n",
			     g_bload_flashaddr);
#endif

	} else if (ihex_rec->rectype == REC_TYPE_DATA) {
		/*   Flash Data into Flashaddr */

		g_bload_flashaddr =
		    (g_bload_flashaddr & 0xFFFF0000) | (ihex_rec->addr);
		g_bload_crc16 ^=
		    mcu_bload_calc_crc16(ihex_rec->recdata, ihex_rec->datasize);

		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_WRITE_MEM_NS;
		g_bload_buf[1] = ~(BL_WRITE_MEM_NS);

		ret = ov2311_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ov2311_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
		g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
		g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
		g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
		g_bload_buf[4] =
		    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
		    g_bload_buf[3];

		ret = ov2311_write(client, g_bload_buf, 5);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ov2311_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf[0] = ihex_rec->datasize - 1;
		checksum = g_bload_buf[0];
		for (i = 0; i < ihex_rec->datasize; i++) {
			g_bload_buf[i + 1] = ihex_rec->recdata[i];
			checksum ^= g_bload_buf[i + 1];
		}

		g_bload_buf[i + 1] = checksum;

		ret = ov2311_write(client, g_bload_buf, i + 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

 poll_busy:
		/*   Wait for ACK or NACK */
		ret = ov2311_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

	} else if (ihex_rec->rectype == REC_TYPE_SLA) {
		/*   Update Instruction pointer to this address */

	} else if (ihex_rec->rectype == REC_TYPE_EOF) {
		/*   End of File - Issue I2C Go Command */
		return 0;
	} else {

		/*   Unhandled Type */
		dev_err(&client->dev,"Unhandled Command Type \n");
		return -1;
	}

	return 0;
}

static int mcu_bload_update_fw(struct i2c_client *client)
{
	/* exclude NULL character at end of string */
	unsigned long hex_file_size = ARRAY_SIZE(g_mcu_fw_buf) - 1;
	unsigned char wbuf[MAX_BUF_LEN];
	int i = 0, recindex = 0, ret = 0;

	for (i = 0; i < hex_file_size; i++) {
		if ((recindex == 0) && (g_mcu_fw_buf[i] == ':')) {
			/*  pr_info("Start of a Record \n"); */
		} else if (g_mcu_fw_buf[i] == CR) {
			/*   No Implementation */
		} else if (g_mcu_fw_buf[i] == LF) {
			if (recindex == 0) {
				/*   Parsing Complete */
				break;
			}

			/*   Analyze Packet and Send Commands */
			ret = mcu_bload_parse_send_cmd(client, wbuf, recindex);
			if (ret < 0) {
				dev_err(&client->dev,"Error in Processing Commands \n");
				break;
			}

			recindex = 0;

		} else {
			/*   Parse Rec Data */
			if ((ret = mcu_bload_ascii2hex(g_mcu_fw_buf[i])) < 0) {
				dev_err(&client->dev,
					"Invalid Character - 0x%02x !! \n",
				     g_mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] = (0xF0 & (ret << 4));
			i++;

			if ((ret = mcu_bload_ascii2hex(g_mcu_fw_buf[i])) < 0) {
				dev_err(&client->dev,
				    "Invalid Character - 0x%02x !!!! \n",
				     g_mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] |= (0x0F & ret);
			recindex++;
		}
	}

#ifdef ov2311_DEBUG
	pr_info("Program FLASH Success !! - CRC = 0x%04x \n",
		     g_bload_crc16);
#endif

	/* ------------ PROGRAM FLASH END ----------------------- */

	return ret;
}

static int mcu_bload_read(struct i2c_client *client,
		   unsigned int g_bload_flashaddr, char *bytearray,
		   unsigned int len)
{
	int ret = 0;

	g_bload_buf[0] = BL_READ_MEM;
	g_bload_buf[1] = ~(BL_READ_MEM);

	ret = ov2311_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
	g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
	g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
	g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
	g_bload_buf[4] =
	    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = ov2311_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf[0] = len - 1;
	g_bload_buf[1] = ~(len - 1);

	ret = ov2311_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	ret = ov2311_read(client, bytearray, len);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	return 0;
}

static int mcu_bload_verify_flash(struct i2c_client *client,
			   unsigned short int orig_crc)
{
	char bytearray[FLASH_READ_LEN];
	unsigned short int calc_crc = 0;
	unsigned int flash_addr = FLASH_START_ADDRESS, i = 0;

	while ((i + FLASH_READ_LEN) <= FLASH_SIZE) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read(
			client, flash_addr + i, bytearray, FLASH_READ_LEN) < 0) {
			dev_err(&client->dev," i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
		i += FLASH_READ_LEN;
	}

	if ((FLASH_SIZE - i) > 0) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read(
			client, flash_addr + i, bytearray, (FLASH_SIZE - i)
			) < 0) {
			dev_err(&client->dev," i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
	}

	if (orig_crc != calc_crc) {
		dev_err(&client->dev," CRC verification fail !! 0x%04x != 0x%04x \n",
		       orig_crc, calc_crc);
//		return -1;
	}

#ifdef ov2311_DEBUG
	pr_info(" CRC Verification Success 0x%04x == 0x%04x \n",
		     orig_crc, calc_crc);
#endif

	return 0;
}

static int mcu_bload_go(struct i2c_client *client)
{
	int ret = 0;

	g_bload_buf[0] = BL_GO;
	g_bload_buf[1] = ~(BL_GO);

	ret = ov2311_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Failed Read 1 \n");
		return -1;
	}

	/*   Start Address */
	g_bload_buf[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
	g_bload_buf[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
	g_bload_buf[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
	g_bload_buf[3] = (FLASH_START_ADDRESS & 0x000000FF);
	g_bload_buf[4] =
	    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = ov2311_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	ret = ov2311_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Failed Read 1 \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	return 0;
}

static int mcu_fw_update(struct i2c_client *client, unsigned char *mcu_fw_version)
{
	int ret = 0;
	g_bload_crc16 = 0;

	/*
	 * TODO: Is this necessary? It seems redundant as it's already called before
	 * calling this function.
	 */
	/* Read Firmware version from bootloader MCU */
	ret = mcu_bload_get_version(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Get Version \n");
		goto exit;
	}

#ifdef ov2311_DEBUG
	pr_info(" Get Version SUCCESS !! \n");
#endif

#if 1
	/* Erase firmware present in the MCU and flash new firmware*/
	ret = mcu_bload_erase_flash(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Erase Flash \n");
		goto exit;
	}

#ifdef ov2311_DEBUG
	pr_info("Erase Flash Success !! \n");
#endif

	/* Read the firmware present in the firmware file */
	if ((ret = mcu_bload_update_fw(client)) < 0) {
		dev_err(&client->dev," Write Flash FAIL !! \n");
		goto exit;
	}

	/* Verify the checksum for the update firmware */
	if ((ret = mcu_bload_verify_flash(client, g_bload_crc16)) < 0) {
		dev_err(&client->dev," verify_flash FAIL !! \n");
		goto exit;
	}

	/* Reverting from bootloader mode */
	/* I2C GO Command */
	if ((ret = mcu_bload_go(client)) < 0) {
		dev_err(&client->dev," i2c_bload_go FAIL !! \n");
		goto exit;
	}
#endif 
	if(mcu_fw_version) {

#ifdef ov2311_DEBUG
	pr_info("(%s) - Firmware Updated - (%.32s)\n",
				__func__, mcu_fw_version);
#endif

	}
 exit:
	return ret;
}

static int mcu_count_or_list_ctrls(struct i2c_client *client,
			  ISP_CTRL_INFO * mcu_cam_ctrl, int *numctrls)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0;
	int ret = 0, err = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	/* Array of Ctrl Info */
	while (1) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		ov2311_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = ov2311_write(client, mc_data, 5);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = ov2311_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
			    " %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			*numctrls = index;
			break;
		}

		payload_len =
		    ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
		    HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
			    " %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = ov2311_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
		    errorcheck(&mc_ret_data[2],
				 payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
			    " %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
			    " %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EINVAL;
			goto exit;
		}

		if(mcu_cam_ctrl != NULL) {
			int sorted_elem = index - 1, elem = index;

			/* append ctrl info in array */
			mcu_cam_ctrl[index].ctrl_id =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			mcu_cam_ctrl[index].ctrl_type = mc_ret_data[6];

			

			switch (mcu_cam_ctrl[index].ctrl_type) {
				case CTRL_STANDARD:
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_min =
						mc_ret_data[7] << 24 | mc_ret_data[8] << 16
						| mc_ret_data[9] << 8 | mc_ret_data[10];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_max =
						mc_ret_data[11] << 24 | mc_ret_data[12] <<
						16 | mc_ret_data[13]
						<< 8 | mc_ret_data[14];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_def =
						mc_ret_data[15] << 24 | mc_ret_data[16] <<
						16 | mc_ret_data[17]
						<< 8 | mc_ret_data[18];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_step =
						mc_ret_data[19] << 24 | mc_ret_data[20] <<
						16 | mc_ret_data[21]
						<< 8 | mc_ret_data[22];

					/*
					 * We need to save the index as the MCU protocol works
					 * based on the index but we have to sort the list in
					 * the driver to ensure the control IDs are in sequence.
					 *
					 * So, the index of the controls in our list and that of
					 * the MCU's list don't match. We save this index so
					 * that we could use it to get/control the corresponding
					 * control using the index it knows.
					 */
					mcu_cam_ctrl[index].mcu_ctrl_index = index;
					break;

				case CTRL_EXTENDED:
					/* Not Implemented */
					break;
			}

#ifdef ov2311_DEBUG
			pr_info("Control: ID: 0x%x;Name:%s Type: %u; min: %d; Max: %d; Def: %d; Step: %u\n",
					mcu_cam_ctrl[index].ctrl_id,
					mcu_cam_ctrl[index].ctrl_ui_data.ctrl_ui_info.ctrl_name,
					mcu_cam_ctrl[index].ctrl_type,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_min,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_max,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_def,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_step
			);
#endif

			ctrldb[index] = mcu_cam_ctrl[index].ctrl_id;

			/*
			 * Keep the control list and control db sorted.
			 */
			while(
				sorted_elem >= 0 &&
				(
					mcu_cam_ctrl[sorted_elem].ctrl_id >
					mcu_cam_ctrl[elem].ctrl_id
				)
			)
			{
				ISP_CTRL_INFO swap_ctrl_elem;
				uint32_t swap_ctrldb_elem;

				/*
				 * Swap the elements in the mcu_cam_ctrl list
				 */
				memcpy(&swap_ctrl_elem, (mcu_cam_ctrl + sorted_elem), sizeof(ISP_CTRL_INFO));
				memcpy((mcu_cam_ctrl + sorted_elem), (mcu_cam_ctrl + elem), sizeof(ISP_CTRL_INFO));
				memcpy((mcu_cam_ctrl + elem), &swap_ctrl_elem, sizeof(ISP_CTRL_INFO));

				/*
				 * Swap the elements in ctrldb
				 */
				swap_ctrldb_elem = ctrldb[sorted_elem];
				ctrldb[sorted_elem] = ctrldb[elem];
				ctrldb[elem] = swap_ctrldb_elem;

				elem = sorted_elem;
				sorted_elem = elem - 1;
			}
		}
		index++;
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

static int mcu_count_or_list_fmts(struct i2c_client *client, ISP_STREAM_INFO *stream_info, int *frm_fmt_size)
{
	uint32_t payload_len = 0, err = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0, skip = 0;
	uint16_t index = 0, mode = 0;

	int loop = 0, num_frates = 0, ret = 0;

	/* Stream Info Variables */

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	/* List all formats from MCU and append to mcu_ov2311_frmfmt array */

	for (index = 0;; index++) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		ov2311_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = ov2311_write(client, mc_data, 5);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = ov2311_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			if(stream_info == NULL) {
				*frm_fmt_size = index;
			} else {
				*frm_fmt_size = mode;
			}
			break;
		}

		payload_len =
		    ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
		    HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = ov2311_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
		    errorcheck(&mc_ret_data[2],
				 payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}
		if(stream_info != NULL) {
			stream_info->fmt_fourcc =
			    mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
			    << 8 | mc_ret_data[5];
			stream_info->width = mc_ret_data[6] << 8 | mc_ret_data[7];
			stream_info->height = mc_ret_data[8] << 8 | mc_ret_data[9];
			stream_info->frame_rate_type = mc_ret_data[10];

		//	printk(" The Stream format at index 0x%04x has format 0x%08x \n", index,
		//		     stream_info->fmt_fourcc);

			switch (stream_info->frame_rate_type) {
			case FRAME_RATE_DISCRETE:
				stream_info->frame_rate.disc.frame_rate_num =
				    mc_ret_data[11] << 8 | mc_ret_data[12];

				stream_info->frame_rate.disc.frame_rate_denom =
				    mc_ret_data[13] << 8 | mc_ret_data[14];

				break;

			case FRAME_RATE_CONTINOUS:
				dev_err(&client->dev,
					" The Stream format at index 0x%04x has FRAME_RATE_CONTINOUS,"
				     "which is unsupported !! \n", index);

				continue;
			}

			switch (stream_info->fmt_fourcc) {
			/*
			 * We check for UYVY here instead of YUYV as the output from the sensor
			 * is UYVY. We swap it to YUYV only making changes in the platform driver.
			 */ 
			
			case V4L2_PIX_FMT_GREY:
			case V4L2_PIX_FMT_Y16:
				/* ov2311_codes is already populated with V4L2_PIX_FMT_YUYV */
				/* check if width and height are already in array - update frame rate only */
				for (loop = 0; loop < (mode); loop++) {
					if ((ov2311_data.mcu_cam_frmfmt[loop].size.width ==
					     stream_info->width)
					    && (ov2311_data.mcu_cam_frmfmt[loop].size.height ==
						stream_info->height)
						&& (ov2311_data.mcu_cam_frmfmt[loop].pixel_format == stream_info->fmt_fourcc)){

						num_frates =
						    ov2311_data.mcu_cam_frmfmt[loop].num_framerates;
						*((int *)(ov2311_data.mcu_cam_frmfmt[loop].framerates) + num_frates)
						    = (int)(stream_info->frame_rate.
							    disc.frame_rate_num /
							    stream_info->frame_rate.
							    disc.frame_rate_denom);

						ov2311_data.mcu_cam_frmfmt[loop].num_framerates++;

						streamdb[index] = loop;
						skip = 1;
						break;
					}
				}

				/*
				 * FIXME: The following case never seems to be happening.
				 * Explore why it was added.
				 */
				if (skip) {
					skip = 0;
					continue;
				}

				/* Add Width, Height, Frame Rate array, Mode into mcu_ov2311_frmfmt array */
				ov2311_data.mcu_cam_frmfmt[mode].size.width = stream_info->width;
				ov2311_data.mcu_cam_frmfmt[mode].size.height = stream_info->height;
				ov2311_data.mcu_cam_frmfmt[loop].pixel_format = stream_info->fmt_fourcc;

				num_frates = ov2311_data.mcu_cam_frmfmt[mode].num_framerates;

				*(ov2311_data.mcu_cam_frmfmt[mode].framerates + num_frates) =
				    (int)(stream_info->frame_rate.disc.frame_rate_num /
					  stream_info->frame_rate.disc.frame_rate_denom);

				ov2311_data.mcu_cam_frmfmt[mode].num_framerates++;

				ov2311_data.mcu_cam_frmfmt[mode].mode = mode;
				ov2311_data.mcu_cam_frmfmt[mode].mode = mode;
				streamdb[index] = mode;
//				printk(" ov2311_data.mcu_cam_frmfmt[mode].size.width : %d ov2311_data.mcu_cam_frmfmt[mode].size.height : %d 
//					ov2311_data.mcu_cam_frmfmt[loop].pixel_format :0x%04x
//				ov2311_data.mcu_cam_frmfmt[mode].mode : %d 
//				streamdb[index] = mode : %d \n", ov2311_data.mcu_cam_frmfmt[mode].size.width,ov2311_data.mcu_cam_frmfmt[mode].size.height,ov2311_data.mcu_cam_frmfmt[loop].pixel_format,
//				 ov2311_data.mcu_cam_frmfmt[mode].mode, index);

				mode++;
				break;

			default:
				dev_err(&client->dev,
					" The Stream format at index 0x%04x has format 0x%08x ,"
				     "which is unsupported !! \n", index,
				     stream_info->fmt_fourcc);
			}
		}
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

/*
 * Function to initialise the data related to MCU. Needs to be called
 * before trying to use them.
 */
static int mcu_data_init(struct device *dev, int frm_fmt_size)
{
	int loop = 0;

	if (dev == NULL)
	{
		dev_err(dev, "%s: Invalid device parameter\n", __func__);
		return -EINVAL;
	}

	mcu_ctrl_info = devm_kzalloc(dev, sizeof(ISP_CTRL_INFO) * num_ctrls, GFP_KERNEL);
	if(!mcu_ctrl_info) {
		dev_err(dev, "Unable to allocate memory \n");
		return -ENOMEM;
	}

	ctrldb = devm_kzalloc(dev, sizeof(uint32_t) * num_ctrls, GFP_KERNEL);
	if(!ctrldb) {
		dev_err(dev, "Unable to allocate memory \n");
		return -ENOMEM;
	}

	stream_info = devm_kzalloc(dev, sizeof(ISP_STREAM_INFO) * (frm_fmt_size + 1), GFP_KERNEL);

	streamdb = devm_kzalloc(dev, sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if(!streamdb) {
		dev_err(dev,"Unable to allocate memory \n");
		return -ENOMEM;
	}

	ov2311_data.mcu_cam_frmfmt = devm_kzalloc(dev, sizeof(struct mcu_frmfmt) * (frm_fmt_size), GFP_KERNEL);
	if(!ov2311_data.mcu_cam_frmfmt) {
		dev_err(dev, "Unable to allocate memory \n");
		return -ENOMEM;
	}

	for(; loop < frm_fmt_size; loop++) {
		ov2311_data.mcu_cam_frmfmt[loop].framerates = devm_kzalloc(dev, sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if(!ov2311_data.mcu_cam_frmfmt[loop].framerates) {
			dev_err(dev, "Unable to allocate memory \n");
			return -ENOMEM;
		}
	}

	return 0;
}

static int mcu_get_sensor_id(struct i2c_client *client, uint16_t * sensor_id)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	err = ov2311_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ov2311_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = ov2311_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*sensor_id = mc_ret_data[2] << 8 | mc_ret_data[3];

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

static int mcu_get_cmd_status(struct i2c_client *client,
			      uint8_t * cmd_id, uint16_t * cmd_status,
			      uint8_t * ret_code)
{
	uint32_t payload_len = 0;
	uint8_t orig_crc = 0, calc_crc = 0;
	int err = 0;

	/* No Semaphore in Get command Status */

	/* First Txn Payload length = 0 */
	payload_len = 1;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = *cmd_id;
	err = ov2311_write(client, mc_data, 3);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	payload_len = CMD_STATUS_MSG_LEN;
	memset(mc_ret_data, 0x00, payload_len);
	err = ov2311_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 3);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		return -EINVAL;
	}

	*cmd_id = mc_ret_data[2];
	*cmd_status = mc_ret_data[3] << 8 | mc_ret_data[4];
	*ret_code = mc_ret_data[payload_len - 1];

	return 0;
}

static int mcu_isp_init(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/* check current status - if initialized, no need for Init */
	cmd_id = CMD_ID_INIT_CAM;
	if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
		dev_err(&client->dev," %s(%d) Error \n", __func__, __LINE__);
		return -EIO;
	}

	if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
	    (retcode == ERRCODE_SUCCESS)) {

#ifdef ov2311_DEBUG
		pr_info(" Already Initialized !! \n");
#endif

		return 0;
	}

	/* call ISP init command */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	err = ov2311_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		mdelay(5);

		cmd_id = CMD_ID_INIT_CAM;
		if (mcu_get_cmd_status(
			client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
			       __func__, __LINE__);
			return -EIO;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    ((retcode == ERRCODE_SUCCESS) || (retcode == ERRCODE_ALREADY))) {

#ifdef ov2311_DEBUG
			pr_info(" ISP Already Initialized !! \n");
#endif

			return 0;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
			    "(%s) %d Init Error STATUS = 0x%04x RET = 0x%02x\n",
			     __func__, __LINE__, cmd_status, retcode);
			return -EIO;
		}
	}

	return -ETIMEDOUT;
}

static int mcu_get_ctrl_ui(struct i2c_client *client,
			   ISP_CTRL_INFO * mcu_ui_info, int index)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, i = 0, err = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = ov2311_write(client, mc_data, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ov2311_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	memset(mc_ret_data, 0x00, payload_len);
	err = ov2311_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
	    errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_name, &mc_ret_data[2],MAX_CTRL_UI_STRING_LEN);

	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type = mc_ret_data[34];
	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags = mc_ret_data[35] << 8 |
	    mc_ret_data[36];

	if (mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type == V4L2_CTRL_TYPE_MENU) {
		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem = mc_ret_data[37];

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu =
		    devm_kzalloc(&client->dev,((mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem +1) * sizeof(char *)), GFP_KERNEL);
		for (i = 0; i < mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem; i++) {
			mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] =
			    devm_kzalloc(&client->dev,MAX_CTRL_UI_STRING_LEN, GFP_KERNEL);
			strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i],
				&mc_ret_data[38 +(i *MAX_CTRL_UI_STRING_LEN)], MAX_CTRL_UI_STRING_LEN);

#ifdef ov2311_DEBUG
			pr_info(" Menu Element %d : %s \n",
				     i, mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i]);
#endif

		}

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] = NULL;
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;

}

static int mcu_isp_configuration(uint8_t cmd_id, struct i2c_client *client)
{
    unsigned char mc_data[100];
	uint32_t payload_len = 0;

	uint16_t payload_data;
	uint16_t cmd_status = 0;
	uint8_t retcode = 0;
	int retry = 1000, err = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/*lock semaphore */
	mutex_lock(&mcu_i2c_mutex);
	/* First Txn Payload length = 0 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = cmd_id;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = cmd_id;

	switch(cmd_id) {
		case CMD_ID_LANE_CONFIG:
		        /* Lane Configuration */
			payload_data = ov2311_data.mipi_lane_config == 4 ? NUM_LANES_4 : NUM_LANES_2;
		        mc_data[2] = payload_data >> 8;
			mc_data[3] = payload_data & 0xff;
			break;
		case CMD_ID_MIPI_CLK_CONFIG:
		        /* MIPI CLK Configuration */
			payload_data = ov2311_data.mipi_clk_config;
		        mc_data[2] = payload_data >> 8;
			mc_data[3] = payload_data & 0xff;
			break;
		default:
	                dev_err(&client->dev,"MCU ISP CONF Error \n");
			err = -1;
	                goto exit;
	}

	/* CRC*/
	mc_data[4] = errorcheck(&mc_data[2], payload_len);
	err = ov2311_write(client, mc_data, payload_len+3);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}


	while (--retry > 0) {
		msleep(20);
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) Error \n",
				__func__, __LINE__);
			err = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_ISP_UNINIT) &&
		    ((retcode == ERRCODE_SUCCESS) || retcode == ERRCODE_ALREADY)) {
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_ISP_UNINIT))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			err = -EIO;
			goto exit;
		}

	}

	err = -ETIMEDOUT;
 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);
	return err;
}

static int mcu_stream_config(struct i2c_client *client, uint32_t format,
			     int mode, int frate_index)
{
	uint32_t payload_len = 0;
	
	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;
	static uint16_t prev_index = 0xFFFE;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	cmd_id = CMD_ID_STREAM_CONFIG;
	if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
		dev_err(&client->dev," %s(%d) Error \n", __func__, __LINE__);
		ret = -EIO;
		goto exit;
	}

	if ((cmd_status != MCU_CMD_STATUS_SUCCESS) ||
	    (retcode != ERRCODE_SUCCESS)) {
		dev_err(&client->dev,
			" ISP is Unintialized or Busy STATUS = 0x%04x Errcode = 0x%02x !! \n",
		     cmd_status, retcode);
		ret = -EBUSY;
		goto exit;
	}

	printk(" /********* func : %s format :  0x%8x\n\r", __func__, format);

	for (loop = 0;(&streamdb[loop]) != NULL; loop++) {
		printk(" loop : %d \n\r", loop);
		if (streamdb[loop] == mode) {
			if(format == V4L2_PIX_FMT_GREY )
			{
				printk(" format = 0x%8x loop = %d index = %d frate_index = %d\n\r", format, loop, index, frate_index);
				index = loop + frate_index;
			}else if(format == V4L2_PIX_FMT_Y16)
			{
				printk(" format = 0x%8x loop = %d index = %d frate_index = %d\n\r", format, loop, index, frate_index);
				index = loop + frate_index + 1;
			}
			else
				index = loop + frate_index;
			//else if(ov2311_data.mcu_cam_frmfmt[mode].size.width != 2560)
			//	index = loop + frate_index + 4;
			//else
			//	index = loop + frate_index;
			break;
		}
	}


	printk(" stream config# : %d \n\r", index);

#ifdef ov2311_DEBUG
	pr_info(" Index = 0x%04x , format = 0x%08x, width = %hu,"
		     " height = %hu, frate num = %hu \n", index, format,
		     ov2311_data.mcu_cam_frmfmt[mode].size.width,
		     ov2311_data.mcu_cam_frmfmt[mode].size.height,
		     ov2311_data.mcu_cam_frmfmt[mode].framerates[frate_index]);
#endif
	

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if(prev_index == index) {
#ifdef ov2311_DEBUG
		pr_info("Skipping Previous mode set ... \n");
#endif
		ret = 0;
		goto exit;
	}

issue_cmd:
	/* First Txn Payload length = 0 */
	payload_len = 14;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Format Fourcc - currently only YUYV */
	mc_data[4] = format >> 24;
	mc_data[5] = format >> 16;
	mc_data[6] = format >> 8;
	mc_data[7] = format & 0xFF;

	/* width */
	mc_data[8] = ov2311_data.mcu_cam_frmfmt[mode].size.width >> 8;
	mc_data[9] = ov2311_data.mcu_cam_frmfmt[mode].size.width & 0xFF;

	/* height */
	mc_data[10] = ov2311_data.mcu_cam_frmfmt[mode].size.height >> 8;
	mc_data[11] = ov2311_data.mcu_cam_frmfmt[mode].size.height & 0xFF;

	/* frame rate num */
	mc_data[12] = ov2311_data.mcu_cam_frmfmt[mode].framerates[frate_index] >> 8;
	mc_data[13] = ov2311_data.mcu_cam_frmfmt[mode].framerates[frate_index] & 0xFF;

	/* frame rate denom */
	mc_data[14] = 0x00;
	mc_data[15] = 0x01;

	mc_data[16] = errorcheck(&mc_data[2], 14);
	err = ov2311_write(client, mc_data, 17);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (--retry > 0) {
		cmd_id = CMD_ID_STREAM_CONFIG;
		if (mcu_get_cmd_status(
			client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev,
				" %s(%d) MCU GET CMD Status Error : loop : %d \n",
				__func__, __LINE__, loop);
			ret = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if(retcode == ERRCODE_AGAIN) {
			/* Issue Command Again if Set */
			retry = 1000;
			goto issue_cmd;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}

		/* Delay after retry */
		mdelay(10);
	}

	dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
			__LINE__, err);
	ret = -ETIMEDOUT;

exit:
	if(!ret)
		prev_index = index;

	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

static int mcu_isp_stream_on(struct i2c_client *client)
{
    unsigned char mc_data[100];
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/*lock semaphore */
	mutex_lock(&mcu_i2c_mutex);
	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_ON;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_ON;
	err = ov2311_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		msleep(20);
		cmd_id = CMD_ID_STREAM_ON;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) Error \n",
				__func__, __LINE__);
			err = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    ((retcode == ERRCODE_SUCCESS) || retcode == ERRCODE_ALREADY)) {
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			err = -EIO;
			goto exit;
		}

	}

	err = -ETIMEDOUT;
 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);
	return err;
}

static int mcu_isp_power_down(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;

	/*lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_PDOWN;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_PDOWN;
	err = ov2311_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		msleep(20);
		cmd_id = CMD_ID_ISP_PDOWN;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) Get Status Error \n",
				__func__, __LINE__);
			err = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_ISP_PWDN) &&
		    ((retcode == ERRCODE_SUCCESS) || retcode == ERRCODE_ALREADY)) {
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status,
				retcode);
			err = -EIO;
			goto exit;
		}

	}
	err = -ETIMEDOUT;
 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);
	return err;
}

static int mcu_isp_power_wakeup(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;

	/*lock semaphore */
	mutex_lock(&mcu_i2c_mutex);
	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_PUP;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_PUP;
	err = ov2311_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		msleep(20);
		cmd_id = CMD_ID_ISP_PUP;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) Error \n",
				__func__, __LINE__);
			err = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    ((retcode == ERRCODE_SUCCESS) || retcode == ERRCODE_ALREADY)) {
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			err = -EIO;
			goto exit;
		}

	}

	err = -ETIMEDOUT;
 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);
	return err;
}

static int mcu_set_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
			uint8_t ctrl_type, int32_t curr_val)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0;
	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	ctrl_id = arg_ctrl_id;

	/* call ISP Ctrl config command */

	for (loop = 0; loop < num_ctrls; loop++) {
		if (ctrldb[loop] == ctrl_id) {
			index = mcu_ctrl_info[loop].mcu_ctrl_index;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	/* First Txn Payload length = 0 */
	payload_len = 11;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;

	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Control ID */
	mc_data[4] = ctrl_id >> 24;
	mc_data[5] = ctrl_id >> 16;
	mc_data[6] = ctrl_id >> 8;
	mc_data[7] = ctrl_id & 0xFF;

	/* Ctrl Type */
	mc_data[8] = ctrl_type;

	/* Ctrl Value */
	mc_data[9] = curr_val >> 24;
	mc_data[10] = curr_val >> 16;
	mc_data[11] = curr_val >> 8;
	mc_data[12] = curr_val & 0xFF;

	/* CRC */
	mc_data[13] = errorcheck(&mc_data[2], 11);

	err = ov2311_write(client, mc_data, 14);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (1) {
		cmd_id = CMD_ID_SET_CTRL;
		if (mcu_get_cmd_status(
			client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
			       __func__, __LINE__);
			ret = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d ISP Error STATUS = 0x%04x RET = 0x%02x\n",
			     __func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

static int mcu_get_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
			uint8_t * ctrl_type, int32_t * curr_val)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0xFFFF;
	int loop = 0, ret = 0, err = 0;

	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex);

	ctrl_id = arg_ctrl_id;

	/* Read the Ctrl Value from Micro controller */

	for (loop = 0; loop < num_ctrls; loop++) {
		if (ctrldb[loop] == ctrl_id) {
			index = mcu_ctrl_info[loop].mcu_ctrl_index;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if (
		mcu_ctrl_info[loop].ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags &
		V4L2_CTRL_FLAG_WRITE_ONLY
	) {
		ret = -EACCES;
		goto exit;
	}

	/* First Txn Payload length = 2 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ov2311_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = ov2311_write(client, mc_data, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ov2311_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -1;
		goto exit;
	}

	if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
		ret = -EIO;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	memset(mc_ret_data, 0x00, payload_len);
	err = ov2311_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
	    errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	/* Ctrl type starts from index 6 */

	*ctrl_type = mc_ret_data[6];

	switch (*ctrl_type) {
	case CTRL_STANDARD:
		*curr_val =
		    mc_ret_data[7] << 24 | mc_ret_data[8] << 16 | mc_ret_data[9]
		    << 8 | mc_ret_data[10];
		break;

	case CTRL_EXTENDED:
		/* Not Implemented */
		break;
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex);

	return ret;
}

/*
 * ---------------------------------------------------------
 *  END of MCU realed functions
 * ---------------------------------------------------------
 */

/*
 * Macro to retry a specific sequence (a function call) for a specific
 * number of times to check if the sequence succeeds within the specified
 * number of tries.
 *
 * Arguments:
 * retries - number of retries for the sequence
 * sequence - the sequence to retry (typically a function call).
 *            It is assumed that the sequence returns a negative value in
 *            case of an error.
 *
 * Evaluation value:
 * - The macro evaluates to zero if the sequence succeeds within the specified
 *   number of tries.
 * - The macro evaluates to a negative value when the sequence does not succeed
 *   within the specified number of tries.
 *
 * WARNING: This macro relies on a GCC extension[1] so this might not work
 * when using other toolchains.
 *
 * [1]: "Statements and Declarations in Expressions"
 *      https://gcc.gnu.org/onlinedocs/gcc/Statement-Exprs.html
 */
#define RETRY_SEQUENCE(retries, sequence)		\
({							\
	int status = 0, err, retry;			\
	for (retry = 0; retry < retries; retry++) {	\
		err = sequence;				\
							\
		if (err < 0)				\
			msleep(5);			\
		else					\
			break;				\
	}						\
	if (retry == retries) {				\
		status = err;				\
	}						\
	status;						\
})

static void toggle_gpio(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio)){
		gpio_direction_output(gpio,val);
		gpio_set_value_cansleep(gpio, val);
	} else{
		gpio_direction_output(gpio,val);
		gpio_set_value(gpio, val);
	}
}

static int ov2311_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	uint32_t index = 0;
	int loop;

	if (sd == NULL || qm == NULL)
		return -EINVAL;

	for (loop = 0; loop < num_ctrls; loop++) {
		if (ctrldb[loop] == qm->id) {
			index = loop;
			break;
		}
	}

	if (loop == num_ctrls) {
		return -EINVAL;
	}

	if (
		!(
			0 <= qm->index &&
			qm->index < mcu_ctrl_info[index].ctrl_ui_data.ctrl_menu_info.num_menu_elem
		 )
	) {
		return -EINVAL;
	}

	/*
	 * Copy the name of the menu.
	 *
	 * We deal only with V4L2_CTRL_TYPE_MENU and not
	 * V4L2_CTRL_TYPE_INTEGER_MENU. So, this should be
	 * enough.
	 */
	strcpy(qm->name, mcu_ctrl_info[index].ctrl_ui_data.ctrl_menu_info.menu[qm->index]);

	/*
	 * Set the reserved to zero as mentioned in spec
	 */
	qm->reserved = 0;

	return 0;
}

static int ov2311_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int index, ctrl_index = -1, ctrl_id;
	bool next_ctrl = (qc->id & V4L2_CTRL_FLAG_NEXT_CTRL);

	if (sd == NULL || qc == NULL)
		return -EINVAL;

	if (next_ctrl) {
		ctrl_id = qc->id & (~V4L2_CTRL_FLAG_NEXT_CTRL);

		/*
		 * Ignore the V4L2_CTRL_FLAG_NEXT_COMPOUND for now
		 */
		ctrl_id = ctrl_id & (~V4L2_CTRL_FLAG_NEXT_COMPOUND);
	}
	else {
		/*
		 * Assume we've just got the control ID itself
		 * directly.
		 */
		ctrl_id = qc->id;
	}

	if (ctrl_id) {
		for (index = 0; index < num_ctrls; index++) {
			if (ctrldb[index] == ctrl_id) {
				ctrl_index = (next_ctrl) ? index + 1 : index;
				break;
			}
		}

		if (index == num_ctrls) {
			/*
			 * We do not know about this control
			 */
			return -EINVAL;
		}
		else if (
			next_ctrl &&
			index == num_ctrls - 1
		)
		{
			/*
			 * We've got a request for the control
			 * after the last one.
			 */
			return -EINVAL;
		}
	}
	else if (next_ctrl) {
		ctrl_index = 0;
	}
	else {
		return -EINVAL;
	}

	if (
		mcu_ctrl_info[ctrl_index].ctrl_type == CTRL_STANDARD
	) {
		/*
		 * We cannot use `v4l2_ctrl_query_fill` instead of manually filling
		 * the details even for standard controls as we sometimes break the
		 * standard for some controls.
		 *
		 * e.g., V4L2_CID_FOCUS_AUTO has a max value of 1 according to standard
		 * but our version of it has a max value of 5.
		 */
		qc->id = mcu_ctrl_info[ctrl_index].ctrl_id;

		strcpy(qc->name, mcu_ctrl_info[ctrl_index].ctrl_ui_data.ctrl_ui_info.ctrl_name);

		qc->type = mcu_ctrl_info[ctrl_index].ctrl_ui_data.ctrl_ui_info.ctrl_ui_type;
		qc->flags = mcu_ctrl_info[ctrl_index].ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags;

		qc->minimum = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_min;
		qc->maximum = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_max;
		qc->step = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_step;
		qc->default_value = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_def;
	}
	else {
		return -EINVAL;
	}

	return 0;
}

static int ov2311_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = ov2311_data.i2c_client;
	int err = 0;
	uint8_t ctrl_type = 0;
	int ctrl_val = 0;

	if (sd == NULL || ctrl == NULL)
		return -EINVAL;

	if ((err = mcu_get_ctrl(client, ctrl->id, &ctrl_type, &ctrl_val)) < 0) {
		return err;
	}

	if (ctrl_type == CTRL_STANDARD) {
		ctrl->value = ctrl_val;
	} else {
		/* Not Implemented */
		return -EINVAL;
	}

	return err;
}

static int ov2311_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = ov2311_data.i2c_client;
	int err = 0, index, ctrl_index = 0;

	if (sd == NULL || ctrl == NULL)
		return -EINVAL;

	for (index = 0; index < num_ctrls; index++) {
		if (ctrldb[index] == ctrl->id) {
			ctrl_index = index;
			break;
		}
	}

	if (index == num_ctrls) {
		return -EINVAL;
	}

	/*
	 * TODO: the following is redundant as the MCU should already return
	 * a return code indicating the value is out of range.
	 *
	 * We do this as we don't want to call the MCU to validate this as
	 * we already have this information.
	 */
	if (
		ctrl->value < mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_min ||
		ctrl->value > mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_max
	)
		return -ERANGE;

	if ((err =
	     mcu_set_ctrl(client, ctrl->id, CTRL_STANDARD, ctrl->value)) < 0) {
		dev_err(&client->dev," %s (%d ) \n", __func__, __LINE__);
		return -EINVAL;
	}

	return err;
}

static int ov2311_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	int i, err = 0;

	if (sd == NULL || ctrls == NULL)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ext_ctrl = ctrls->controls + i;
		struct v4l2_control ctrl = {
			.id = ext_ctrl->id,
		};

		err = ov2311_g_ctrl(sd, &ctrl);
		if (err) {
			/*
			 * TODO: see TODO in ov2311_s_ext_ctrls
			 * related to setting the error_idx value
			 */
			ctrls->error_idx = ctrls->count;
			break;
		}
		else {
			ext_ctrl->value = ctrl.value;
		}
	}

	return err;
}

static int ov2311_try_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	int i;

	if (sd == NULL || ctrls == NULL)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ext_ctrl = ctrls->controls + i;
		int ctrl_index = 0, index;

		/*
		 * TODO: need to move this into a helper function
		 */
		for (index = 0; index < num_ctrls; index++) {
			if (ctrldb[index] == ext_ctrl->id) {
				ctrl_index = index;
				break;
			}
		}

		if (index == num_ctrls) {
			ctrls->error_idx = ext_ctrl->id;
			return -EINVAL;
		}

		if (
			ext_ctrl->value < mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_min ||
			ext_ctrl->value > mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_max
		) {
			ctrls->error_idx = ext_ctrl->id;
			return -ERANGE;
		}
	}

	return 0;

}

static int ov2311_s_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	int i, err = 0;
	if (sd == NULL || ctrls == NULL)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ext_ctrl = ctrls->controls + i;
		struct v4l2_control ctrl = {
			.id = ext_ctrl->id,
			.value = ext_ctrl->value
		};

		err = ov2311_s_ctrl(sd, &ctrl);
		if (err) {
			/*
			 * TODO: We would have to indicate whether there
			 * is an issue in validation or in the
			 * hardware by correctly setting the error_idx
			 * to count only when the validation failed
			 * and setting it to index when there is an
			 * issue in communication with the hardware.
			 *
			 * For now, just return the count for all cases.
			 */
			ctrls->error_idx = ctrls->count;
			break;
		}
	}

	return err;
}

static int ov2311_enum_frameintervals(struct v4l2_subdev *sd,struct v4l2_subdev_pad_config *cfg,struct v4l2_subdev_frame_interval_enum *fival)
{
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int j;
	int32_t format_fourcc;
	printk("ov2311_enum_frameintervals\n");
	if (fival->width == 0 || fival->height == 0)
	{
		dev_err(&client->dev, "Please assign width and height.\n");
		return -EINVAL;
	}

	switch(fival->code) {
	case MEDIA_BUS_FMT_Y8_1X8:
		format_fourcc = V4L2_PIX_FMT_GREY;
		break;
	case MEDIA_BUS_FMT_Y10_1X10:
		format_fourcc = V4L2_PIX_FMT_Y16;
		break;
	default:
		dev_err(&client->dev, "Please use valid mbus code.\n");
		return -EINVAL;
	}
	/*if (fival->code != ov2311_data.fmt.code)
	{
		return -EINVAL;
	}*/

	for (j=0; j < ov2311_data.num_frm_fmts; j++) {
		if (
			fival->width == ov2311_data.mcu_cam_frmfmt[j].size.width &&
			fival->height == ov2311_data.mcu_cam_frmfmt[j].size.height &&
			format_fourcc == ov2311_data.mcu_cam_frmfmt[j].pixel_format
		) {
			if (fival->index >= ov2311_data.mcu_cam_frmfmt[j].num_framerates)
			{
				pr_err("Func : %s Line : %u\n", __func__, __LINE__);
				return -EINVAL;
			}

			fival->interval.numerator = 1;
			fival->interval.denominator = ov2311_data.mcu_cam_frmfmt[j].framerates[fival->index];

			return 0;
		}
	}
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	return -EINVAL;

}

static int ov2311_enum_framesizes(struct v4l2_subdev *sd,struct v4l2_subdev_pad_config *cfg,struct v4l2_subdev_frame_size_enum *fse)
{
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	int32_t format_fourcc;
	#if 0
	if(fse->code == MEDIA_BUS_FMT_Y8_1X8 && fse->index > 3)
	{
		pr_err("Func : %s Line : %u\n", __func__, __LINE__);
		return -EINVAL;
	}
#endif 
	//else if(fse->code == MEDIA_BUS_FMT_Y10_1X10 && fse->index <=3)
	 
	if(fse->code == MEDIA_BUS_FMT_Y10_1X10)
	{
		fse->index = fse->index + 1;
		pr_err("Func : %s Line : %u\n", __func__, __LINE__);
		return -EINVAL;
	}
	//else if(fse->code == MEDIA_BUS_FMT_Y10_1X10 && fse->index > 3)
	//	return -EINVAL;

		
	if (fse->index >= ov2311_data.num_frm_fmts)
	{
		pr_err("Func : %s Line : %u\n", __func__, __LINE__);
		return -EINVAL;
	}
	pr_err(" fse->index : %d \n\r", fse->index);
	/*if (fse->code != ov2311_data.fmt.code)
	{
		return -EINVAL;
	}*/

	fse->max_width = ov2311_data.mcu_cam_frmfmt[fse->index].size.width;
	fse->min_width = fse->max_width;

	fse->max_height = ov2311_data.mcu_cam_frmfmt[fse->index].size.height;
	fse->min_height = fse->max_height;

	return 0;
}

static int ov2311_enum_mbus_code(struct v4l2_subdev *sd,struct v4l2_subdev_pad_config *cfg,struct v4l2_subdev_mbus_code_enum *code)
{
	pr_err(" code-> index : %d Func : %s Line : %u\n",code->index, __func__, __LINE__);

	if (code->pad || code->index >= ov2311_MAX_FORMAT_SUPPORTED)
	{
		pr_err("###ov2311_enum_mbus_code error \n\r");
		return -EINVAL;
	}

	if(code->index == 0)	
	{
		pr_err("###ov2311_enum_mbus_code : GREY \n\r");
		code->code = MEDIA_BUS_FMT_Y8_1X8;
	}
	else
	{
		pr_err("###ov2311_enum_mbus_code : Y10 \n\r");
		code->code = MEDIA_BUS_FMT_Y10_1X10;
	}

	return 0;
}

static int ov2311_s_stream(struct v4l2_subdev *sd, int enable)
{
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0,
		/*
		 * Have more retries for power wakeup and power down as we
		 * seem to be facing I2C failures despite the retries
		 * which seem to be avoiding those failures in other cases.
		 */
		retries = retries_for_i2c_commands * 2;
	if (!enable) {
		/* Perform Stream Off Sequence - if any */
	}

	err = RETRY_SEQUENCE(
			retries,
			mcu_isp_stream_on(client)
			);
	if (err < 0)
	{
		dev_err(&client->dev, "%s: Failed Stream on\n", __func__);
		return err;
	}

	/* Perform Stream On Sequence - if any  */
	mdelay(10);

	return 0;
}

static int ov2311_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int mode = ov2311_data.streamcap.capturemode;

	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
	param->parm.capture.timeperframe.denominator =
	    ov2311_data.mcu_cam_frmfmt[mode].framerates[ov2311_data.frate_index];
	param->parm.capture.timeperframe.numerator = 1;

	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	return 0;
}

static int ov2311_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0, err = 0;
	int mode = ov2311_data.streamcap.capturemode;
	int fourcc = ov2311_data.pix.pixelformat;

	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	param->parm.capture.timeperframe.denominator = 	ov2311_data.mcu_cam_frmfmt[mode].framerates[ov2311_data.frate_index]; 
	param->parm.capture.timeperframe.numerator = 1;	
	
	memset(param->parm.capture.reserved, 0, 4*sizeof(u32));

	if (
		param->parm.capture.timeperframe.denominator == 0 &&
		param->parm.capture.timeperframe.numerator == 0 &&
		ov2311_data.mcu_cam_frmfmt[mode].num_framerates == 1
	) {
		param->parm.capture.timeperframe.denominator =
			ov2311_data.mcu_cam_frmfmt[mode].framerates[ov2311_data.frate_index];
		param->parm.capture.timeperframe.numerator = 1;
		/*
		 * We would have to reset the frame interval to a
		 * nominal value in this case but as we just have one
		 * frame interval we just return success.
		 */
		return 0;
	}

	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/*
	 * Currently, we haven't handled numerators other than 1.
	 * So, fail in case we get something else.
	 */
	if (param->parm.capture.timeperframe.numerator != 1) {
		dev_err(&client->dev, "Invalid numerator for timeperframe\n");
		return -EINVAL;
	}

	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	for (ret = 0; ret < ov2311_data.mcu_cam_frmfmt[mode].num_framerates;
	     ret++) {
		if ((ov2311_data.mcu_cam_frmfmt[mode].framerates[ret] ==
		     param->parm.capture.timeperframe.denominator)) {
			/*
			 * Have more retries for power wakeup and power down as we
			 * seem to be facing I2C failures despite the retries
			 * which seem to be avoiding those failures in other cases.
			 */
			int retries = retries_for_i2c_commands * 2;
			ov2311_data.frate_index = ret;

			/* call stream config with width, height, frame rate */
			err = RETRY_SEQUENCE(
				retries,
				mcu_stream_config(client, fourcc, mode,	ov2311_data.frate_index)
			);
			if (err < 0) {
				dev_err(&client->dev, "%s: Failed stream_config \n", __func__);
				return err;
			}

			mdelay(10);

			return 0;
		}
	}

	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	return -EINVAL;
}

static int ov2311_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0,
	    /*
	     * Have more retries for power wakeup and power down as we
	     * seem to be facing I2C failures despite the retries
	     * which seem to be avoiding those failures in other cases.
	     */
	retries = retries_for_i2c_commands * 2;
	return 0;
}

static int ov2311_get_fmt(struct v4l2_subdev *sd,struct v4l2_subdev_pad_config *cfg,struct v4l2_subdev_format *format)
{
	int ret = 0;
	
	if (format->pad)
	{
		printk(" /************** ov2311_get_fmt *************/\n");
		return -EINVAL;
	}
	printk(" /************** ov2311_get_fmt *************/\n");
	format->format.code = ov2311_data.fmt.code;
	format->format.colorspace = ov2311_data.fmt.colorspace;
	format->format.field = V4L2_FIELD_NONE;
	format->format.width	= ov2311_data.pix.width;
	format->format.height	= ov2311_data.pix.height; 

	return ret;
}

static int ov2311_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int flag = 0, err = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	/*
	 * Have more retries for power wakeup and power down as we
	 * seem to be facing I2C failures despite the retries
	 * which seem to be avoiding those failures in other cases.
	 */
	int retries = retries_for_i2c_commands * 2;
	if(format->format.code == MEDIA_BUS_FMT_Y10_1X10)
	{
		ov2311_data.fmt.code = MEDIA_BUS_FMT_Y10_1X10;
		ov2311_data.pix.pixelformat = V4L2_PIX_FMT_Y16;
	}
	else
	{
		format->format.code = MEDIA_BUS_FMT_Y8_1X8;
		ov2311_data.pix.pixelformat = V4L2_PIX_FMT_GREY;
	}
	format->format.colorspace = ov2311_data.fmt.colorspace;
	format->format.field = V4L2_FIELD_NONE;

	for (i = 0; i < ov2311_data.num_frm_fmts ; i++) {
		if (
			ov2311_data.mcu_cam_frmfmt[i].size.width == format->format.width &&
			ov2311_data.mcu_cam_frmfmt[i].size.height == format->format.height
		) {
			flag = 1;
			break;
		}
	}

	if(flag == 0) {
		format->format.width	= ov2311_data.pix.width;
		format->format.height	= ov2311_data.pix.height;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		return 0;
	}
printk(" pixel formmat : %#x - %#x - %#x\n", ov2311_data.pix.pixelformat, V4L2_PIX_FMT_GREY, V4L2_PIX_FMT_Y16);
	/* call stream config with width, height, frame rate */
	err = RETRY_SEQUENCE(
		retries,
		mcu_stream_config(
			client,
			ov2311_data.pix.pixelformat,
			ov2311_data.mcu_cam_frmfmt[i].mode,
			ov2311_data.frate_index
		)
	);
	if (err < 0)
	{
		dev_err(&client->dev, "%s: Failed stream_config \n", __func__);
		return err;
	}

	ov2311_data.pix.width = format->format.width;
	ov2311_data.pix.height = format->format.height;
	ov2311_data.streamcap.capturemode = ov2311_data.mcu_cam_frmfmt[i].mode;

	mdelay(10);

	return 0;
}

static struct v4l2_subdev_video_ops ov2311_subdev_video_ops = {
	.g_parm = ov2311_g_parm,
	.s_parm = ov2311_s_parm,
	.s_stream = ov2311_s_stream,
};

static const struct v4l2_subdev_pad_ops ov2311_subdev_pad_ops = {

	.enum_frame_size       = ov2311_enum_framesizes,
	.enum_frame_interval   = ov2311_enum_frameintervals,
	.enum_mbus_code        = ov2311_enum_mbus_code,
	.set_fmt               = ov2311_set_fmt,
	.get_fmt               = ov2311_get_fmt,
};

static struct v4l2_subdev_core_ops ov2311_subdev_core_ops = {
	.s_power	= ov2311_s_power,
	.queryctrl = ov2311_queryctrl,
	.g_ctrl = ov2311_g_ctrl,
	.s_ctrl = ov2311_s_ctrl,
	.g_ext_ctrls = ov2311_g_ext_ctrls,
	.s_ext_ctrls = ov2311_s_ext_ctrls,
	.try_ext_ctrls = ov2311_try_ext_ctrls,
	.querymenu = ov2311_querymenu,
};

static struct v4l2_subdev_ops ov2311_subdev_ops = {
	.core	= &ov2311_subdev_core_ops,
	.video	= &ov2311_subdev_video_ops,
	.pad	= &ov2311_subdev_pad_ops,
};

static int ov2311_init(struct i2c_client *client)
{
	u32 tgt_xclk;	/* target xclk */
	int ret = 0;

	ov2311_data.on = true;

	/* mclk */
	tgt_xclk = ov2311_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)ov2311_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)ov2311_XCLK_MIN);
	ov2311_data.mclk = tgt_xclk;

#ifdef ov2311_DEBUG
	pr_info("mclk: %d MHz\n", tgt_xclk / 1000000);
#endif

	ret = mcu_stream_config(client, ov2311_data.pix.pixelformat,
			ov2311_data.streamcap.capturemode,
			ov2311_data.frate_index);

	return ret;
}

static int ov2311_ctrls_init(ISP_CTRL_INFO *mcu_cam_ctrls)
{
	struct i2c_client *client = NULL;
	int numctrls = 0;
	int err = 0, i = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	client = ov2311_data.i2c_client;

	if (mcu_cam_ctrls == NULL)
	{
		dev_err(
			&client->dev,
			"%s: MCU control data hasn't been allocated\n",
			__func__
		);
		return -EINVAL;
	}

	/* Array of Ctrls */

	/* Custom Ctrl */

	/*
	 * Enumerate the controls from the MCU
	 */
	err = mcu_count_or_list_ctrls(client, mcu_cam_ctrls, &numctrls);
	if (err < 0) {
		dev_err(&client->dev, "Unable to enumerate the controls in the sensor\n");
		return err;
	}

	for (i = 0; i < numctrls; i++) {
		if (mcu_cam_ctrls[i].ctrl_type == CTRL_STANDARD) {

			err = mcu_get_ctrl_ui(client, &mcu_ctrl_info[i], mcu_ctrl_info[i].mcu_ctrl_index);
			if (err != ERRCODE_SUCCESS) {
				dev_err(&client->dev, "Error Enumerating Control 0x%08x !! \n",
					mcu_ctrl_info[i].ctrl_id);
				return err;
			}
			else if (
				mcu_ctrl_info[i].ctrl_ui_data.ctrl_ui_info.ctrl_ui_type ==
				V4L2_CTRL_TYPE_MENU
			) {
				/*
				 * FIXME: This is a workaround to make the step size of
				 * menu controls to have a standard value of 1.
				 *
				 * This is done here rather than directly changing the MCU
				 * as we do not like to change the release a new version
				 * of the MCU code for now.
				 */
				mcu_ctrl_info[i].ctrl_data.std.ctrl_step = 1;
			}
		}
	}

	return 0;
}

static int ov2311_verify_mcu(struct i2c_client *client)
{
	int ret = 0;
	unsigned char fw_version[32] = {0};

	if (client == NULL)
	{
		dev_err(&client->dev, "%s: Invalid I2C client parameter\n", __func__);
		return -EINVAL;
	}

	toggle_gpio(pwdn_gpio, 0);
	msleep(1);
	toggle_gpio(reset_gpio, 0);
	msleep(1);
	toggle_gpio(reset_gpio, 1);
	msleep(100);

#if 1
	ret = mcu_get_fw_version(client, fw_version);

	if (ret == 0)
	{
		ret = mcu_verify_fw_version(fw_version);
	}
	else
	{
		dev_dbg(
			&client->dev,
			"Could not read the firmware version from the MCU\n"
		);
	}

	/*
	 * Try booting and flashing in bootloader mode when an error is detected
	 * or the force update bit is set in the firmware version
	 */
	if (ret != 0) {
		int loop = 0;

		/*
		 * Verification of the MCU in firmware mode failed so
		 * try to boot the MCU in bootloader mode.
		 */

#ifdef ov2311_DEBUG
		pr_info(" Trying to Detect Bootloader mode\n");
#endif

		toggle_gpio(reset_gpio, 0);
		msleep(1);
		toggle_gpio(pwdn_gpio, 1);
		msleep(1);
		toggle_gpio(reset_gpio, 1);
		msleep(1);

		for(loop = 0; loop < 10; loop++) {
			ret = mcu_bload_get_version(client);
			if (ret < 0) {
				msleep(1);
				continue;
			} else {
#ifdef ov2311_DEBUG
				pr_info(" Get Bload Version Success\n");
#endif
				break;
			}
		}

		if(loop == 10) {
			dev_err(&client->dev, "Error getting firmware version in bootloader mode\n");
			return -EFAULT;
		}

		pr_info("Updating firmware. Please wait ...\n");

		if (mcu_fw_update(client, NULL) < 0) {
			dev_err(&client->dev, "Error when trying to update the firmware\n");
			return -EFAULT;
		}

		toggle_gpio(pwdn_gpio, 0);

		/* Allow FW Updated MCU to reboot */
		msleep(10);

		/*
		 * Ensure the firmware has been flashed correctly by getting the version
		 * of the firmware (in firmware mode).
		 *
		 * TODO:
		 * - Ensure the above description is correct
		 * - Is this required? Shouldn't the firmware be updated correctly
		 *   when the `mcu_fw_update` function returns successfully?
		 */
		for(loop = 0; loop < 100; loop++) {
			ret = mcu_get_fw_version(client, fw_version);

			if (ret == 0)
			{
				ret = mcu_verify_fw_version(fw_version);
			}

			if (ret < 0) {
				msleep(1);
				continue;
			} else {
#ifdef ov2311_DEBUG
				pr_info(" Get FW Version Success\n");
#endif
				break;
			}
		}

		if(loop == 100) {
			dev_err(
				&client->dev,
				"Couldn't get firmware version correctly after update (did the update fail?)\n"
			);
			return ret;
		}
		else {
			pr_info("Firmware has been updated successfully.\n");
		}
	}
#endif 
//	dev_info(&client->dev, "Current Firmware Version - (%.32s)\n", fw_version);

	return ret;
}

static int ov2311_parse_and_get_clocks(struct device *dev)
{
	int retval = 0;

	if (dev == NULL)
	{
		dev_err(dev, "%s: Invalid device parameter\n", __func__);
		return -EINVAL;
	}

	ov2311_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov2311_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ov2311_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ov2311_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ov2311_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ov2311_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}
	
	retval = of_property_read_u32(dev->of_node, "mipi-data-lanes", (u32 *) &(ov2311_data.mipi_lane_config));
	if (retval) {
		dev_err(dev, " mipi lane missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "camera-mipi-clk", (u32 *) &(ov2311_data.mipi_clk_config));
	if (retval) {
		dev_err(dev, "camera mipi clk is missing or invalid\n");
		return retval;
	}
	else
		pr_info("camera-mipi-clk = %dMHz\n",ov2311_data.mipi_clk_config);

	/*
	 * Though we parse csi_id, we do not seem to be using it as there
	 * doesn't seem to be a way to specify the virutal channel to the
	 * MCU.
	 */
	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ov2311_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}
	

	return 0;
}

static int ov2311_parse_and_get_gpios(struct device *dev)
{
	int err;
	struct device_node *node = NULL;

	if (dev == NULL)
	{
		dev_err(dev, "%s: Invalid device parameter\n", __func__);
		return -EINVAL;
	}

	/*
	 * Assume node would be good if the given 'dev' parameter
	 * is good.
	 */
	node = dev->of_node;

	pwdn_gpio = of_get_named_gpio(node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwdn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available");
		return -EINVAL;
	}
	else {
#ifdef ov2311_DEBUG
		printk("BOOT = %x \n", pwdn_gpio);
#endif
	}

	reset_gpio = of_get_named_gpio(node, "rst-gpios", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(dev, "no sensor reset pin available");
		return -EINVAL;
	}
	else {
#ifdef ov2311_DEBUG
		printk("RESET = %x \n", reset_gpio);
#endif
	}

	/*
	 * The ov2311_mipi_pwdn & ov2311_mipi_reset seem to be just new
	 * identifiers. So any name shouldn't be a problem.
	 */
	err = devm_gpio_request_one(dev, pwdn_gpio, GPIOF_OUT_INIT_HIGH,
					"ov2311_mipi_pwdn");
	if (err < 0) {
		dev_warn(dev, "Failed to set power pin\n");
		dev_warn(dev, "err = %d\n", err);
		return err;
	}

	err = devm_gpio_request_one(dev, reset_gpio, GPIOF_OUT_INIT_HIGH,
					"ov2311_mipi_reset");
	if (err < 0) {
		dev_warn(dev, "Failed to set reset pin\n");
		dev_warn(dev, "err = %d\n", err);
		return err;
	}

	return 0;
}

/*!
 * ov2311 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov2311_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	printk(" ov2311_probe \n");
	struct pinctrl *pinctrl;
	struct device_node *node = client->dev.of_node;
	struct device *dev = &client->dev;

	int ret, frm_fmt_size = 0, i;
	uint16_t sensor_id = 0;
	pr_err("Func : %s Line : %u\n", __func__, __LINE__);
	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "no pin available\n");

	/* Set initial values for the sensor struct. */
	memset(&ov2311_data, 0, sizeof(ov2311_data));

	ret = ov2311_parse_and_get_gpios(dev);
	if (ret)
	{
		pr_info("Warning: couldn't get GPIOs\n");
	}

	ret = ov2311_parse_and_get_clocks(dev);
	if (ret)
	{
		dev_err(dev, "Error occurred when getting clock\n");
		return ret;
	}

	clk_prepare_enable(ov2311_data.sensor_clk);

	/*
	 * We usually get and set/enable regulators here. But it doesn't
	 * seem to be needed here as the Variscite EVK seems to be supplying
	 * the required voltage directly without us needing to set it.
	 */
#if 1
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		ov2311_verify_mcu(client)
	);
	if (ret < 0)
	{
		dev_err(dev, "Error occurred when verifying MCU\n");
		return ret;
	}
#endif 	
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_isp_configuration(CMD_ID_LANE_CONFIG,client)
	);
	if (ret < 0) 
	{
		dev_err(dev, "Error occured in configuring mipi lanes\n");
		return ret;
	}

	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_isp_configuration(CMD_ID_MIPI_CLK_CONFIG,client)
	);

	if (ret < 0)
	{
		dev_err(dev, "Error occured in configuring mipi clock\n");
		return ret;
	}

	/*
	 * Query the number of controls from MCU
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_count_or_list_ctrls(client, NULL, &num_ctrls)
	);
	if (ret < 0)
	{
		dev_err(dev, "%s, Failed to get number of controls for sensor\n", __func__);
		return ret;
	}

	/*
	 * Query the number for Formats available from MCU
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_count_or_list_fmts(client, NULL, &frm_fmt_size)
	);
	if (ret < 0)
	{
		dev_err(dev, "%s, Failed to get number of formats for sensor\n", __func__);
		return ret;
	}

	/*
	 * Initialise the MCU related data as we're about to use them.
	 */
	ret = mcu_data_init(dev, frm_fmt_size);
	if (ret < 0)
	{
		dev_err(dev, "%s: failed to initialize MCU related data\n", __func__);
		return ret;
	}

	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_get_sensor_id(client, &sensor_id)
	);
	if (ret < 0)
	{
		dev_err(dev, "Unable to get MCU Sensor ID \n");
		return ret;
	}

	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_isp_init(client)
	);
	if (ret < 0)
	{
		dev_err(dev, "Unable to INIT ISP \n");
		return ret;
	}

	/*
	 * Enumerate the Formats in the sensor
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_count_or_list_fmts(client, stream_info, &frm_fmt_size)
	);
	if (ret < 0)
	{
		dev_err(dev, "Unable to enumerate the formats in the sensor\n");
		return ret;
	}

	/*
	 * Fill some state information as required.
	 */
	ov2311_data.i2c_client = client;

	ov2311_data.pix.pixelformat = V4L2_PIX_FMT_GREY;
	//ov2311_data.pix.pixelformat = V4L2_PIX_FMT_Y16;
	ov2311_data.fmt.code = ov2311_DEFAULT_DATAFMT;
	ov2311_data.fmt.colorspace = ov2311_DEFAULT_COLORSPACE;
	ov2311_data.pix.width = ov2311_DEFAULT_WIDTH;
	ov2311_data.pix.height = ov2311_DEFAULT_HEIGHT;
	ov2311_data.streamcap.capability =  V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	ov2311_data.streamcap.capturemode = ov2311_DEFAULT_MODE;
	ov2311_data.streamcap.timeperframe.denominator = ov2311_DEFAULT_FPS;
	ov2311_data.streamcap.timeperframe.numerator = 1;
	ov2311_data.num_frm_fmts = frm_fmt_size;
	ov2311_data.power_on = 0;

	/*
	 * Configure the stream with default configuration
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		ov2311_init(client)
	);
	if (ret < 0)
	{
		dev_err(dev, "Failed to initialise the device with default configuration\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&ov2311_data.subdev, client, &ov2311_subdev_ops);
	printk(" V4l2 i2c subdev ret value : %d\n ", ret);
	/*
	 * Initialize Controls by getting details about the controls from the MCU
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		ov2311_ctrls_init(mcu_ctrl_info)
	);
	if (ret < 0)
	{
		dev_warn(dev, "Failed to initialise the controls. Controls might not work\n");
	}

	/*
	 * Write default values for all controls
	 */
	for (i = 0; i < num_ctrls; i++) {
		if (mcu_ctrl_info[i].ctrl_type == CTRL_STANDARD) {
			int ret;
			struct v4l2_control ctrl = {
				.id = mcu_ctrl_info[i].ctrl_id,
				.value = mcu_ctrl_info[i].ctrl_data.std.ctrl_def
			};

			if (
				mcu_ctrl_info[i].ctrl_id == 0x9a0926 
			)
			{
				/*
				 * We know that the MCU would fail when we
				 * try to write the V4L2_CID_ROI_EXPOSURE
				 * control as we are not in the correct auto
				 * exposure mode by default. So, skip it.
				 */
				continue;
			}

			ret = RETRY_SEQUENCE(
				retries_for_i2c_commands,
				ov2311_s_ctrl(&ov2311_data.subdev, &ctrl)
			);
			if (ret < 0)
			{
				dev_warn(dev, "Failed to write default value for a control: %d; Control ID: %x\n", i, mcu_ctrl_info[i].ctrl_id);
			}
		}
	}

	ret = v4l2_async_register_subdev(&ov2311_data.subdev);
	if (ret)
	{
		dev_err(dev, "Failed to register the I2C subdev for the sensor\n");
		return ret;
	}

	pr_info("ov2311 V%s detected.\n", VERSION);

	return 0;
}

/*!
 * ov2311 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov2311_remove(struct i2c_client *client)
{
//	int err;
	v4l2_async_unregister_subdev(&ov2311_data.subdev);

	clk_disable_unprepare(ov2311_data.sensor_clk);

	/*
	 * Power down the MCU
	 */
	if (reset_gpio >= 0)
	{
		toggle_gpio(reset_gpio, 0);
	}

	/*
	 * Free up the GPIOs
	 */
	if (pwdn_gpio >= 0)
		devm_gpio_free(&client->dev, pwdn_gpio);

	if (reset_gpio >= 0)
		devm_gpio_free(&client->dev, reset_gpio);

	return 0;
}

static const struct i2c_device_id ov2311_id[] = {
	{"ov2311", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov2311_id);

static struct i2c_driver ov2311_i2c_driver = {
	.driver = {
		   .name = "ov2311",
		   .owner = THIS_MODULE
	},
	.probe = ov2311_probe,
	.remove = ov2311_remove,
	.id_table = ov2311_id,
};


module_i2c_driver(ov2311_i2c_driver);

MODULE_DESCRIPTION("ov2311 V4L2 driver");
MODULE_AUTHOR("e-con Systems");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(VERSION);
MODULE_ALIAS("CSI");
