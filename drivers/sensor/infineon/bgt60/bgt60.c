/*
 * Copyright (c) 2025 Adam Labbe <adam@hfi.io>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_bgt60

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>

#include <zephyr/drivers/sensor/bgt60.h>

//LOG_MODULE_REGISTER(BGT60, CONFIG_SENSOR_LOG_LEVEL);
LOG_MODULE_REGISTER(BGT60, LOG_LEVEL_DBG);

/* Register Definitions */
#define RADAR_REG_MAIN		0x0000
#define RADAR_REG_ADC0		0x0001
#define RADAR_REG_STAT1		0x0003
#define RADAR_REG_PACR1		0x0004
#define RADAR_REG_PACR2		0x0005
#define RADAR_REG_SFCTL		0x0006
#define RADAR_REG_CSCI		0x000B
#define RADAR_REG_CSCDS		0x000F
#define RADAR_REG_CS1_U_0	0x0010
#define RADAR_REG_CS1_U_1	0x0011 // MADC_BBCH1_EN is bit 20 here
#define RADAR_REG_CS1_U_2	0x0012
#define RADAR_REG_CS1		0x0016
#define RADAR_REG_CS2		0x001D
#define RADAR_REG_CS3		0x0024
#define RADAR_REG_CS4		0x002B
#define RADAR_REG_CCR0		0x002C
#define RADAR_REG_CCR1		0x002D
#define RADAR_REG_CCR2		0x002E
#define RADAR_REG_CCR3		0x002F
#define RADAR_REG_PLL1_0	0x0030
#define RADAR_REG_PLL1_1	0x0031
#define RADAR_REG_PLL1_2	0x0032
#define RADAR_REG_PLL1_3	0x0033
#define RADAR_REG_PLL1_7	0x0037
#define RADAR_REG_PLL2_7	0x003F
#define RADAR_REG_PLL3_7	0x0047
#define RADAR_REG_PLL4_7	0x004F
#define RADAR_REG_ADC1		0x0050 // Present in MP code comments
#define RADAR_REG_FD		0x005F
#define RADAR_REG_WU		0x0060
#define RADAR_REG_STAT0		0x0061
#define RADAR_REG_FSTAT		0x0063

/* MAIN register bits */
#define RADAR_MAIN_FRAME_START_BIT	(1U << 0)

/* PACR1 register bits */
#define RADAR_PACR1_OSCCLKEN_BIT	(1U << 23)

/* CS1_U_1 (0x011) register bits */
#define RADAR_CS1_U_1_MADC_BBCH1_EN_BIT (1U << 20)

/* STAT0 register bits */
#define RADAR_STAT0_ADC_RDY_BIT        (1U << 1)
#define RADAR_STAT0_ADC_BGUP_BIT       (1U << 2)
#define RADAR_STAT0_LDO_RDY_BIT        (1U << 3)

#define RADAR_STAT0_POWER_MODE_POS     5
#define RADAR_STAT0_POWER_MODE_MASK    (0x7U << RADAR_STAT0_POWER_MODE_POS)

#define RADAR_STAT0_CH_IDX_POS         8
#define RADAR_STAT0_CH_IDX_MASK        (0x7U << RADAR_STAT0_CH_IDX_POS)

#define RADAR_STAT0_SH_IDX_POS         11
#define RADAR_STAT0_SH_IDX_MASK        (0x7U << RADAR_STAT0_SH_IDX_POS)

/* FSTAT register bits */
#define RADAR_FSTAT_FIFO_FILL_MASK		0x3FFF
#define RADAR_FSTAT_FILL_MAX_VALUE		0x0800U  // 2048 -> 100%
#define RADAR_FSTAT_RAM_PWR_DOWN_BIT	(1U << 14)
#define RADAR_FSTAT_CLK_NUM_ERR_BIT		(1U << 17)
#define RADAR_FSTAT_BURST_ERR_BIT		(1U << 18)
#define RADAR_FSTAT_FUF_ERR_BIT			(1U << 19)
#define RADAR_FSTAT_FIFO_EMPTY_BIT		(1U << 20)
#define RADAR_FSTAT_CREF_BIT			(1U << 21)
#define RADAR_FSTAT_FIFO_FULL_BIT		(1U << 22)
#define RADAR_FSTAT_FIFO_OVF_BIT		(1U << 23)

/* Global status register bits */
#define RADAR_GSR0_CLK_NUMBER_ERROR_BIT	(1U << 0)
#define RADAR_GSR0_SPI_BURST_ERROR_BIT	(1U << 1)
#define RADAR_GSR0_MISO_HS_READ_BIT		(1U << 2)
#define RADAR_GSR0_FOU_ERROR_BIT		(1U << 3)

/* FIFO and Sample characteristics */
#define RADAR_MAX_FIFO_BLOCKS		CONFIG_BGT60_RADAR_SENSOR_DEFAULT_FIFO_BLOCKS
#define RADAR_BYTES_PER_BLOCK		3	// Each block from FIFO is 3 bytes
#define RADAR_SAMPLES_PER_BLOCK		2	// Each 3-byte block yields two 12-bit samples
#define RADAR_FIFO_POLL_TIMEOUT_MS	CONFIG_BGT60_RADAR_FIFO_POLL_TIMEOUT_MS
#define RADAR_FIFO_POLL_INTERVAL_MS	CONFIG_BGT60_RADAR_FIFO_POLL_INTERVAL_MS

struct bgt60_config {
	struct spi_dt_spec bus;
};

struct bgt60_data {
	const struct device *dev;
	uint16_t num_fifo_blocks_to_read; // Number of 3-byte blocks to read, configurable
};

typedef enum resetable_subsystem {
	RESET_SOFTWARE = 1,
	RESET_FSM = 2,
	RESET_FIFO = 3
} resetable_subsystem;

static int initialize_registers(const struct device *dev);
static int reset_subsystem(const struct device *dev, resetable_subsystem subsystem);

/**
 * @brief Write a 24-bit value to a sensor register.
 *
 * @param dev Pointer to the sensor device structure.
 * @param reg_addr_7bit The 7-bit register address.
 * @param value_24bit The 24-bit value to write.
 * @return 0 on success, negative error code otherwise.
 */
static int write_register(const struct device *dev, uint8_t reg_addr_7bit, uint32_t value_24bit)
{
	const struct bgt60_config *config = dev->config;
	uint8_t cmd_byte = (reg_addr_7bit << 1) | 0x01; // Set R/W bit to 1 for write
	uint8_t tx_data[3];

	// Pack 24-bit value as big-endian
	tx_data[0] = (value_24bit >> 16) & 0xFF;
	tx_data[1] = (value_24bit >> 8) & 0xFF;
	tx_data[2] = value_24bit & 0xFF;

	const struct spi_buf tx_bufs[] = {
		{ .buf = &cmd_byte, .len = 1 },
		{ .buf = tx_data,   .len = sizeof(tx_data) }
	};
	const struct spi_buf_set tx = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };

	return spi_write_dt(&config->bus, &tx);
}

/**
 * @brief Read a 24-bit value from a sensor register.
 *
 * @param dev Pointer to the sensor device structure.
 * @param reg_addr_7bit The 7-bit register address.
 * @param value_24bit Pointer to store the read 24-bit value.
 * @param gsr0_status Pointer to store the Global Status Register (GSR0) byte. Can be NULL.
 * @return 0 on success, negative error code otherwise.
 */
static int read_register(const struct device *dev, uint8_t reg_addr_7bit, uint32_t *value_24bit)
{
	const struct bgt60_config *config = dev->config;
	uint8_t cmd_byte = (reg_addr_7bit << 1) & ~0x01U; // Clear R/W bit for read
	uint8_t dummy_tx_payload[3] = {0, 0, 0}; // Dummy bytes for data part of read
	uint8_t rx_payload[4]; // 1 byte GSR0 + 3 bytes data

	const struct spi_buf tx_bufs[] = {
		{ .buf = &cmd_byte,		   .len = 1 },
		{ .buf = dummy_tx_payload,	.len = sizeof(dummy_tx_payload) }
	};
	const struct spi_buf_set tx = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };

	struct spi_buf rx_bufs[] = {
		// Single RX buffer for the entire transaction (CMD echo + data)
		// The first byte will be GSR0, next 3 are data.
		{ .buf = rx_payload, .len = sizeof(rx_payload) }
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	const int ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret < 0) {
		LOG_DBG("SPI transceive failed for reg 0x%02X: %d", reg_addr_7bit, ret);
		return ret;
	}

	const uint8_t gsr0_status = rx_payload[0];
	const bool clock_number_error = gsr0_status & RADAR_GSR0_CLK_NUMBER_ERROR_BIT;
	const bool spi_burst_error = gsr0_status & RADAR_GSR0_SPI_BURST_ERROR_BIT;
	const bool fifo_over_under_flow_error = gsr0_status & RADAR_GSR0_FOU_ERROR_BIT;

	if (clock_number_error) {
		LOG_DBG("GSR0 clock number error");
	}
	if (spi_burst_error) {
		LOG_DBG("GSR0 spi burst error");
	}
	if (fifo_over_under_flow_error) {
		LOG_DBG("GSR0 fifo over/under-flow");
	}

	// Unpack 24-bit value from big-endian format (rx_payload[1], rx_payload[2], rx_payload[3])
	*value_24bit = ((uint32_t)rx_payload[1] << 16) |
				   ((uint32_t)rx_payload[2] << 8)  |
				   ((uint32_t)rx_payload[3]);

	return ret;
}

/**
 * @brief Perform a burst read from the sensor's FIFO.
 *
 * @param dev Pointer to the sensor device structure.
 * @param buffer Pointer to the buffer to store raw FIFO data.
 * @param num_bytes_to_read Number of bytes to read from FIFO. Must be multiple of RADAR_BYTES_PER_BLOCK.
 * @return 0 on success, negative error code otherwise.
 */
static int radar_fifo_burst_read(const struct device *dev, uint8_t *buffer, size_t num_bytes_to_read)
{
	const struct bgt60_config *config = dev->config;

	// Construct the burst read command:
	// - ADDR (31:25): 0x7F for burst mode
	// - RW (24): 1 for write mode (set bit 24 to 1)
	// - SADDR (23:17): 0x64 for FIFO access
	// - RWB (16): 0 for read burst
	// - NBURSTS (15:9): 0 for unbounded burst access
	// - RSVD (8:0): Reserved, set to 0
	uint32_t cmd_fifo_burst = sys_cpu_to_be32(
			(0x7F << 25) | (1 << 24) | (0x64 << 17) | (0 << 16) | (0 << 9));

	// This SPI transaction sends a 4-byte command, then clocks in num_bytes_to_read.
	const struct spi_buf tx_bufs[] = {
		{ .buf = &cmd_fifo_burst, .len = sizeof(cmd_fifo_burst) }, // Send the 32-bit burst command
		{ .buf = NULL,	  .len = num_bytes_to_read } // Clock data in from sensor
	};
	const struct spi_buf_set tx = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };

	struct spi_buf rx_bufs[] = {
		{ .buf = NULL,   .len = sizeof(cmd_fifo_burst) },   // Discard data received during command
		{ .buf = buffer, .len = num_bytes_to_read }   // Store actual FIFO data
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	return spi_transceive_dt(&config->bus, &tx, &rx);
}

static int print_status(const struct device *dev)
{
	LOG_DBG("STAT0 status");
	uint32_t STAT0;
	int ret = read_register(dev, (RADAR_REG_STAT0 & 0x7F), &STAT0);
	if (ret < 0) {
		return ret;
	}
	LOG_DBG("RADAR_STAT0_ADC_RDY:  %s", (STAT0 & RADAR_STAT0_ADC_RDY_BIT)  ? "Ready" : "Not ready");
	LOG_DBG("RADAR_STAT0_ADC_BGUP: %s", (STAT0 & RADAR_STAT0_ADC_BGUP_BIT) ? "Ready" : "Not ready");
	LOG_DBG("RADAR_STAT0_LDO_RDY:  %s", (STAT0 & RADAR_STAT0_LDO_RDY_BIT)  ? "Ready" : "Not ready");

	// Multi-bit fields
	uint8_t pm = (STAT0 & RADAR_STAT0_POWER_MODE_MASK) >> RADAR_STAT0_POWER_MODE_POS;
	LOG_DBG("RADAR_STAT0_POWER_MODE:");
	switch (pm) {
		case 0: LOG_DBG("\tDeep sleep (after reset)"); break;
		case 1: LOG_DBG("\tActive mode"); break;
		case 2: LOG_DBG("\tInterchirp mode"); break;
		case 3: LOG_DBG("\tIdle mode"); break;
		case 5: LOG_DBG("\tDeep sleep"); break;
		default: LOG_DBG("\tReserved"); break;
	}

	uint8_t ch_idx = (STAT0 & RADAR_STAT0_CH_IDX_MASK) >> RADAR_STAT0_CH_IDX_POS;
	LOG_DBG("RADAR_STAT0_CH_IDX:");
	switch (ch_idx) {
		case 0: LOG_DBG("\tCSU1"); break;
		case 1: LOG_DBG("\tCSD1"); break;
		case 2: LOG_DBG("\tCSU2"); break;
		case 3: LOG_DBG("\tCSD2"); break;
		case 4: LOG_DBG("\tCSU3"); break;
		case 5: LOG_DBG("\tCSD3"); break;
		case 6: LOG_DBG("\tCSU4"); break;
		case 7: LOG_DBG("\tCSD4"); break;
	}

	uint8_t sh_idx = (STAT0 & RADAR_STAT0_SH_IDX_MASK) >> RADAR_STAT0_SH_IDX_POS;
	LOG_DBG("RADAR_STAT0_SH_IDX:");
	switch (sh_idx) {
		case 0: LOG_DBG("\tPLLU1"); break;
		case 1: LOG_DBG("\tPLLD1"); break;
		case 2: LOG_DBG("\tPLLU2"); break;
		case 3: LOG_DBG("\tPLLD2"); break;
		case 4: LOG_DBG("\tPLLU3"); break;
		case 5: LOG_DBG("\tPLLD3"); break;
		case 6: LOG_DBG("\tPLLU4"); break;
		case 7: LOG_DBG("\tPLLD");  break;
	}

	LOG_DBG("\nFIFO status:");
	uint32_t FSTAT;
	ret = read_register(dev, RADAR_REG_FSTAT, &FSTAT);
	if (ret < 0) {
		return ret;
	}

	const uint16_t fill_status = FSTAT & RADAR_FSTAT_FIFO_FILL_MASK;
    const uint8_t fill_percent = (fill_status * 100) / RADAR_FSTAT_FILL_MAX_VALUE;
    LOG_DBG("FSTAT_FILL_STATUS: 0x%04X (%u%% full)", fill_status, fill_percent);
	LOG_DBG("FSTAT_RAM_PWR_DOWN: %s",
			(FSTAT & RADAR_FSTAT_RAM_PWR_DOWN_BIT) ? "Powered down" : "Powered up");
	LOG_DBG("FSTAT_CLK_NUM_ERR:  %s",
			(FSTAT & RADAR_FSTAT_CLK_NUM_ERR_BIT) ? "Clock number error" : "No error");
	LOG_DBG("FSTAT_BURST_ERR:    %s",
			(FSTAT & RADAR_FSTAT_BURST_ERR_BIT) ? "Burst error" : "No error");
	LOG_DBG("FSTAT_FUF_ERR:      %s",
			(FSTAT & RADAR_FSTAT_FUF_ERR_BIT) ? "FIFO underflow" : "No underflow");
	LOG_DBG("FSTAT_EMPTY:        %s",
			(FSTAT & RADAR_FSTAT_FIFO_EMPTY_BIT) ? "FIFO empty" : "FIFO not empty");
	LOG_DBG("FSTAT_CREF:         %s",
			(FSTAT & RADAR_FSTAT_CREF_BIT) ? "Above reference" : "Below reference");
	LOG_DBG("FSTAT_FULL:         %s",
			(FSTAT & RADAR_FSTAT_FIFO_FULL_BIT) ? "FIFO full" : "FIFO not full");
	LOG_DBG("FSTAT_FOF_ERR:      %s",
			(FSTAT & RADAR_FSTAT_FIFO_OVF_BIT) ? "FIFO overflow" : "No overflow");

	return 0;
}

/**
 * @brief Submit a read request to the sensor.
 * This function is called by the RTIO framework to acquire raw sensor data.
 */
static void bgt60_submit(const struct device *dev, struct rtio_iodev_sqe *sqe)
{
	struct bgt60_data *data = dev->data;

	// Determine the expected number of raw bytes based on configured blocks
	uint32_t expected_raw_bytes = data->num_fifo_blocks_to_read * RADAR_BYTES_PER_BLOCK;

	// Get the buffer for storing raw data from the submission queue entry
	uint8_t *raw_data_buffer;
	uint32_t raw_buffer_alloc_len; // Actual allocated length by RTIO
	int ret = rtio_sqe_rx_buf(sqe, expected_raw_bytes, expected_raw_bytes,
			&raw_data_buffer, &raw_buffer_alloc_len);
	if (ret != 0) {
		LOG_ERR("Failed to get RX buffer from RTIO SQE: %d", ret);
		rtio_iodev_sqe_err(sqe, ret);
		return;
	}

	// Ensure the allocated buffer is sufficient and a multiple of block size
	if (raw_buffer_alloc_len < expected_raw_bytes || raw_buffer_alloc_len % RADAR_BYTES_PER_BLOCK != 0) {
		LOG_ERR("RTIO buffer length invalid: alloc %u, expected %u, block_size %d",
				raw_buffer_alloc_len, expected_raw_bytes, RADAR_BYTES_PER_BLOCK);
		rtio_iodev_sqe_err(sqe, -EINVAL);
		return;
	}
	// The actual number of bytes to read will be raw_buffer_alloc_len

	LOG_DBG("Attempting to read %u bytes from FIFO into RTIO buffer", raw_buffer_alloc_len);

	// --- Start of Measurement Sequence
	// 1. Reset FIFO
	ret = reset_subsystem(dev, RESET_FIFO);
	if (ret < 0) {
		LOG_ERR("FIFO reset failed: %d", ret);
		goto complete_with_error;
	}

	// 2. Start Frame
	uint32_t temp_val;
	ret = read_register(dev, RADAR_REG_MAIN, &temp_val);
	if (ret < 0) {
		LOG_ERR("Failed to read MAIN for frame start: %d", ret);
		goto complete_with_error;
	}
	temp_val |= RADAR_MAIN_FRAME_START_BIT;
	ret = write_register(dev, RADAR_REG_MAIN, temp_val);
	if (ret < 0) {
		LOG_ERR("Failed to write MAIN for frame start: %d", ret);
		goto complete_with_error;
	}
	LOG_DBG("Frame started. MAIN: 0x%06X", temp_val);

	// 3. Poll for data ready (FIFO not empty)
	bool data_is_ready = false;
	uint32_t fstat_val;
	int poll_attempts = RADAR_FIFO_POLL_TIMEOUT_MS / RADAR_FIFO_POLL_INTERVAL_MS;

	for (int i = 0; i < poll_attempts; i++) {
		ret = read_register(dev, RADAR_REG_FSTAT, &fstat_val);
		if (ret < 0) {
			LOG_WRN("Failed to read FSTAT (poll %d): %d", i, ret);
			k_sleep(K_MSEC(RADAR_FIFO_POLL_INTERVAL_MS)); // Still wait before retrying or failing
			continue; // Try reading FSTAT again after delay
		}
		const bool fifo_empty = fstat_val & RADAR_FSTAT_FIFO_EMPTY_BIT;
		const bool fifo_full = fstat_val & RADAR_FSTAT_FIFO_FULL_BIT;
		if (fifo_full) {
			data_is_ready = true;
			LOG_DBG("FIFO data ready after %dms. FSTAT: 0x%06X", (i + 1) * RADAR_FIFO_POLL_INTERVAL_MS, fstat_val);
			print_status(dev);
			break;
		}
		k_sleep(K_MSEC(RADAR_FIFO_POLL_INTERVAL_MS));
	}

	if (!data_is_ready) {
		LOG_WRN("Timeout waiting for FIFO data after %dms. Last FSTAT: 0x%06X", RADAR_FIFO_POLL_TIMEOUT_MS, fstat_val);
		ret = -ETIMEDOUT;
		goto complete_with_error;
	}

	// 4. Perform FIFO Burst Read into the RTIO-provided raw buffer
	ret = radar_fifo_burst_read(dev, raw_data_buffer, raw_buffer_alloc_len);
	if (ret < 0) {
		LOG_ERR("FIFO burst read failed: %d", ret);
	}

complete_with_error:
	print_status(dev);
	if (ret < 0) {
		rtio_iodev_sqe_err(sqe, ret); // Report error to RTIO
	} else {
		// On success, result for rtio_iodev_sqe_ok is typically 0 for sensor_read.
		// The actual amount of data is known by RTIO from raw_buffer_alloc_len.
		rtio_iodev_sqe_ok(sqe, 0);
		LOG_DBG("FIFO read successful, %u raw bytes obtained.", raw_buffer_alloc_len);
	}
}

static DEVICE_API(sensor, bgt60_api_funcs) = {
	.submit = bgt60_submit, /* no decode since user can FFT on chirp units */
};

/**
 * @brief Performs hardware initialization steps.
 */
static int initialize_sensor(const struct device *dev)
{
	int ret;
	uint32_t reg_val;

	// 1. Populate registers with predefined values
	ret = initialize_registers(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize sensor registers: %d", ret);
		return ret;
	}

	// 2. Enable Oscillator Clock (OSCCLKEN in PACR1, bit 23)
	ret = read_register(dev, (RADAR_REG_PACR1 & 0x7F), &reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to read PACR1: %d", ret);
		return ret;
	}
	reg_val |= RADAR_PACR1_OSCCLKEN_BIT;
	ret = write_register(dev, (RADAR_REG_PACR1 & 0x7F), reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write PACR1 to enable oscillator: %d", ret);
		return ret;
	}
	LOG_INF("Oscillator enabled. PACR1: 0x%06X", reg_val);

	// 3. Enable MADC_BBCH1_EN (bit 20 in CS1_U_1 (0x011))
	ret = read_register(dev, (RADAR_REG_CS1_U_1 & 0x7F), &reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to read CS1_U_1: %d", ret);
		return ret;
	}
	reg_val |= RADAR_CS1_U_1_MADC_BBCH1_EN_BIT;
	ret = write_register(dev, (RADAR_REG_CS1_U_1 & 0x7F), reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to write CS1_U_1 to enable MADC_BBCH1: %d", ret);
		return ret;
	}
	LOG_INF("MADC_BBCH1 enabled. CS1_U_1: 0x%06X", reg_val);

	// 4. Reset FIFO to ensure it starts clean
	ret = reset_subsystem(dev, RESET_FIFO);
	if (ret < 0) {
		LOG_DBG("Failed to reset FIFO: %d", ret);
		return ret;
	}
	LOG_INF("FIFO reset.");

	// Optional: Check LDO/ADC ready status from STAT0 (as in MicroPython `check_ldo_adc_ready`)
	// This is a good health check after initialization.
	k_sleep(K_MSEC(5000)); // Small delay for statuses to update after config
	ret = read_register(dev, (RADAR_REG_STAT0 & 0x7F), &reg_val);
	if (ret == 0) {
		LOG_INF("STAT0: 0x%06X", reg_val);
		LOG_INF("LDO %s ready.", (reg_val & RADAR_STAT0_LDO_RDY_BIT) ? "is" : "not");
		LOG_INF("ADC %s ready.", (reg_val & RADAR_STAT0_ADC_RDY_BIT) ? "is" : "not");
	} else {
		LOG_ERR("Failed to read STAT0 for status check: %d", ret);
	}

	LOG_INF("Radar sensor hardware initialized successfully.");
	return 0;
}

/**
 * @brief Initializes sensor registers with predefined values.
 * Similar to `populate_registers` in the MicroPython driver.
 */
static int initialize_registers(const struct device *dev) {
	// Table of register addresses and their initial values
	// Ensure register addresses are 7-bit for the SPI helper functions
	const struct { uint8_t reg_7bit; uint32_t val_24bit; const char *name; } reg_init_table[] = {
		{ (RADAR_REG_ADC0    & 0x7F), 0x050010, "ADC0"    },
		{ (RADAR_REG_PACR1   & 0x7F), 0xE967FD, "PACR1"   },
		{ (RADAR_REG_PACR2   & 0x7F), 0x4805B4, "PACR2"   },
		// { (RADAR_REG_SFCTL & 0x7F), 0x1087FF, "SFCTL"  },
		{ (RADAR_REG_SFCTL   & 0x7F), 0x000000, "SFCTL"  }, // disable hi-speed SPI and strong pin driving
		{ (RADAR_REG_CSCI    & 0x7F), 0xD0D9E0, "CSCI"    },
		{ (RADAR_REG_CSCDS   & 0x7F), 0x000960, "CSCDS"   },
		{ (RADAR_REG_CS1_U_0 & 0x7F), 0x003C51, "CS1_U_0" },
		{ (RADAR_REG_CS1_U_1 & 0x7F), 0x14041F, "CS1_U_1" }, // MADC_BBCH1_EN (bit 20) is set here
		{ (RADAR_REG_CS1_U_2 & 0x7F), 0x00000B, "CS1_U_2" },
		{ (RADAR_REG_CS1     & 0x7F), 0x000490, "CS1"     },
		{ (RADAR_REG_CS2     & 0x7F), 0x000480, "CS2"     },
		{ (RADAR_REG_CS3     & 0x7F), 0x000480, "CS3"     },
		{ (RADAR_REG_CS4     & 0x7F), 0x000480, "CS4"     },
		{ (RADAR_REG_CCR0    & 0x7F), 0x11BE0E, "CCR0"    },
		{ (RADAR_REG_CCR1    & 0x7F), 0x989C0A, "CCR1"    },
		{ (RADAR_REG_CCR2    & 0x7F), 0x000000, "CCR2"    },
		{ (RADAR_REG_CCR3    & 0x7F), 0xBF3E1E, "CCR3"    },
		{ (RADAR_REG_PLL1_0  & 0x7F), 0xA83662, "PLL1_0"  },
		{ (RADAR_REG_PLL1_1  & 0x7F), 0x00030D, "PLL1_1"  },
		{ (RADAR_REG_PLL1_2  & 0x7F), 0x000532, "PLL1_2"  },
		{ (RADAR_REG_PLL1_3  & 0x7F), 0x000200, "PLL1_3"  },
		{ (RADAR_REG_PLL1_7  & 0x7F), 0x000110, "PLL1_7"  },
		{ (RADAR_REG_PLL2_7  & 0x7F), 0x000100, "PLL2_7"  },
		{ (RADAR_REG_PLL3_7  & 0x7F), 0x000100, "PLL3_7"  },
		{ (RADAR_REG_PLL4_7  & 0x7F), 0x000100, "PLL4_7"  },
		{ (RADAR_REG_ADC1    & 0x7F), 0x0A0000, "ADC1"    },
		{ (RADAR_REG_FD      & 0x7F), 0x000400, "FD"      },
		{ (RADAR_REG_WU      & 0x7F), 0x000827, "WU"      },
		{ (RADAR_REG_MAIN    & 0x7F), 0x1C0E20, "MAIN"    }
	};

	reset_subsystem(dev, RESET_FIFO);
	reset_subsystem(dev, RESET_FSM);
	reset_subsystem(dev, RESET_SOFTWARE);

	LOG_INF("Initializing %d registers...", ARRAY_SIZE(reg_init_table));
	for (size_t i = 0; i < ARRAY_SIZE(reg_init_table); ++i) {
		int ret = write_register(dev, reg_init_table[i].reg_7bit, reg_init_table[i].val_24bit);
		if (ret < 0) {
			LOG_ERR("Failed to write 0x%06X to %s (0x%02X): %d", reg_init_table[i].val_24bit,
					reg_init_table[i].name, reg_init_table[i].reg_7bit, ret);
			return ret;
		}

		#if CONFIG_BGT60_RADAR_VERIFY_WRITES
		uint32_t read_value;
		ret = read_register(dev, reg_init_table[i].reg_7bit, &read_value);
		if (ret < 0) {
			LOG_ERR("Failed to read-back %s (0x%02X): %d", reg_init_table[i].name,
					reg_init_table[i].reg_7bit, ret);
			return ret;
		} else if (read_value != reg_init_table[i].val_24bit) {
			LOG_WRN("Verify FAIL %s (0x%02X): W:0x%06X, R:0x%06X",
					reg_init_table[i].name, reg_init_table[i].reg_7bit,
					reg_init_table[i].val_24bit, read_value);
			return -EIO;
		} else {
			LOG_DBG("Verified %s (0x%02X): 0x%06X", reg_init_table[i].name,
					reg_init_table[i].reg_7bit, read_value);
		}
		#endif
	}
	LOG_INF("Finished writing initial register values.");
	return 0;
}

/**
 * @brief Reset parts of the radar sensor.
 *
 * This sensor allows for requests to reset the software, the FSM, and the FIFO.
 *
 * @param dev Pointer to the sensor device structure.
 * @param subsystem The subsystem to reset.
 * @return 0 on success, negative error code otherwise.
 */
static int reset_subsystem(const struct device *dev, resetable_subsystem subsystem)
{
	const uint32_t reset_bit = 1U << subsystem;  // convert subsystem to register bit

	uint32_t main_val;
	int ret = read_register(dev, RADAR_REG_MAIN, &main_val);
	if (ret < 0) {
		LOG_DBG("Failed to read MAIN for reset: %d", ret);
		return ret;
	}

	main_val |= reset_bit;

	ret = write_register(dev, RADAR_REG_MAIN, main_val);
	if (ret < 0) {
		LOG_DBG("Failed to write MAIN for reset (0x%06X): %d", main_val, ret);
	}
	return ret;
}

/**
 * @brief Initialization function for the radar sensor driver.
 */
int bgt60_init(const struct device *dev)
{
	struct bgt60_data *data = dev->data;
	const struct bgt60_config *config = dev->config;

	data->dev = dev;
	// Set default number of FIFO blocks to read. Can be changed by SENSOR_ATTR_CONFIGURATION.
	data->num_fifo_blocks_to_read = RADAR_MAX_FIFO_BLOCKS;
	LOG_INF("Default FIFO blocks to read: %u", data->num_fifo_blocks_to_read);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI device not ready: %s", config->bus.bus->name);
		return -ENODEV;
	}

	// Perform hardware initialization (writing registers, enabling components, etc.)
	if (initialize_sensor(dev) != 0) {
		LOG_ERR("Failed to initialize radar hardware specific settings");
		return -EIO;
	}

	LOG_INF("Initialized FMCW Radar sensor: %s", dev->name);
	return 0;
}

#define BGT60_SPI_CFG SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define BGT60_DEFINE(inst)                                                  \
	IF_ENABLED(CONFIG_BGT60_STREAM, (BGT60_RTIO_DEFINE(inst)));             \
	static struct bgt60_data bgt60_data_##inst = {                          \
	IF_ENABLED(CONFIG_BGT60_STREAM, (.rtio_ctx = &bgt60_rtio_ctx_##inst,    \
				.iodev = &bgt60_iodev_##inst,))                             \
	};                                                                      \
	static const struct bgt60_config bgt60_config_##inst = {                \
		.bus = SPI_DT_SPEC_INST_GET(inst, BGT60_SPI_CFG, 0),                \
	};																	  \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,									  \
			bgt60_init,								 \
			NULL, /* No PM support for now */				  \
			&bgt60_data_##inst,								\
			&bgt60_config_##inst,							  \
			POST_KERNEL,									   \
			CONFIG_SENSOR_INIT_PRIORITY,					   \
			&bgt60_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(BGT60_DEFINE)
