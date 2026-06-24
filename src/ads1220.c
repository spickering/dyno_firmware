/*
 * ADS1220 driver
 *
 * SPI2 (D8=SCK, D9=MISO, D10=MOSI, D3=~CS), Mode 1, 4 MHz.
 * AIN1(+)/AIN2(-) bridge sense; AVDD/AVSS ratiometric reference + PSW excitation.
 * GAIN = 128 — suits a 2 mV/V load cell driven at AVDD ≈ 3.3 V.
 */

#include "ads1220.h"

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

/* -------------------------------------------------------------------------
 * SPI device binding
 * ------------------------------------------------------------------------- */

static const struct spi_dt_spec ads1220_spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(ads1220),
	SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPHA | SPI_TRANSFER_MSB);

/* -------------------------------------------------------------------------
 * Commands
 * ------------------------------------------------------------------------- */

#define CMD_RESET          0x06
#define CMD_START          0x08
#define CMD_POWERDOWN      0x02
#define CMD_RDATA          0x10
#define CMD_WREG(r, n)     (0x40 | ((r) << 2) | ((n) - 1))

/* -------------------------------------------------------------------------
 * Register values
 *
 * Load mode (regs 0-2):
 *   Reg 0  0x3E  MUX=AIN1(+)/AIN2(-), GAIN=128, PGA on
 *   Reg 1  0x00  20 SPS, normal mode, single-shot, TS=0
 *   Reg 2  0x88  VREF=AVDD/AVSS (ratiometric), PSW=1
 *
 * Temp mode (regs 1-2):
 *   Reg 1  0x02  TS=1
 *   Reg 2  0x00  Internal reference, PSW=0
 * ------------------------------------------------------------------------- */

#define REG0_LOAD      0x3E
#define REG1_LOAD      0x00   /* 20 SPS (DR=000) — ~50 ms conversion */
#define REG2_LOAD      0x88
#define REG1_TEMP      0x02
#define REG2_TEMP      0x00

#define WAIT_MS_LOAD   60     /* 20 SPS: 50 ms + margin */

/* GAIN × 2^23: denominator for the ratiometric mV/V calculation */
#define FULL_SCALE     1073741824LL   /* 128 × 8 388 608 */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static int send_cmd(uint8_t cmd)
{
	struct spi_buf sb     = { .buf = &cmd, .len = 1 };
	struct spi_buf_set ss = { .buffers = &sb, .count = 1 };
	return spi_write_dt(&ads1220_spi, &ss);
}

static int wreg(uint8_t start_reg, const uint8_t *vals, uint8_t n)
{
	uint8_t tx[5];

	tx[0] = CMD_WREG(start_reg, n);
	for (uint8_t i = 0; i < n; i++) {
		tx[1 + i] = vals[i];
	}

	struct spi_buf tb     = { .buf = tx, .len = 1U + n };
	struct spi_buf_set ts = { .buffers = &tb, .count = 1 };
	return spi_write_dt(&ads1220_spi, &ts);
}

/*
 * Trigger one conversion and clock out the 24-bit result into out[3].
 * Does NOT send POWERDOWN — caller is responsible, allowing multi-sample
 * bursts without re-opening the PSW between samples.
 */
static int start_and_read(uint32_t wait_ms, uint8_t out[3])
{
	if (send_cmd(CMD_START) != 0) {
		return -EIO;
	}

	k_sleep(K_MSEC(wait_ms));

	uint8_t tx[4] = { CMD_RDATA, 0, 0, 0 };
	uint8_t rx[4] = { 0 };
	struct spi_buf txb     = { .buf = tx, .len = 4 };
	struct spi_buf rxb     = { .buf = rx, .len = 4 };
	struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
	if (spi_transceive_dt(&ads1220_spi, &txs, &rxs) != 0) {
		return -EIO;
	}

	/* rx[0] clocked during command byte — discard; rx[1..3] = data */
	out[0] = rx[1];
	out[1] = rx[2];
	out[2] = rx[3];
	return 0;
}

/* Sign-extend 24-bit two's complement raw value to int32_t. */
static int32_t raw24_to_int32(const uint8_t data[3])
{
	int32_t v = ((int32_t)data[0] << 16) |
		    ((int32_t)data[1] << 8)  |
		     (int32_t)data[2];
	return (v << 8) >> 8;   /* arithmetic shift sign-extends bit 23 */
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

int ads1220_init(void)
{
	if (!spi_is_ready_dt(&ads1220_spi)) {
		return -ENODEV;
	}

	if (send_cmd(CMD_RESET) != 0) {
		return -EIO;
	}

	k_sleep(K_MSEC(1));
	return 0;
}

/* Single-shot at 20 SPS; bridge excited for ~50 ms then POWERDOWN. */
int ads1220_read_load(int32_t *mv_v_x10000, int32_t *raw_out)
{
	static const uint8_t cfg[3] = { REG0_LOAD, REG1_LOAD, REG2_LOAD };

	if (wreg(0, cfg, 3) != 0) {
		return -EIO;
	}

	uint8_t data[3];

	if (start_and_read(WAIT_MS_LOAD, data) != 0) {
		(void)send_cmd(CMD_POWERDOWN);
		return -EIO;
	}

	(void)send_cmd(CMD_POWERDOWN);

	int32_t raw = raw24_to_int32(data);

	if (raw_out != NULL) {
		*raw_out = raw;
	}

	if (mv_v_x10000 != NULL) {
		*mv_v_x10000 = (int32_t)((int64_t)raw * 10000000LL / FULL_SCALE);
	}
	return 0;
}

/*
 * 14-bit two's complement in bits [23:10] of the 24-bit result per the
 * datasheet; right-shift 10 to recover the raw count.  LSB = 0.03125 °C.
 */
int ads1220_read_temp(int32_t *centideg)
{
	static const uint8_t cfg[2] = { REG1_TEMP, REG2_TEMP };

	if (wreg(1, cfg, 2) != 0) {
		return -EIO;
	}

	uint8_t data[3];

	if (start_and_read(60, data) != 0) {
		(void)send_cmd(CMD_POWERDOWN);
		return -EIO;
	}

	(void)send_cmd(CMD_POWERDOWN);

	int32_t raw = ((int32_t)data[0] << 16) |
		      ((int32_t)data[1] << 8)  |
		       (int32_t)data[2];
	/* 14-bit value left-justified in 24-bit word per datasheet; shift right 10.
	 * (raw << 8) >> 18 gives a net right shift of 10 with sign extension. */
	raw = (raw << 8) >> 18;

	*centideg = (raw * 25) / 8;
	return 0;
}
