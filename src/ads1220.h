#pragma once

#include <zephyr/types.h>

/*
 * ADS1220 driver — public API
 *
 * Hardware assumed by the implementation (set in the board overlay):
 *   SPI2, Mode 1, 4 MHz, CS = P0.29
 *   AIN1(+) / AIN2(-) = bridge sense
 *   AVDD / AVSS = ratiometric reference and bridge excitation via PSW
 *
 * All functions return 0 on success, negative errno on failure.
 */

/* Reset the ADS1220 and verify the SPI bus is ready. */
int ads1220_init(void);

/*
 * Read bridge load — single-shot at 20 SPS.
 *
 * *mv_v_x10000 — mV/V scaled by 10000 (e.g. 20000 = 2.0000 mV/V). Pass NULL
 *                if not needed.
 * *raw_out     — raw 24-bit signed ADC count. Pass NULL if not needed.
 *
 * The bridge is energised (PSW closed) only for the ~50 ms conversion window.
 */
int ads1220_read_load(int32_t *mv_v_x10000, int32_t *raw_out);

/*
 * Read the ADS1220 internal temperature in centidegrees (°C × 100).
 * e.g. *centideg = 2306 means 23.06 °C.
 */
int ads1220_read_temp(int32_t *centideg);
