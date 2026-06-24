/*
 * Low-power BLE NUS demo for Seeed XIAO nRF52840 Sense
 * nRF Connect SDK v3.3.0
 *
 *  - Advertising interval: 1.5 s
 *  - When connected:
 *      · weight in lbs (1 d.p.) over NUS every 500 ms
 *      · raw ADC count over NUS every 500 ms
 *      · internal temperature over NUS every 5 s
 *  - ADS1220 on SPI2: D8=SCK, D9=MISO, D10=MOSI, D3=~CS
 *  - IMU (LSM6DS3TR-C) and PDM mic are unpowered (P1.08 = 0)
 *  - External QSPI flash (P25Q16H) is in Deep Power Down
 *  - CPU spends idle time in System ON (WFE)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_power.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/services/nus.h>

#include "ads1220.h"

/* -------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

#define ADV_INTERVAL_MIN    2400U   /* 2400 × 0.625 ms = 1500 ms */
#define ADV_INTERVAL_MAX    2400U

#define LOAD_PERIOD_MS       500
#define TEMP_EVERY_N_CYCLES  10     /* temperature every 10th load cycle = 5 s */
#define BATT_EVERY_N_CYCLES  20     /* TODO: change to 120 (60 s) for final version; 20 = 10 s for testing */

#define CONN_INTERVAL_MIN   320     /* 1.25 ms units → 400 ms */
#define CONN_INTERVAL_MAX   400     /* 1.25 ms units → 500 ms */
#define CONN_LATENCY        0
#define CONN_TIMEOUT        400     /* 10 ms units  → 4 s */

/* -------------------------------------------------------------------------
 * ASCII formatting helpers (no floating-point printf needed)
 * ------------------------------------------------------------------------- */

/* "W:±XXXXX.X\n"  — weight in lbs to 1 decimal place */
static int format_weight(char *buf, int32_t weight_x10)
{
	char *p = buf;

	*p++ = 'W';
	*p++ = ':';
	if (weight_x10 < 0) {
		*p++ = '-';
		weight_x10 = -weight_x10;
	}

	int32_t i = weight_x10 / 10;
	int32_t f = weight_x10 % 10;

	char tmp[8];
	int n = 0;
	uint32_t v = (uint32_t)i;

	if (v == 0) {
		tmp[n++] = '0';
	} else {
		while (v > 0) {
			tmp[n++] = '0' + (v % 10);
			v /= 10;
		}
	}
	for (int j = n - 1; j >= 0; j--) {
		*p++ = tmp[j];
	}

	*p++ = '.';
	*p++ = '0' + f;
	*p++ = '\n';

	return (int)(p - buf);
}

/* "R:±XXXXXXXX\n"  — raw 24-bit ADC count (averaged in L mode) */
static int format_raw(char *buf, int32_t raw)
{
	char *p = buf;

	*p++ = 'R';
	*p++ = ':';
	if (raw < 0) {
		*p++ = '-';
		raw = -raw;
	}

	char tmp[8];
	int n = 0;
	uint32_t v = (uint32_t)raw;

	if (v == 0) {
		tmp[n++] = '0';
	} else {
		while (v > 0) {
			tmp[n++] = '0' + (v % 10);
			v /= 10;
		}
	}
	for (int i = n - 1; i >= 0; i--) {
		*p++ = tmp[i];
	}
	*p++ = '\n';

	return (int)(p - buf);
}

typedef enum {
	CHARGE_NONE,           /* no VBUS */
	CHARGE_ACTIVE,         /* VBUS present, temperature OK, charging */
	CHARGE_COMPLETE,       /* VBUS present, temperature OK, charge complete */
	CHARGE_INHIBIT_COLD,   /* VBUS present, temperature below 0 °C */
	CHARGE_INHIBIT_HOT,    /* VBUS present, temperature above 50 °C */
} charge_state_t;

/* "B:X.XX\n"  — battery voltage in volts to 2 decimal places */
static int format_batt(char *buf, int32_t mv)
{
	char *p = buf;

	*p++ = 'B';
	*p++ = ':';

	int32_t v = mv / 1000;
	int32_t f = (mv % 1000) / 10;

	*p++ = '0' + v;
	*p++ = '.';
	*p++ = '0' + f / 10;
	*p++ = '0' + f % 10;
	*p++ = '\n';

	return (int)(p - buf);
}

/* "P:XX\n" with an optional suffix depending on charging state.
 * Max: "P:100 Charging Inhibited Cold\n" = 30 chars.
 * Requires ATT MTU ≥ 33 bytes; requested via bt_gatt_exchange_mtu() on connect. */
static int format_soc(char *buf, int32_t pct, charge_state_t state)
{
	char *p = buf;

	*p++ = 'P';
	*p++ = ':';
	if (pct >= 100) {
		*p++ = '1'; *p++ = '0'; *p++ = '0';
	} else {
		if (pct >= 10) {
			*p++ = '0' + pct / 10;
		}
		*p++ = '0' + pct % 10;
	}

	const char *suffix;
	switch (state) {
	case CHARGE_ACTIVE:        suffix = " Charging";    break;
	case CHARGE_COMPLETE:      suffix = " Charged";     break;
	case CHARGE_INHIBIT_COLD:  suffix = " ChgStop Cold"; break;
	case CHARGE_INHIBIT_HOT:   suffix = " ChgStop Hot";  break;
	default:                   suffix = "";              break;
	}
	while (*suffix) {
		*p++ = *suffix++;
	}
	*p++ = '\n';

	return (int)(p - buf);
}

/* "T:±XX.XX\n"  — temperature in °C to 2 decimal places */
static int format_temp(char *buf, int32_t centideg)
{
	char *p = buf;

	*p++ = 'T';
	*p++ = ':';
	if (centideg < 0) {
		*p++ = '-';
		centideg = -centideg;
	}

	int32_t d = centideg / 100;
	int32_t f = centideg % 100;

	if (d >= 100) { *p++ = '0' +  d / 100;       }
	if (d >=  10) { *p++ = '0' + (d /  10) % 10; }
	               *p++ = '0' +  d % 10;

	*p++ = '.';
	*p++ = '0' + f / 10;
	*p++ = '0' + f % 10;
	*p++ = '\n';

	return (int)(p - buf);
}

/* -------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */

static struct bt_conn *current_conn;
static struct k_work adv_work;

/* Calibration: two known points (raw count → lbs).
 * Updated at runtime via NUS 'C' commands. */
static volatile int32_t cal_zero_raw   = -137312;  /* raw count at 0 lbs */
static volatile int32_t cal_span_raw   =  3793638; /* raw count at cal_span_lbs */
static volatile int32_t cal_span_lbs   =  4394;    /* lbs at the span point */
static volatile int      cal_pending    = 0;        /* 0=none, 1=zero, 2=span */
static volatile int32_t  cal_pending_lbs = 0;

#define VBATT_NODE DT_PATH(vbatt)

static const struct adc_dt_spec batt_adc = ADC_DT_SPEC_GET(VBATT_NODE);
static const struct gpio_dt_spec batt_en  = GPIO_DT_SPEC_GET(VBATT_NODE, power_gpios);

/* Returns true when USB VBUS > ~4.4 V (charger connected).
 * Reads the nRF52840 hardware USBDETECT comparator — no USB stack required. */
static bool vbus_present(void)
{
	return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

static const struct gpio_dt_spec chg_stat =
	GPIO_DT_SPEC_GET(DT_NODELABEL(chg_stat_gpio), gpios);
static const struct gpio_dt_spec chg_inhibit =
	GPIO_DT_SPEC_GET(DT_NODELABEL(chg_inhibit_gpio), gpios);

static const struct pwm_dt_spec pwm_r = PWM_DT_SPEC_GET(DT_NODELABEL(led_red_pwm));
static const struct pwm_dt_spec pwm_g = PWM_DT_SPEC_GET(DT_NODELABEL(led_green_pwm));
static const struct pwm_dt_spec pwm_b = PWM_DT_SPEC_GET(DT_NODELABEL(led_blue_pwm));

/* Latest temperature from ADS1220; pre-loaded to 25 °C so charging is
 * not incorrectly inhibited before the first reading arrives. */
static volatile int32_t latest_temp_centideg = 2500;

static int chg_init(void)
{
	if (!gpio_is_ready_dt(&chg_stat) || !gpio_is_ready_dt(&chg_inhibit)) {
		return -ENODEV;
	}
	gpio_pin_configure_dt(&chg_stat,    GPIO_INPUT);
	gpio_pin_configure_dt(&chg_inhibit, GPIO_OUTPUT_INACTIVE); /* floating = allow */
	return 0;
}

/* Drive P0.28 low to inhibit charging when temperature is out of safe range.
 * VBUS detection is not needed here — inhibiting when VBUS is absent is
 * harmless, and the inhibit must engage regardless of VBUS state. */
static void update_charge_inhibit(int32_t temp_centideg)
{
	if (temp_centideg < 0 || temp_centideg > 5000) {
		gpio_pin_set_dt(&chg_inhibit, 1); /* too cold or too hot — inhibit */
	} else {
		gpio_pin_set_dt(&chg_inhibit, 0); /* temperature OK — allow */
	}
}

static charge_state_t get_charge_state(int32_t temp_centideg)
{
	if (temp_centideg < 0) {
		return CHARGE_INHIBIT_COLD;
	}
	if (temp_centideg > 5000) {
		return CHARGE_INHIBIT_HOT;
	}

	/* Temperature is OK. Use P0.17 (BQ25101 CHG, active-low open-drain) as
	 * the primary VBUS proxy: the charger IC only drives this pin LOW when
	 * VBUS is present and it is actively charging. */
	if (gpio_pin_get_dt(&chg_stat) == 1) { /* active = pin LOW = charging */
		return CHARGE_ACTIVE;
	}

	/* P0.17 is HIGH: VBUS absent, charge complete, or charging inhibited.
	 * POWER.USBREGSTATUS.VBUSDETECT is a hardware comparator that works
	 * without the USB stack; vbus_present() reliably distinguishes the two
	 * cases. */
	return vbus_present() ? CHARGE_COMPLETE : CHARGE_NONE;
}

/* -------------------------------------------------------------------------
 * RGB LED driver
 *
 * All three channels are on PWM0 (common anode, PWM_POLARITY_INVERTED):
 *   ch0 = Red (P0.26), ch1 = Green (P0.30), ch2 = Blue (P0.06)
 * pulse == period → fully on; pulse == 0 → fully off.
 *
 * Public API:
 *   LED_On(colour)    — solid on
 *   LED_Flash(colour) — 2 Hz, 50 % duty
 *   LED_Fade(colour)  — 1 Hz triangle wave (0 → 100 → 0 %)
 *   LED_off()         — all channels off immediately
 *
 * Colours: LED_RED, LED_GREEN, LED_BLUE, LED_YELLOW,
 *          LED_CYAN, LED_MAGENTA, LED_WHITE
 * ------------------------------------------------------------------------- */

#define LED_TICK_MS     50
#define LED_FADE_TICKS  20   /* 20 × 50 ms = 1 000 ms = 1 Hz */
#define LED_FLASH_TICKS 10   /* 10 × 50 ms =   500 ms = 2 Hz */

typedef enum {
	LED_RED = 0, LED_GREEN, LED_BLUE,
	LED_YELLOW, LED_CYAN, LED_MAGENTA, LED_WHITE,
} led_colour_t;

/* R, G, B channel intensities in percent (0–100) */
static const uint8_t colour_rgb[][3] = {
	[LED_RED]     = {100,   0,   0},
	[LED_GREEN]   = {  0, 100,   0},
	[LED_BLUE]    = {  0,   0, 100},
	[LED_YELLOW]  = {100, 100,   0},
	[LED_CYAN]    = {  0, 100, 100},
	[LED_MAGENTA] = {100,   0, 100},
	[LED_WHITE]   = {100, 100, 100},
};

typedef enum { PAT_OFF = 0, PAT_ON, PAT_FLASH, PAT_FADE } led_pattern_t;

static volatile led_colour_t  cur_colour  = LED_GREEN;
static volatile led_pattern_t cur_pattern = PAT_OFF;

static struct k_work  led_work;
static struct k_timer led_timer;

static void leds_drive(uint32_t r, uint32_t g, uint32_t b)
{
	pwm_set_dt(&pwm_r, pwm_r.period, pwm_r.period * r / 100);
	pwm_set_dt(&pwm_g, pwm_g.period, pwm_g.period * g / 100);
	pwm_set_dt(&pwm_b, pwm_b.period, pwm_b.period * b / 100);
}

static void LED_On(led_colour_t c)    { cur_colour = c; cur_pattern = PAT_ON;    }
static void LED_Flash(led_colour_t c) { cur_colour = c; cur_pattern = PAT_FLASH; }
static void LED_Fade(led_colour_t c)  { cur_colour = c; cur_pattern = PAT_FADE;  }
static void LED_off(void)
{
	cur_pattern = PAT_OFF;
	leds_drive(0, 0, 0);
}

/* Translates charger state to LED pattern; acts only on state change. */
static void sync_charge_led(void)
{
	static bool           first = true;
	static charge_state_t last;
	charge_state_t cs = get_charge_state(latest_temp_centideg);

	if (!first && cs == last) {
		return;
	}
	first = false;
	last  = cs;
	switch (cs) {
	case CHARGE_ACTIVE:       LED_Fade(LED_GREEN);  break;
	case CHARGE_COMPLETE:     LED_On(LED_GREEN);    break;
	case CHARGE_INHIBIT_HOT:  LED_Flash(LED_RED);   break;
	case CHARGE_INHIBIT_COLD: LED_Flash(LED_BLUE);  break;
	default:                  LED_off();            break;
	}
}

static void led_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	static uint32_t tick;
	tick++;

	sync_charge_led();

	uint32_t scale;
	switch (cur_pattern) {
	case PAT_ON:
		scale = 100;
		break;
	case PAT_FLASH:
		scale = ((tick % LED_FLASH_TICKS) < (LED_FLASH_TICKS / 2)) ? 100 : 0;
		break;
	case PAT_FADE: {
		uint32_t phase = tick % LED_FADE_TICKS;
		uint32_t half  = LED_FADE_TICKS / 2;
		scale = (phase <= half)
			? (phase * 100 / half)
			: ((LED_FADE_TICKS - phase) * 100 / half);
		break;
	}
	default:
		scale = 0;
		break;
	}

	leds_drive(colour_rgb[cur_colour][0] * scale / 100,
		   colour_rgb[cur_colour][1] * scale / 100,
		   colour_rgb[cur_colour][2] * scale / 100);
}

static void led_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&led_work);
}

static void led_boot_sequence(void)
{
	if (!device_is_ready(pwm_r.dev)) {
		return;
	}
	leds_drive(100, 100, 100);
	k_sleep(K_MSEC(1000));
	leds_drive(0, 0, 0);
}

static int led_init(void)
{
	if (!device_is_ready(pwm_r.dev)) {
		return -ENODEV;
	}
	LED_off();
	k_work_init(&led_work, led_work_fn);
	k_timer_init(&led_timer, led_timer_fn, NULL);
	k_timer_start(&led_timer, K_MSEC(LED_TICK_MS), K_MSEC(LED_TICK_MS));
	return 0;
}

static int batt_init(void)
{
	if (!adc_is_ready_dt(&batt_adc) || !gpio_is_ready_dt(&batt_en)) {
		return -ENODEV;
	}
	gpio_pin_configure_dt(&batt_en, GPIO_OUTPUT_INACTIVE);
	return adc_channel_setup_dt(&batt_adc);
}

static int batt_read_mv(int32_t *mv_out)
{
	int16_t raw;
	struct adc_sequence seq = {
		.buffer      = &raw,
		.buffer_size = sizeof(raw),
	};
	adc_sequence_init_dt(&batt_adc, &seq);

	gpio_pin_set_dt(&batt_en, 1);
	k_sleep(K_USEC(1000));
	int err = adc_read_dt(&batt_adc, &seq);
	gpio_pin_set_dt(&batt_en, 0);

	if (err) {
		return err;
	}

	int32_t mv = raw;
	err = adc_raw_to_millivolts_dt(&batt_adc, &mv);
	if (err < 0) {
		return err;
	}

	/* Scale from divided voltage back to actual battery voltage */
	*mv_out = (int32_t)((int64_t)mv
		  * DT_PROP(VBATT_NODE, full_ohms)
		  / DT_PROP(VBATT_NODE, output_ohms));
	return 0;
}

/* LiPo discharge curve — linear interpolation between entries.
 * Accuracy ±10-15% in practice due to load-dependent voltage sag. */
static const struct { int32_t mv; int32_t pct; } lipo_curve[] = {
	{ 4200, 100 },
	{ 4100,  90 },
	{ 4000,  80 },
	{ 3900,  70 },
	{ 3800,  60 },
	{ 3700,  50 },
	{ 3600,  40 },
	{ 3500,  30 },
	{ 3400,  20 },
	{ 3300,  10 },
	{ 3000,   0 },
};

static int32_t batt_soc(int32_t mv)
{
	if (mv >= lipo_curve[0].mv) {
		return 100;
	}
	int n = ARRAY_SIZE(lipo_curve);
	if (mv <= lipo_curve[n - 1].mv) {
		return 0;
	}
	for (int i = 0; i < n - 1; i++) {
		if (mv >= lipo_curve[i + 1].mv) {
			int32_t mv_hi  = lipo_curve[i].mv;
			int32_t mv_lo  = lipo_curve[i + 1].mv;
			int32_t pct_hi = lipo_curve[i].pct;
			int32_t pct_lo = lipo_curve[i + 1].pct;
			return pct_lo +
			       (mv - mv_lo) * (pct_hi - pct_lo) / (mv_hi - mv_lo);
		}
	}
	return 0;
}

#define IMU_PWR_NODE  DT_NODELABEL(imu_pwr_gpio)
static const struct gpio_dt_spec imu_pwr_gpio =
	GPIO_DT_SPEC_GET(IMU_PWR_NODE, gpios);

#define QSPI_FLASH_NODE  DT_NODELABEL(p25q16h)

/* -------------------------------------------------------------------------
 * Advertising data
 * ------------------------------------------------------------------------- */

#define DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN  (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* -------------------------------------------------------------------------
 * Connection callbacks
 * ------------------------------------------------------------------------- */

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(conn); ARG_UNUSED(err); ARG_UNUSED(params);
}

static struct bt_gatt_exchange_params mtu_params = {
	.func = mtu_exchange_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}

	current_conn = bt_conn_ref(conn);

	const struct bt_le_conn_param param = {
		.interval_min = CONN_INTERVAL_MIN,
		.interval_max = CONN_INTERVAL_MAX,
		.latency      = CONN_LATENCY,
		.timeout      = CONN_TIMEOUT,
	};
	(void)bt_conn_le_param_update(conn, &param);

	/* Request Coded PHY (125 kbps, S=8 FEC) for better range.
	 * Falls back silently to 1M PHY if the peer doesn't support it. */
	const struct bt_conn_le_phy_param phy = {
		.options     = BT_CONN_LE_PHY_OPT_CODED_S8,
		.pref_tx_phy = BT_GAP_LE_PHY_CODED,
		.pref_rx_phy = BT_GAP_LE_PHY_CODED,
	};
	(void)bt_conn_le_phy_update(conn, &phy);

	/* Request larger ATT MTU so NUS notifications up to 30 bytes fit.
	 * Without this, the peer may stay at the 23-byte default (20-byte payload)
	 * and longer P: charging-status strings are silently dropped. */
	(void)bt_gatt_exchange_mtu(conn, &mtu_params);
}

static void adv_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN,
		ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
		NULL);

	(void)bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	/* Defer advertising restart — bt_le_adv_start() must not be called
	 * synchronously from the disconnected callback while the stack is
	 * still tearing down the connection. */
	k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

/* -------------------------------------------------------------------------
 * NUS callbacks
 * ------------------------------------------------------------------------- */

static void nus_rx_cb(struct bt_conn *conn,
		      const uint8_t *const data, uint16_t len)
{
	ARG_UNUSED(conn);

	if (len == 0) {
		return;
	}

	switch (data[0]) {
	case 'C': case 'c':
		/* C0      → capture next raw reading as the zero (0 lbs) point.
		 * C<N>    → capture next raw reading as the N lbs span point. */
		if (len >= 2) {
			int32_t val = 0;
			for (uint16_t i = 1; i < len; i++) {
				if (data[i] >= '0' && data[i] <= '9') {
					val = val * 10 + (data[i] - '0');
				}
			}
			if (val == 0) {
				cal_pending = 1;
			} else {
				cal_pending_lbs = val;
				cal_pending = 2;
			}
		}
		break;
	default:
		break;
	}
}

static struct bt_nus_cb nus_cbs = {
	.received = nus_rx_cb,
};

/* -------------------------------------------------------------------------
 * Power down on-board peripherals
 * ------------------------------------------------------------------------- */

static void powerdown_onboard_peripherals(void)
{
	if (gpio_is_ready_dt(&imu_pwr_gpio)) {
		(void)gpio_pin_configure_dt(&imu_pwr_gpio,
					    GPIO_OUTPUT_INACTIVE);
	}

	const struct device *qspi =
		DEVICE_DT_GET_OR_NULL(QSPI_FLASH_NODE);

	if (qspi != NULL && device_is_ready(qspi)) {
		(void)pm_device_action_run(qspi, PM_DEVICE_ACTION_SUSPEND);
	}
}

/* -------------------------------------------------------------------------
 * Sensor thread
 *
 * Every 500 ms sends weight in lbs ("W:±XXXXX.X\n") and raw ADC count
 * ("R:±XXXXXXXX\n"). Every 10th cycle (5 s) also sends temperature
 * ("T:±XX.XX\n").
 * ------------------------------------------------------------------------- */

static void sensor_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	ads1220_init();
	batt_init();
	chg_init();

	uint32_t cycle = 0;

	while (1) {
		k_sleep(K_MSEC(LOAD_PERIOD_MS));
		cycle++;

		/* --- Temperature every 5 s (runs regardless of connection) --- */
		if (cycle % TEMP_EVERY_N_CYCLES == 0) {
			int32_t centideg;
			if (ads1220_read_temp(&centideg) == 0) {
				latest_temp_centideg = centideg;
				update_charge_inhibit(centideg);

				if (current_conn != NULL) {
					char buf[14];
					int len = format_temp(buf, centideg);
					(void)bt_nus_send(current_conn, (uint8_t *)buf, len);
				}
			}
		}

		if (current_conn == NULL) {
			continue;
		}

		/* --- Load measurement ---
		 * On the P: cycle (cycle % BATT_EVERY_N_CYCLES == 1) we still read
		 * the ADS1220 so field calibration always works, but we suppress the
		 * W:/R: sends.  P: is then the only bt_nus_send call that cycle,
		 * avoiding contention for the BLE TX buffer pool. */
		static int32_t batt_mv_cached;
		bool is_soc_cycle = (cycle % BATT_EVERY_N_CYCLES == 1 &&
				     batt_mv_cached != 0);

		int32_t raw;
		if (ads1220_read_load(NULL, &raw) == 0) {
			/* Apply any pending field calibration */
			int pending = cal_pending;
			if (pending == 1) {
				cal_zero_raw = raw;
				cal_pending  = 0;
			} else if (pending == 2) {
				cal_span_raw = raw;
				cal_span_lbs = cal_pending_lbs;
				cal_pending  = 0;
			}

			if (!is_soc_cycle) {
				/* Convert raw count to lbs × 10 */
				int64_t span = (int64_t)cal_span_raw - (int64_t)cal_zero_raw;
				int32_t weight_x10 = 0;
				if (span != 0) {
					weight_x10 = (int32_t)(
						((int64_t)raw - (int64_t)cal_zero_raw)
						* (int64_t)cal_span_lbs * 10LL / span);
				}

				char buf[16];
				int len = format_weight(buf, weight_x10);
				(void)bt_nus_send(current_conn, (uint8_t *)buf, len);
				len = format_raw(buf, raw);
				(void)bt_nus_send(current_conn, (uint8_t *)buf, len);
			}
		}

		/* --- Battery voltage + SoC, staggered across two cycles ---
		 * Cycle N:   read + send B:
		 * Cycle N+1: send P: only (W:/R: suppressed above) */
		if (cycle % BATT_EVERY_N_CYCLES == 0) {
			int32_t batt_mv;
			if (batt_read_mv(&batt_mv) == 0) {
				batt_mv_cached = batt_mv;
				char buf[12];
				int len = format_batt(buf, batt_mv);
				(void)bt_nus_send(current_conn, (uint8_t *)buf, len);
			}
		} else if (is_soc_cycle) {
			char buf[24];
			charge_state_t cs = get_charge_state(latest_temp_centideg);
			int len = format_soc(buf, batt_soc(batt_mv_cached), cs);
			(void)bt_nus_send(current_conn, (uint8_t *)buf, len);
		}
	}
}

K_THREAD_DEFINE(sensor_tid, 2048,
		sensor_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(7), 0, 0);

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
	int err;

	k_work_init(&adv_work, adv_work_handler);

	led_boot_sequence();
	(void)led_init();

	powerdown_onboard_peripherals();

	err = bt_enable(NULL);
	if (err) {
		return 0;
	}

	err = bt_nus_init(&nus_cbs);
	if (err) {
		return 0;
	}

	struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN,
		ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
		NULL);

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		return 0;
	}

	return 0;
}
