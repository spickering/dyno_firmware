# Bluemax_dev — Low-Power BLE Load Cell + Battery Monitor

An NCS v3.3.0 application for the Seeed XIAO nRF52840 Sense, optimised for
minimum **System ON idle** standby current while streaming load, temperature,
and battery data over the **Nordic UART Service (NUS)**.

---

## BLE data stream

When a central is connected the device sends the following NUS notifications:

| Message | Format | Interval |
|---|---|---|
| Weight | `W:±XXXXX.X\n` — lbs to 1 d.p. | Every 500 ms |
| Raw ADC | `R:±XXXXXXXX\n` — 24-bit signed count | Every 500 ms |
| Temperature | `T:±XX.XX\n` — °C to 2 d.p. | Every 5 s |
| Battery voltage | `B:X.XX\n` — volts to 2 d.p. | Every 10 s |
| State of charge | `P:XX [suffix]\n` — % + charge state | Every 10 s (cycle after B:) |

State-of-charge suffixes:

| Suffix | Meaning |
|---|---|
| ` Charging` | VBUS present, BQ25101 actively charging |
| ` Charged` | VBUS present, charge complete |
| ` ChgStop Hot` | Charging inhibited — temperature > 50 °C |
| ` ChgStop Cold` | Charging inhibited — temperature < 0 °C |
| _(none)_ | No VBUS (on battery) |

> **ATT payload note.** The default ATT MTU is 23 bytes (20-byte payload).
> `bt_gatt_exchange_mtu()` is called on connect with `CONFIG_BT_L2CAP_TX_MTU=64`
> to negotiate a larger MTU, but until that exchange completes all NUS messages
> must be ≤ 20 bytes. W: and R: are suppressed on the same cycle as P: to
> prevent TX-buffer pool contention.

### NUS commands (central → device)

| Command | Action |
|---|---|
| `C0` | Capture next raw reading as the zero (0 lbs) calibration point |
| `C<N>` | Capture next raw reading as the N-lbs span calibration point |

---

## Hardware

### ADS1220 load cell interface

| XIAO pin | nRF52840 | ADS1220 | Function |
|---|---|---|---|
| D8 | P1.13 | SCK | SPI clock |
| D9 | P1.14 | DOUT/MISO | SPI data out |
| D10 | P1.15 | DIN/MOSI | SPI data in |
| D3 | P0.29 | ~CS | Chip select (active low) |

SPI2 (`xiao_spi`), Mode 1 (CPOL=0, CPHA=1), 4 MHz.

**Single-shot at 20 SPS** — one conversion per 500 ms load cycle (~50 ms
bridge-on time). The PSW switch is closed only during conversion and opened by
an explicit POWERDOWN command (0x02) immediately after RDATA.

#### Bridge configuration

```
AVDD ──── AIN0 ──── Bridge +Excitation
                        │
                    Bridge body
                     │       │
                   AIN1    AIN2      ← differential sense input
                        │
                   AIN3 ──── PSW ──── AVSS
```

- **AIN1 / AIN2** — bridge sense +/−, read differentially
- **AVDD / AVSS** — ratiometric reference (supply cancels out)
- **PSW** — internal low-side switch; closes AIN3→AVSS during conversion only

**GAIN = 128** suits a 2 mV/V load cell at AVDD ≈ 3.3 V. Adjust `REG0_LOAD`
and `FULL_SCALE` in `src/ads1220.c` together if your cell has a different
sensitivity.

#### PSW / POWERDOWN quirk

With `PSW=1` in Config Register 2, the ADS1220 does **not** automatically open
the switch after a single-shot conversion — the bridge stays energised. An
explicit POWERDOWN command must follow every RDATA to open PSW and return the
chip to its ~0.5 µA idle state.

### Battery and charger (BQ25101)

| Signal | Pin | Notes |
|---|---|---|
| Battery voltage sense | P0.31 / AIN7 | Via 1 MΩ + 510 kΩ divider; P0.14 enables divider |
| CHG_STAT | P0.17 | Active-low open-drain; LOW = actively charging |
| CHG_INHIBIT | P0.28 | Output to BQ25101 CE; driven LOW to inhibit charging |

SoC is estimated from a linear interpolation of a LiPo discharge curve
(3.0 V = 0 %, 4.2 V = 100 %).

Charging is inhibited (P0.28 driven low) when the ADS1220 internal temperature
is outside 0–50 °C.

**VBUS detection** uses `nrf_power_usbregstatus_vbusdet_get(NRF_POWER)` — the
nRF52840 hardware comparator works without the USB stack initialised.

### RGB LED

The XIAO onboard RGB LED (common anode → 3.3 V, active-low cathodes) is driven
by three PWM0 channels:

| Colour | Pin | PWM0 channel |
|---|---|---|
| Red | P0.26 | ch 0 |
| Green | P0.30 | ch 1 |
| Blue | P0.06 | ch 2 |

PWM polarity is inverted in the overlay (`PWM_POLARITY_INVERTED`): pulse =
period → fully on; pulse = 0 → fully off.

#### Boot sequence

White on for **1 second** at boot, then off, then charge-state indication
begins.

#### Charge-state indication

| Charge state | LED pattern |
|---|---|
| Charging (VBUS, temp OK, BQ25101 active) | Green fade 0→100→0 % at 1 Hz |
| Charge complete | Green solid on |
| Inhibited — too hot (> 50 °C) | Red flash at 2 Hz |
| Inhibited — too cold (< 0 °C) | Blue flash at 2 Hz |
| No VBUS | All off |

#### LED API

Four functions are available from anywhere in `main.c`:

```c
LED_On(led_colour_t colour);    // solid on
LED_Flash(led_colour_t colour); // 2 Hz, 50 % duty
LED_Fade(led_colour_t colour);  // 1 Hz triangle wave
LED_off(void);                  // all channels off immediately
```

Colours: `LED_RED`, `LED_GREEN`, `LED_BLUE`, `LED_YELLOW`, `LED_CYAN`,
`LED_MAGENTA`, `LED_WHITE`. RGB intensities for each colour are in the
`colour_rgb[7][3]` table in `src/main.c` and can be tuned to compensate
for differences in individual LED efficiency.

---

## Files

```
Bluemax_dev/
├── CMakeLists.txt
├── prj.conf
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay
└── src/
    ├── main.c        — BLE, sensor thread, battery, LED driver
    ├── ads1220.c     — ADS1220 SPI driver
    └── ads1220.h     — ADS1220 public API
```

---

## Build

```bash
west build -p always -b xiao_ble/nrf52840/sense
```

Or via the **nRF Connect** VS Code extension:
1. Open the folder.
2. Sidebar → **Add Build Configuration**.
3. Board: `xiao_ble/nrf52840/sense`, SDK `v3.3.0`, toolchain `v3.3.0`.

---

## Flash (SWD — no bootloader)

The project is configured to start at the reset vector (0x00000000). The
Adafruit UF2 bootloader is not used and the `.uf2` output is suppressed.

```bash
# With a J-Link or nRF DK wired to the SWD pads:
west flash

# Or directly with nrfjprog:
nrfjprog --program build/zephyr/zephyr.hex --chiperase --verify -f NRF52
```

> **Why the offset override is needed.** The `xiao_ble` board includes
> `nrf52840_partition_uf2_sdv7.dtsi` which sets `zephyr,code-partition` to
> a node at 0x27000 (the UF2 layout: 156 KB reserved for MBR + SoftDevice
> slot). `CONFIG_USE_DT_CODE_PARTITION=y` (board default) reads the flash load
> address from this DTS node, so any `FLASH_LOAD_OFFSET` assignment in
> `prj.conf` is silently overridden. The fix is
> `CONFIG_USE_DT_CODE_PARTITION=n` in `prj.conf`, which lets the explicit
> `CONFIG_FLASH_LOAD_OFFSET=0x0` take effect. `CONFIG_BUILD_OUTPUT_UF2=n`
> suppresses the now-redundant `.uf2` artefact.

---

## Measuring standby current

- Power from the **battery pads**, not USB. VBUS raises the VBUS comparator
  and the BQ25101 draws additional quiescent current.
- **No SWD debugger attached** — the debug interface prevents the deepest idle.
- Disconnect everything from the GPIO header.
- Use a PPK2 in source-meter mode at ~3.7 V.

Observed average current (advertising only, no central connected):

| Phase | Current (typ.) |
|---|---|
| Between adv events | a few µA (System ON idle, RAM retained) |
| Adv event (3 channels) | brief spike during TX/RX windows |
| Average @ 1.5 s interval | low tens of µA |

With a central connected, sending W: + R: at 500 ms, T: at 5 s:

| Phase | Approx. average |
|---|---|
| ADS1220 conversion (20 SPS) | ~0.5 mA for ~50 ms / cycle |
| Idle between conversions | tens of µA |

---

## Configuration notes

- **`CONFIG_TICKLESS_KERNEL=y`** enables System ON idle (WFE) between events.
  Without it the CPU wakes every 1 ms for the system tick.
- **`CONFIG_PM_DEVICE=y`** enables device-level PM, used to suspend the QSPI
  flash driver into Deep Power Down at boot.
- **`CONFIG_USE_DT_CODE_PARTITION=n`** overrides the board default, which
  would otherwise lock the flash load address to 0x27000 (UF2 layout) via the
  DTS `code-partition` node regardless of what `FLASH_LOAD_OFFSET` is set to.
- **`CONFIG_BT_L2CAP_TX_MTU=64`** advertises a larger ATT MTU on connect.
  The default 23-byte MTU (20-byte payload) is too small for some P: messages
  with long suffixes, causing `bt_nus_send()` to return -ENOMEM.
- **DC/DC regulator (REG1)** is enabled via the overlay:
  `&reg1 { regulator-initial-mode = <NRF5X_REG_MODE_DCDC>; }`. In NCS v2.x+
  `CONFIG_SOC_DCDC_NRF52X` has no Kconfig prompt; the DTS property
  auto-selects it. The XIAO populates the required 10 µH inductor.
- **Advertising restart** after disconnection uses a `k_work` item — calling
  `bt_le_adv_start()` synchronously from the `disconnected` callback fails
  silently while the stack is still tearing down the connection.
- **Coded PHY (125 kbps, S=8 FEC)** is requested after connection via
  `bt_conn_le_phy_update()`. If the peer does not support it, the request is
  rejected and the link stays on 1M PHY with no application-level handling
  needed.
- **TX power** is +8 dBm (nRF52840 hardware maximum) via
  `CONFIG_BT_CTLR_TX_PWR_PLUS_8=y`.
- **`SPI_DT_SPEC_GET`** in NCS v3.3.0 takes two arguments (node + operation
  flags); the deprecated third `delay` argument must be omitted.

## ADS1220 temperature shift

The ADS1220 (SBAS501) temperature result is a 14-bit value **left-justified**
in the 24-bit output word. Right-shift by 10 to recover the raw count.
`(raw << 8) >> 18` achieves a net shift of 10 with sign extension. LSB = 0.03125 °C.

---

## Re-enabling debug output

Add temporarily to `prj.conf` (removes these before measuring current):

```
CONFIG_LOG=y
CONFIG_LOG_BACKEND_RTT=y
CONFIG_USE_SEGGER_RTT=y
CONFIG_LOG_MODE_DEFERRED=y
CONFIG_PRINTK=y
```
