# Heart Rate Monitor

Firmware for a heart rate monitor built on the **nRF52840 DK**, using Nordic nRF Connect SDK (NCS) v3.2.0 / Zephyr RTOS 4.2.x.

The application reads photoplethysmography (PPG) data from the **XD58C** pulse sensor via ADC, applies a DC-offset removal filter, and streams AC samples over UART for host-side heart rate processing.

## Hardware

| Component | Detail |
|-----------|--------|
| SoC | nRF52840 |
| Board | nRF52840 DK (`nrf52840dk/nrf52840`) |
| Sensor | XD58C (PPG / heart rate) |
| Interface | ADC (async) + UART (async TX) |
| Logging | Segger RTT (deferred mode) |

## Repository layout

```
app/            Application entry point and Kconfig/prj.conf
lib/xd58c/      XD58C driver (ADC → DC filter → UART TX)
include/lib/    Public header for the xd58c driver
tests/xd58c/    Unit tests (Zephyr ZTEST / Unity, runs on native_sim)
scripts/        Host-side tools
  ppg_monitor.py      Real-time PPG visualiser and heart rate calculator
  ppg_recordings.ods  Two-subject PPG recording dataset (columns: ac_ppg, raw_adc)
keys/           Signing keys — gitignored, not committed
boards/         Custom board definitions (if any)
dts/bindings/   Custom devicetree bindings
```

## How it works

1. `xd58c_init()` — configures the ADC channel at 200 Hz (5 ms interval), registers an async UART callback, and starts continuous ADC sampling.
2. ADC callback — each raw sample passes through a cascaded 4th-order Butterworth highpass (0.5 Hz) then a cascaded 4th-order Butterworth lowpass (4 Hz) to isolate the cardiac AC waveform, then the result is enqueued. See [Filter frequency response](#filter-frequency-response) below for details.
3. `xd58c_process()` — dequeues one sample and transmits it as a decimal string (`"<value>\r\n"`) over UART0.
4. `main()` calls `xd58c_init()` once, then loops on `xd58c_process()`.

## Filter frequency response

`lib/xd58c/xd58c.c` filters every ADC sample through two cascaded biquad filters before it reaches the application:

- **`_hpf_0_5Hz`** — 0.5 Hz highpass, removes DC baseline / slow drift.
- **`_lpf_4Hz`** — 4 Hz lowpass, removes high-frequency noise above the cardiac band.

Together they pass roughly **30–240 BPM** (0.5–4 Hz) with margin around the target 40–230 BPM measurement range.

Both filters are implemented as **2 cascaded biquad sections** (4th-order Butterworth), which was chosen over a single biquad (2nd-order) or a single-pole design (1st-order) specifically for its steeper rolloff — each added order doubles the attenuation slope past the cutoff, at the cost of some added filter state/compute (negligible on the nRF52840) and a slightly slower step response.

| Order | Rolloff | HPF/LPF sections | Notes |
|-------|---------|-------------------|-------|
| 1st | 6 dB/octave | single-pole | Fastest to settle, weakest noise rejection — the old DC-tracking approach was roughly equivalent to this |
| 2nd | 12 dB/octave | 1 biquad | Previous implementation |
| **4th (current)** | **24 dB/octave** | **2 cascaded biquads** | Current implementation |

### Why 4th order

The sensor's raw ADC signal carries a persistent interference tone around **20 Hz** (plus harmonics at 40/60/80 Hz) — likely PWM, mechanical vibration, or a switching artifact — that sits well outside the 0.5–4 Hz cardiac band but close enough that a shallow rolloff barely touches it. Applying each candidate filter order's theoretical lowpass response to a real captured raw spectrum (`samples.log`, finger-on capture) shows the difference concretely:

![Filter order comparison](docs/filter_order_comparison.png)

![Noise suppression by filter order](docs/filter_order_noise_suppression.png)

Measured attenuation at the interference tone and its harmonics, applying each order's 4 Hz lowpass to the same captured raw spectrum:

| Tone | Raw (dB) | After 1st order | After 2nd order | After 4th order (current) |
|------|---------:|-----------------:|-----------------:|---------------------------:|
| 20 Hz (interference) | 9.49 | -4.89 | -18.96 | **-47.40** |
| 40 Hz (2nd harmonic) | 0.79 | -20.45 | -41.62 | **-84.03** |
| 60 Hz (3rd harmonic) | -1.07 | -27.93 | -54.76 | **-108.45** |
| 80 Hz (4th harmonic) | -1.79 | -35.62 | -69.44 | **-137.09** |

At 1st order the 20 Hz interference is only pulled down a few dB, still close to the surrounding floor. At 2nd order it's clearly attenuated but remains a visible bump. At 4th order (current) it's pushed to -47.4 dB — over 55 dB below its raw level and well beneath the noise floor, with the harmonics suppressed even further. This was verified against real hardware captures, not just the analytic model — see the interactive Bode plot / measured PSD below for the live comparison.

**Interactive Bode plot + measured spectrum:** [xd58c Filter Frequency Response](https://claude.ai/code/artifact/59b802e4-a292-4a3a-9cb1-cb7f7538e935) — hover for exact dB values at any frequency, toggle table view, and compare the analytic 4th-order response against a Welch PSD of an actual `samples.log` capture.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) — all build tooling is in the provided `Dockerfile`
- [west](https://docs.zephyrproject.org/latest/develop/west/index.html) — Zephyr meta-tool (installed inside the Docker image)

## Local development setup

Pull the pre-built image (one-time):

```bash
docker pull vinaydivakar/heart-rate-build-env:latest
```

Start a shell inside the container, mounting the workspace:

```bash
docker run --rm -it \
  -v /path/to/heart-rate-workspace:/workdir \
  vinaydivakar/heart-rate-build-env:latest
```

Inside the container, initialize the west workspace (one-time):

```bash
cd /workdir/heart-rate-monitor
west init -l .
west update --narrow
```

## Building

```bash
west build -p always \
  -b nrf52840dk/nrf52840 \
  --sysbuild \
  app \
  -- \
  -DSB_CONFIG_BOOT_SIGNATURE_KEY_FILE="/workdir/heart-rate-monitor/keys/heart-rate-ec-p256-dev.pem"
```

- `--sysbuild` builds MCUboot + the application together.
- The signing key path overrides the hardcoded path in `app/sysbuild.conf`.
- Compiled output lands in `build/`.

### Treat warnings as errors (recommended)

Add `-Dapp_CONFIG_COMPILER_WARNINGS_AS_ERRORS=y` to the build command. This is scoped to the app domain only and does not affect MCUboot.

## Flashing

```bash
west flash
```

## Unit tests

Tests live in `tests/xd58c/` and run on the `native_sim` platform — no hardware required.

```bash
west twister \
  -T tests/ \
  --platform native_sim \
  --inline-logs \
  --outdir twister-out/
```

Test results are written to `twister-out/twister.xml` (JUnit format).

### What is tested

| Test | Description |
|------|-------------|
| `test_xd58c_init_success` | Happy path: UART + ADC ready, init returns 0 |
| `test_xd58c_init_no_uart` | UART not ready → returns `-ENODEV` |
| `test_xd58c_init_no_adc` | ADC not ready → returns `-ENODEV` |
| `test_xd58c_init_adc_setup_error` | ADC channel setup fails → returns `-EINVAL` |
| `test_xd58c_uart_write_format` | Sample enqueued and transmitted as `"<value>\r\n"` |

## CI

GitHub Actions runs on every pull request and push to `main`.

| Job | Runs when | What it does |
|-----|-----------|--------------|
| `changes` | Every push and pull request | Detects whether the `Dockerfile` changed |
| `docker` | `Dockerfile` changed | Builds and pushes the build-env image to Docker Hub |
| `build-and-test` | Always (after `docker` succeeds or is skipped) | Runs inside the build-env container: initializes the west workspace (cached), builds firmware with warnings-as-errors, runs Twister unit tests, uploads `artifacts/` |

The signing key (`MCUBOOT_SIGNING_KEY_PEM`) must be set as a GitHub repository secret.

## Signing keys

Development keys live in `keys/` (gitignored). Generate a new EC P-256 key pair:

```bash
python3 $ZEPHYR_BASE/../bootloader/mcuboot/scripts/imgtool.py keygen \
  -k keys/heart-rate-ec-p256-dev.pem \
  -t ecdsa-p256
```

## License

Apache-2.0
