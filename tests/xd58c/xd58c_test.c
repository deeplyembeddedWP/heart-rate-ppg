/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unity unit tests for heart‑rate-monitor/lib/xd58c/xd58c.c
 */

/*
 * 1. Define configuration flags FIRST.
 */
#define CONFIG_XD58C 1
#ifndef CONFIG_XD58C_LOG_LEVEL
#define CONFIG_XD58C_LOG_LEVEL 3
#endif

/*---------------------------------------------------------------------------
 *  2. Standard C & Zephyr System Headers
 *---------------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h> // for k_msgq
#include <zephyr/logging/log.h>

// Unity API
#include <unity.h>

/*---------------------------------------------------------------------------
 *  3. Mock state & Function prototypes
 *---------------------------------------------------------------------------*/
static bool _uart_ready = true;
static bool _adc_ready = true;
static bool _adc_setup_ok = true;
static int _last_uart_len = 0;
static char _last_uart_buf[64] = {0};
static void (*_rx_cb)(const struct device *, struct uart_event *,
                      void *) = NULL;

/* Signaled from mock_uart_tx() so a test can block until xd58c_process()
 * (running on its own thread) has actually sent a UART message, instead of
 * polling _last_uart_buf. */
static struct k_sem _test_uart_done;

static struct device mock_uart_device;

bool mock_device_is_ready(const struct device *dev) {
  ARG_UNUSED(dev);
  return _uart_ready;
}

int mock_adc_is_ready_dt(const struct adc_dt_spec *spec) {
  ARG_UNUSED(spec);
  return _adc_ready ? 1 : 0;
}

int mock_adc_channel_setup_dt(const struct adc_dt_spec *spec) {
  ARG_UNUSED(spec);
  return _adc_setup_ok ? 0 : -EINVAL;
}

int mock_adc_sequence_init_dt(const struct adc_dt_spec *spec,
                              struct adc_sequence *seq) {
  ARG_UNUSED(spec);
  ARG_UNUSED(seq);
  return 0;
}

int mock_adc_read_async(const struct device *dev,
                        const struct adc_sequence *seq, void *user_data) {
  ARG_UNUSED(dev);
  ARG_UNUSED(seq);
  ARG_UNUSED(user_data);
  return 0;
}

/* Prototype for mock_uart_tx (defined below driver include) */
int mock_uart_tx(const struct device *dev, const uint8_t *data, size_t len,
                 int32_t timeout);

int mock_uart_callback_set(const struct device *dev,
                           void (*cb)(const struct device *,
                                      struct uart_event *, void *),
                           void *user_data) {
  ARG_UNUSED(dev);
  ARG_UNUSED(user_data);
  _rx_cb = cb;
  return 0;
}

/*---------------------------------------------------------------------------
 *  4. Macro API Mapping & Stubs (Constants only for global initializer pass)
 *---------------------------------------------------------------------------*/
#undef ADC_DT_SPEC_GET
#define ADC_DT_SPEC_GET(node)                                                  \
  {.dev = NULL, .channel_id = 0, .resolution = 0, .oversampling = 0}

#undef DEVICE_DT_GET
#define DEVICE_DT_GET(node) NULL

#undef device_is_ready
#define device_is_ready mock_device_is_ready
#define adc_is_ready_dt mock_adc_is_ready_dt
#define adc_channel_setup_dt mock_adc_channel_setup_dt
#define adc_sequence_init_dt mock_adc_sequence_init_dt
#define adc_read_async mock_adc_read_async

#undef uart_tx
#define uart_tx mock_uart_tx
#undef uart_callback_set
#define uart_callback_set mock_uart_callback_set

// Helper macro for Unity
#define TEST_ASSERT_OK(res) TEST_ASSERT_EQUAL_INT(0, (res))

/*---------------------------------------------------------------------------
 *  5. Driver API and Source Execution
 *---------------------------------------------------------------------------*/
#include <lib/xd58c.h>

#include <../lib/xd58c/xd58c.c>

/*---------------------------------------------------------------------------
 *  6. Mock Definitions Requiring Driver Internals
 *---------------------------------------------------------------------------*/
int mock_uart_tx(const struct device *dev, const uint8_t *data, size_t len,
                 int32_t timeout) {
  ARG_UNUSED(dev);
  ARG_UNUSED(timeout);
  _last_uart_len = (int)len;
  if (len <= sizeof(_last_uart_buf))
    memcpy(_last_uart_buf, data, len);

  /* Real '_this' structure from xd58c.c is naturally visible here */
  k_sem_give(&_this.tx_sem);
  k_sem_give(&_test_uart_done);

  return 0;
}

// Test setup state clearing and structural assignments
void setUp(void) {
  _uart_ready = true;
  _adc_ready = true;
  _adc_setup_ok = true;
  _last_uart_len = 0;
  memset(_last_uart_buf, 0, sizeof(_last_uart_buf));
  _rx_cb = NULL;
  k_sem_init(&_test_uart_done, 0, 1);

  /* Safe pointer assignment fixes the segmentation fault bug */
  mock_uart_device.name = "mock_uart0";

  // Cast away the const restriction on initialization pointers to swap them
  // dynamically
  struct device **uart_ptr = (struct device **)&_this.uart;
  *uart_ptr = &mock_uart_device;
}

void tearDown(void) {}

/*---------------------------------------------------------------------------
 *    Test cases: xd58c_init
 *---------------------------------------------------------------------------*/
static void test_xd58c_init_success(void) { TEST_ASSERT_OK(xd58c_init()); }

static void test_xd58c_init_no_uart(void) {
  _uart_ready = false;
  TEST_ASSERT_EQUAL_INT(-ENODEV, xd58c_init());
}

static void test_xd58c_init_no_adc(void) {
  _adc_ready = false;
  TEST_ASSERT_EQUAL_INT(-ENODEV, xd58c_init());
}

static void test_xd58c_init_adc_setup_error(void) {
  _adc_setup_ok = false;
  TEST_ASSERT_EQUAL_INT(-EINVAL, xd58c_init());
}

/*---------------------------------------------------------------------------
 *    Test cases: _median_of
 *---------------------------------------------------------------------------*/
static void test_median_of_unsorted(void) {
  float32_t vals[5] = {3.0f, 1.0f, 4.0f, 1.5f, 2.0f};
  TEST_ASSERT_EQUAL_FLOAT(2.0f, _median_of(vals, 5U));
}

static void test_median_of_already_ascending(void) {
  float32_t vals[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  TEST_ASSERT_EQUAL_FLOAT(3.0f, _median_of(vals, 5U));
}

static void test_median_of_already_descending(void) {
  float32_t vals[5] = {5.0f, 4.0f, 3.0f, 2.0f, 1.0f};
  TEST_ASSERT_EQUAL_FLOAT(3.0f, _median_of(vals, 5U));
}

static void test_median_of_all_equal(void) {
  float32_t vals[5] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
  TEST_ASSERT_EQUAL_FLOAT(2.0f, _median_of(vals, 5U));
}

static void test_median_of_outlier_rejected(void) {
  /* One block's peak_bin landed on an octave-error outlier (0.0); the other
   * four agree -- the median should ignore the outlier entirely. */
  float32_t vals[5] = {6.0f, 6.1f, 0.0f, 5.9f, 6.0f};
  TEST_ASSERT_EQUAL_FLOAT(6.0f, _median_of(vals, 5U));
}

/*---------------------------------------------------------------------------
 *    Test cases: _parabolic_interpolate
 *---------------------------------------------------------------------------*/
static void test_parabolic_interpolate_symmetric_peak(void) {
  /* peak_bin=5 (interior); neighbors equal on both sides -> offset 0. */
  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  hps_vals[5U - 1U - XD58C_BPM_BIN_MIN] = 1.0f; /* bin 4 */
  hps_vals[5U - XD58C_BPM_BIN_MIN] = 4.0f;      /* bin 5 (peak) */
  hps_vals[5U + 1U - XD58C_BPM_BIN_MIN] = 1.0f; /* bin 6 */

  TEST_ASSERT_EQUAL_FLOAT(5.0f, _parabolic_interpolate(hps_vals, 5U));
}

static void test_parabolic_interpolate_skewed_toward_next(void) {
  /* peak_bin=5; next-bin energy > prev-bin energy -> offset shifts toward
   * the next (higher) bin. */
  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  hps_vals[5U - 1U - XD58C_BPM_BIN_MIN] = 1.0f; /* bin 4 */
  hps_vals[5U - XD58C_BPM_BIN_MIN] = 4.0f;      /* bin 5 (peak) */
  hps_vals[5U + 1U - XD58C_BPM_BIN_MIN] = 2.0f; /* bin 6 */

  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.1f, _parabolic_interpolate(hps_vals, 5U));
}

static void test_parabolic_interpolate_flat_top_no_offset(void) {
  /* Equal energy on all three bins -> denom is 0, offset stays 0. */
  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  hps_vals[5U - 1U - XD58C_BPM_BIN_MIN] = 3.0f;
  hps_vals[5U - XD58C_BPM_BIN_MIN] = 3.0f;
  hps_vals[5U + 1U - XD58C_BPM_BIN_MIN] = 3.0f;

  TEST_ASSERT_EQUAL_FLOAT(5.0f, _parabolic_interpolate(hps_vals, 5U));
}

static void test_parabolic_interpolate_low_edge_no_interpolation(void) {
  /* peak_bin == XD58C_BPM_BIN_MIN: no lower neighbor, must return the bin
   * unmodified even if hps_vals is heavily skewed. */
  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  hps_vals[XD58C_BPM_BIN_MIN - XD58C_BPM_BIN_MIN] = 100.0f;
  hps_vals[XD58C_BPM_BIN_MIN + 1U - XD58C_BPM_BIN_MIN] = 1.0f;

  TEST_ASSERT_EQUAL_FLOAT((float32_t)XD58C_BPM_BIN_MIN,
                          _parabolic_interpolate(hps_vals, XD58C_BPM_BIN_MIN));
}

static void test_parabolic_interpolate_high_edge_no_interpolation(void) {
  /* peak_bin == XD58C_BPM_BIN_MAX: no upper neighbor, same guard. */
  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  hps_vals[XD58C_BPM_BIN_MAX - 1U - XD58C_BPM_BIN_MIN] = 1.0f;
  hps_vals[XD58C_BPM_BIN_MAX - XD58C_BPM_BIN_MIN] = 100.0f;

  TEST_ASSERT_EQUAL_FLOAT((float32_t)XD58C_BPM_BIN_MAX,
                          _parabolic_interpolate(hps_vals, XD58C_BPM_BIN_MAX));
}

/*---------------------------------------------------------------------------
 *    Test cases: _hps_peak_bin
 *---------------------------------------------------------------------------*/
static void test_hps_peak_bin_finds_dominant_bin(void) {
  const uint32_t k0 = 6U;

  /* Zero the whole spectrum, then place nonzero energy at exactly the three
   * harmonics _hps_peak_bin() reads for bin k0 (2*k0, 4*k0, 6*k0). Every
   * other candidate bin's own harmonic triplet still has at least one
   * factor at 0, so its HPS product stays exactly 0 -- k0 is guaranteed to
   * be the unique, deterministic peak. */
  memset(_this.fft_output, 0, sizeof(_this.fft_output));
  _this.fft_output[2U * k0] = 1000.0f;
  _this.fft_output[4U * k0] = 1000.0f;
  _this.fft_output[6U * k0] = 1000.0f;

  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  float32_t hps_peak = 0.0f;
  uint32_t peak_bin = _hps_peak_bin(hps_vals, &hps_peak);

  TEST_ASSERT_EQUAL_UINT32(k0, peak_bin);
  TEST_ASSERT_FLOAT_WITHIN(1e13f, 1e18f, hps_peak);
  TEST_ASSERT_FLOAT_WITHIN(1e13f, 1e18f, hps_vals[k0 - XD58C_BPM_BIN_MIN]);

  for (uint32_t k = XD58C_BPM_BIN_MIN; k <= XD58C_BPM_BIN_MAX; k++) {
    if (k == k0) {
      continue;
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, hps_vals[k - XD58C_BPM_BIN_MIN]);
  }
}

static void test_hps_peak_bin_all_zero_defaults_to_bin_min(void) {
  memset(_this.fft_output, 0, sizeof(_this.fft_output));

  float32_t hps_vals[XD58C_HPS_BIN_COUNT] = {0};
  float32_t hps_peak = 123.0f;
  uint32_t peak_bin = _hps_peak_bin(hps_vals, &hps_peak);

  /* No bin exceeds the initial hps_peak=0.0f, so the peak_bin/hps_peak
   * seeds are returned unchanged. */
  TEST_ASSERT_EQUAL_UINT32(XD58C_BPM_BIN_MIN, peak_bin);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, hps_peak);
}

/*---------------------------------------------------------------------------
 *    Test cases: full pipeline (xd58c_process -> UART "BPM:%u\r\n")
 *---------------------------------------------------------------------------*/
#define TEST_PROCESS_STACK_SIZE 4096
#define TEST_PI 3.14159265358979323846f

K_THREAD_STACK_DEFINE(_test_process_stack, TEST_PROCESS_STACK_SIZE);
static struct k_thread _test_process_thread;

static void _process_thread_entry(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);
  (void)xd58c_process(); /* loops forever; the test just leaves it running */
}

static void test_xd58c_process_reports_bpm_over_uart(void) {
  TEST_ASSERT_OK(xd58c_init());

  k_thread_create(&_test_process_thread, _test_process_stack,
                  TEST_PROCESS_STACK_SIZE, _process_thread_entry, NULL, NULL,
                  NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

  /* A pure tone at exactly bin k0 -> expected BPM = k0 * (200*60)/512. */
  const uint32_t k0 = 6U;
  const uint32_t expected_bpm =
      (uint32_t)((float32_t)k0 * (float32_t)(XD58C_FFT_SAMPLE_RATE * 60U) /
                     (float32_t)XD58C_FFT_SIZE +
                 0.5f);
  const float32_t freq_per_sample =
      (float32_t)k0 / (float32_t)XD58C_FFT_SIZE;

  /* Feed enough blocks for the HPF/LPF transient to settle and for the
   * 5-block median-filter history to fill entirely with this tone, so the
   * UART-reported bpm reflects the steady-state (median-filtered) value. */
  const uint32_t num_blocks = 10U;
  uint32_t sample_n = 0U;

  for (uint32_t block = 0U; block < num_blocks; block++) {
    for (uint32_t i = 0U; i < XD58C_FFT_SIZE; i++, sample_n++) {
      int16_t sample = (int16_t)(2000.0f * sinf(2.0f * TEST_PI *
                                                 freq_per_sample *
                                                 (float32_t)sample_n));
      TEST_ASSERT_OK(k_msgq_put(&_this.queue, &sample, K_FOREVER));
    }

    int err = k_sem_take(&_test_uart_done, K_MSEC(5000));
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, err,
                                  "xd58c_process did not send a UART message "
                                  "for this block in time");
  }

  _last_uart_buf[_last_uart_len] = '\0';

  unsigned int bpm = 0U;
  int matched = sscanf(_last_uart_buf, "BPM:%u\r\n", &bpm);
  TEST_ASSERT_EQUAL_INT_MESSAGE(1, matched,
                                "UART message did not match \"BPM:%u\\r\\n\"");
  TEST_ASSERT_UINT32_WITHIN(3U, expected_bpm, bpm);
}

// Zephyr Unity Test Entry Point
void test_main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_xd58c_init_success);
  RUN_TEST(test_xd58c_init_no_uart);
  RUN_TEST(test_xd58c_init_no_adc);
  RUN_TEST(test_xd58c_init_adc_setup_error);

  RUN_TEST(test_median_of_unsorted);
  RUN_TEST(test_median_of_already_ascending);
  RUN_TEST(test_median_of_already_descending);
  RUN_TEST(test_median_of_all_equal);
  RUN_TEST(test_median_of_outlier_rejected);

  RUN_TEST(test_parabolic_interpolate_symmetric_peak);
  RUN_TEST(test_parabolic_interpolate_skewed_toward_next);
  RUN_TEST(test_parabolic_interpolate_flat_top_no_offset);
  RUN_TEST(test_parabolic_interpolate_low_edge_no_interpolation);
  RUN_TEST(test_parabolic_interpolate_high_edge_no_interpolation);

  RUN_TEST(test_hps_peak_bin_finds_dominant_bin);
  RUN_TEST(test_hps_peak_bin_all_zero_defaults_to_bin_min);

  /* Must run last: spawns a thread that runs xd58c_process() forever and
   * leaves module statics (s_k_true_hist, filter states, fft_write_idx)
   * populated for the rest of the process's lifetime. */
  RUN_TEST(test_xd58c_process_reports_bpm_over_uart);

  UNITY_END();
}
