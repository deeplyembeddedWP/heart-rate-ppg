/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <autoconf.h>
#include <lib/xd58c.h>

#include <arm_math.h>
#include <math.h>

LOG_MODULE_REGISTER(xd58c, CONFIG_XD58C_LOG_LEVEL);

#define FFT_SIZE 512U
#define FFT_SAMPLE_RATE 200U
#define BPM_BIN_MIN 1U
#define BPM_BIN_MAX 10U
#define BPM_MEDIAN_WINDOW 5U

#define MESSAGE_QUEUE_SIZE 64

typedef struct {
  float b0, b1, b2;
  float a1, a2;
} biquad_coeffs_t;

typedef struct {
  float x1, x2, y1, y2;
} biquad_state_t;

#define HPF_STAGES 2U
static const biquad_coeffs_t HPF_COEFFS[HPF_STAGES] = {
    {0.9796854872f, -1.959370974f, 0.9796854872f, -1.971148609f, 0.9713918146f},
    {1.0f, -2.0f, 1.0f, -1.98780471f, 0.9880499706f},
};

#define LPF_STAGES 2U
static const biquad_coeffs_t LPF_COEFFS[LPF_STAGES] = {
    {1.32937289e-05f, 2.65874578e-05f, 1.32937289e-05f, -1.778313488f,
     0.7924474718f},
    {1.0f, 2.0f, 1.0f, -1.893415601f, 0.9084644129f},
};

static struct {
  const struct adc_dt_spec adc_chan;
  struct adc_sequence_options adc_seq_opts;
  struct adc_sequence adc_seq;
  int16_t adc_raw;
  const struct device *uart;
  struct k_msgq queue;
  char buffer[MESSAGE_QUEUE_SIZE * sizeof(int16_t)];
  char tx_buf[12];
  struct k_sem tx_sem;

  arm_rfft_fast_instance_f32 fft_rfft;
  float32_t fft_hann[FFT_SIZE];
  float32_t fft_input[FFT_SIZE];
  float32_t fft_output[FFT_SIZE];
  int16_t fft_ibuf[FFT_SIZE];
  uint32_t fft_write_idx;

} _this = {
    .adc_chan = ADC_DT_SPEC_GET(DT_PATH(zephyr_user)),
    .uart = DEVICE_DT_GET(DT_NODELABEL(uart0)),
};

static float _biquad_apply(const biquad_coeffs_t *c, biquad_state_t *s,
                           float input) {
  float output = c->b0 * input + c->b1 * s->x1 + c->b2 * s->x2 - c->a1 * s->y1 -
                 c->a2 * s->y2;

  s->x2 = s->x1;
  s->x1 = input;
  s->y2 = s->y1;
  s->y1 = output;

  return output;
}

static float _hpf_0_5Hz(float input) {
  static biquad_state_t state[HPF_STAGES];

  float output = input;
  for (uint32_t i = 0U; i < HPF_STAGES; i++) {
    output = _biquad_apply(&HPF_COEFFS[i], &state[i], output);
  }

  return output;
}

static float _lpf_4Hz(float input) {
  static biquad_state_t state[LPF_STAGES];

  float output = input;
  for (uint32_t i = 0U; i < LPF_STAGES; i++) {
    output = _biquad_apply(&LPF_COEFFS[i], &state[i], output);
  }

  return output;
}

static enum adc_action _callback(const struct device *dev,
                                 const struct adc_sequence *sequence,
                                 uint16_t sampling_index) {

  int16_t adc_raw = *(int16_t *)sequence->buffer;

  LOG_DBG("raw=%d", adc_raw);

  int err = k_msgq_put(&_this.queue, &adc_raw, K_NO_WAIT);
  if (err) {
    LOG_ERR("msg put %d", err);
  }

  return ADC_ACTION_REPEAT;
}

/**
 * @brief UART transmission callback.
 *
 * Signals the completion or abortion of a UART TX operation using a semaphore.
 *
 * @param dev Pointer to the UART device.
 * @param evt Pointer to the UART event structure.
 * @param user_data User-provided data pointer.
 */
static void _tx_callback(const struct device *dev, struct uart_event *evt,
                         void *user_data) {
  ARG_UNUSED(dev);
  ARG_UNUSED(user_data);

  if (evt->type == UART_TX_DONE || evt->type == UART_TX_ABORTED) {
    k_sem_give(&_this.tx_sem);
  }
}

/**
 * @brief Internal function to format and send a sample over UART.
 *
 * @param raw The 16-bit sample to transmit as a formatted string.
 */
static void _uart_send(const char *buf, size_t len) {
  int err = uart_tx(_this.uart, (const uint8_t *)buf, len, SYS_FOREVER_US);
  if (err) {
    LOG_ERR("uart_tx (err %d)", err);
    return;
  }
  err = k_sem_take(&_this.tx_sem, K_FOREVER);
  if (err) {
    LOG_ERR("uart tx wait (err %d)", err);
  }
}

static void _uart_write_bpm(uint32_t bpm) {
  int len = snprintk(_this.tx_buf, sizeof(_this.tx_buf), "BPM:%u\r\n", bpm);
  if (len < 0 || (size_t)len >= sizeof(_this.tx_buf)) {
    LOG_ERR("snprintk BPM (err %d)", len);
    return;
  }
  // LOG_INF("BPM:%u", bpm);
  _uart_send(_this.tx_buf, (size_t)len);
}

#define HPS_BIN_COUNT (BPM_BIN_MAX - BPM_BIN_MIN + 1U)

/**
 * @brief Harmonic Product Spectrum peak search over _this.fft_output.
 * H[k] = |X[k]|^2 * |X[2k]|^2 * |X[3k]|^2 - Suppresses spurious single-bin
 * energy
 * @param out_hps_vals Filled with every H[k] (BPM_BIN_MIN..BPM_BIN_MAX), so
 *                      the caller can interpolate around the peak.
 * @param out_hps_peak  Set to the HPS value at the winning bin.
 * @return The bin index (BPM_BIN_MIN..BPM_BIN_MAX) with the largest H[k].
 */
static uint32_t _hps_peak_bin(float32_t out_hps_vals[HPS_BIN_COUNT],
                              float32_t *out_hps_peak) {
  float32_t hps_peak = 0.0f;
  uint32_t peak_bin = BPM_BIN_MIN;

  for (uint32_t k = BPM_BIN_MIN; k <= BPM_BIN_MAX; k++) {
    float32_t re, im;

    re = _this.fft_output[2U * k];
    im = _this.fft_output[2U * k + 1U];
    float32_t hps = re * re + im * im;

    re = _this.fft_output[4U * k];
    im = _this.fft_output[4U * k + 1U];
    hps *= re * re + im * im;

    re = _this.fft_output[6U * k];
    im = _this.fft_output[6U * k + 1U];
    hps *= re * re + im * im;

    out_hps_vals[k - BPM_BIN_MIN] = hps;

    if (hps > hps_peak) {
      hps_peak = hps;
      peak_bin = k;
    }
  }

  *out_hps_peak = hps_peak;

  return peak_bin;
}

/**
 * @brief Parabolic (quadratic) interpolation around the HPS peak for
 * sub-bin frequency accuracy.
 *
 * @param hps_vals The full HPS array from _hps_peak_bin().
 * @param peak_bin The winning bin returned by _hps_peak_bin().
 * @return The interpolated, fractional bin index.
 */
static float32_t _parabolic_interpolate(const float32_t hps_vals[HPS_BIN_COUNT],
                                        uint32_t peak_bin) {
  float32_t k_true = (float32_t)peak_bin;

  if (peak_bin > BPM_BIN_MIN && peak_bin < BPM_BIN_MAX) {
    float32_t h_prev = hps_vals[peak_bin - 1U - BPM_BIN_MIN];
    float32_t h_curr = hps_vals[peak_bin - BPM_BIN_MIN];
    float32_t h_next = hps_vals[peak_bin + 1U - BPM_BIN_MIN];
    float32_t denom = h_prev - 2.0f * h_curr + h_next;
    if (denom != 0.0f) {
      float32_t offset = 0.5f * (h_prev - h_next) / denom;
      k_true = (float32_t)peak_bin + offset;
    }
  }

  return k_true;
}

/**
 * @brief Median of the first n entries of vals, sorting a local copy in
 * place. Callers always pass n = BPM_MEDIAN_WINDOW (5), so an O(n^2) bubble
 * sort is simpler than a real sort routine for no real cost.
 *
 * @param vals Array to sort in place; the caller's copy is destroyed.
 * @param n Number of leading entries of vals to sort (always
 *          BPM_MEDIAN_WINDOW).
 * @return The median value (vals[n / 2] after sorting).
 */
static float32_t _median_of(float32_t vals[], uint32_t n) {
  for (uint32_t i = 0U; i < n - 1U; i++) {
    for (uint32_t j = 0U; j < n - 1U - i; j++) {
      if (vals[j] > vals[j + 1U]) {
        float32_t tmp = vals[j];
        vals[j] = vals[j + 1U];
        vals[j + 1U] = tmp;
      }
    }
  }
  return vals[n / 2U];
}

/**
 * @brief Fetches a sample from the message queue, filters it, and either
 * transmits it directly or buffers it for block FFT/BPM processing.
 *
 * @return 0 on success, negative error code on failure.
 */
int xd58c_process(void) {
  int16_t sample = 0;
  float32_t mean = 0.0f;
  float hpf_out = 0.0f, lpf_out = 0.0f;
  float32_t hps_vals[HPS_BIN_COUNT] = {}, k_true_sorted[BPM_MEDIAN_WINDOW] = {};
  static float32_t s_k_true_hist[BPM_MEDIAN_WINDOW] = {};
  static uint32_t s_k_true_idx = 0U;

  while (true) {
    int err = k_msgq_get(&_this.queue, &sample, K_FOREVER);
    if (err) {
      LOG_ERR("dequeue (err %d)", err);
      return err;
    }

    hpf_out = _hpf_0_5Hz(
        (float)sample); // 4th order hpf, fc = 0.5Hz (removes dc baseline)
    lpf_out = _lpf_4Hz(
        hpf_out); // 4th order lpf, fc = 4Hz (removes high frequency noise)

    LOG_DBG("raw=%d hpf_out=%d lpf_out=%d", sample, (int)hpf_out, (int)lpf_out);

    _this.fft_ibuf[_this.fft_write_idx++] = (int16_t)lpf_out;
    if (_this.fft_write_idx < FFT_SIZE) {
      continue;
    }

    _this.fft_write_idx = 0U;
    mean = 0;

    for (uint32_t i = 0U; i < FFT_SIZE; i++) { // compute mean for DC removal
      mean += (float32_t)_this.fft_ibuf[i];
    }
    mean /= (float32_t)FFT_SIZE;

    for (uint32_t i = 0U; i < FFT_SIZE; i++) {
      _this.fft_input[i] =
          ((float32_t)_this.fft_ibuf[i] - mean) * _this.fft_hann[i];
    }

    arm_rfft_fast_f32(&_this.fft_rfft, _this.fft_input, _this.fft_output, 0);

    float32_t hps_peak = 0.0f;
    uint32_t peak_bin = _hps_peak_bin(hps_vals, &hps_peak);
    float32_t k_true = _parabolic_interpolate(hps_vals, peak_bin);

    s_k_true_hist[s_k_true_idx] = k_true;
    s_k_true_idx = (s_k_true_idx + 1U) % BPM_MEDIAN_WINDOW;

    memcpy(k_true_sorted, s_k_true_hist, sizeof(k_true_sorted));
    float32_t k_true_med = _median_of(k_true_sorted, BPM_MEDIAN_WINDOW);

    uint32_t bpm_raw = (uint32_t)(k_true * (float32_t)(FFT_SAMPLE_RATE * 60U) /
                                      (float32_t)FFT_SIZE +
                                  0.5f);
    uint32_t bpm = (uint32_t)(k_true_med * (float32_t)(FFT_SAMPLE_RATE * 60U) /
                                  (float32_t)FFT_SIZE +
                              0.5f);

    /* hps_peak can reach ~1e20 -- print in dB to avoid an overflowing cast. */
    LOG_DBG("peak_bin=%u k_true=%d hps_peak_db=%d bpm_raw=%u bpm=%u", peak_bin,
            (int)(k_true * 100.0f), (int)(10.0f * log10f(hps_peak + 1e-6f)),
            bpm_raw, bpm);
    _uart_write_bpm(bpm);
  }
}

/**
 * @brief Initialize the XD58C heart rate monitor.
 *
 * Configures the ADC channels, initializes sequences, sets up the UART
 * asynchronous callback, and initializes the message queue.
 *
 * @return 0 on success, negative error code on failure.
 */
int xd58c_init(void) {
  if (!device_is_ready(_this.uart)) {
    LOG_ERR("uart0 not ready");
    return -ENODEV;
  }

  int err = adc_is_ready_dt(&_this.adc_chan);
  if (!err) {
    LOG_ERR("ADC device not ready");
    return -ENODEV;
  }

  err = adc_channel_setup_dt(&_this.adc_chan);
  if (err < 0) {
    LOG_ERR("setup ADC channel (err %d)", err);
    return err;
  }

  _this.adc_seq_opts.interval_us = 5000 /* 5 ms */;
  _this.adc_seq_opts.extra_samplings = 0;
  _this.adc_seq_opts.callback = NULL;
  _this.adc_seq_opts.user_data = NULL;

  _this.adc_seq.options = &_this.adc_seq_opts;
  _this.adc_seq.channels = BIT(_this.adc_chan.channel_id);
  _this.adc_seq.buffer = &_this.adc_raw;
  _this.adc_seq.buffer_size = sizeof(_this.adc_raw);
  _this.adc_seq.resolution = _this.adc_chan.resolution;
  _this.adc_seq.oversampling = _this.adc_chan.oversampling;
  _this.adc_seq.calibrate = false;

  err = adc_sequence_init_dt(&_this.adc_chan, &_this.adc_seq);
  if (err < 0) {
    LOG_ERR("init ADC sequence (err %d)", err);
    return err;
  }

  k_sem_init(&_this.tx_sem, 0, 1);
  _this.fft_write_idx = 0U;

  arm_status arm_err = arm_rfft_fast_init_f32(&_this.fft_rfft, FFT_SIZE);
  if (arm_err != ARM_MATH_SUCCESS) {
    LOG_ERR("RFFT init (arm_err %d)", arm_err);
    return -ENOTSUP;
  }

  arm_hanning_f32(_this.fft_hann, FFT_SIZE);

  err = uart_callback_set(_this.uart, _tx_callback, NULL);
  if (err) {
    LOG_ERR("uart_callback_set (err %d)", err);
    return err;
  }

  k_msgq_init(&_this.queue, _this.buffer, sizeof(int16_t), MESSAGE_QUEUE_SIZE);

  _this.adc_seq_opts.callback = _callback;
  _this.adc_seq_opts.user_data = NULL;

  return adc_read_async(_this.adc_chan.dev, &_this.adc_seq, NULL);
}
