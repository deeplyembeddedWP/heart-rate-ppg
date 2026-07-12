/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <autoconf.h>
#include <lib/xd58c.h>

#if defined(CONFIG_XD58C_FFT_BPM)
#include <arm_math.h>

#define FFT_SIZE 512U
#define FFT_HOP 256U
#define FFT_SAMPLE_RATE 200U
#define BPM_BIN_MIN 1U
#define BPM_BIN_MAX 10U
/* Debug CSV covers bins up to 3*BPM_BIN_MAX so every k in the search range
 * has its 2nd and 3rd harmonic bins available for offline HPS analysis. */
#define FFT_DEBUG_BIN_MAX (BPM_BIN_MAX * 3U)
#endif /* CONFIG_XD58C_FFT_BPM */

LOG_MODULE_REGISTER(xd58c, CONFIG_XD58C_LOG_LEVEL);

#define MESSAGE_QUEUE_SIZE 64
/* ADC counts; larger than typical steady-state baseline wander (observed
 * ~100-450 counts, see README raw_adc characterization), small enough to
 * reliably catch a genuine finger placement/removal transient. Empirical
 * starting point — may need tuning against real contact-transient data. */
#define DC_RESET_THRESHOLD 1500

static struct {
  const struct adc_dt_spec adc_chan;
  struct adc_sequence_options adc_seq_opts;
  struct adc_sequence adc_seq;
  int16_t adc_buf;
  const struct device *uart;
  const int shift;
  int32_t dc;
  struct k_msgq queue;
  char buffer[MESSAGE_QUEUE_SIZE * sizeof(int16_t)];
  char tx_buf[12];
  struct k_sem tx_sem;
#if defined(CONFIG_XD58C_FFT_BPM)
  arm_rfft_fast_instance_f32 fft_rfft;
  float32_t fft_hann[FFT_SIZE];
  float32_t fft_input[FFT_SIZE];
  float32_t fft_output[FFT_SIZE];
  int16_t fft_ibuf[FFT_SIZE];
  uint32_t fft_write_idx;
  uint32_t fft_sample_count;

#if defined(CONFIG_XD58C_FFT_DEBUG)
  char fft_dbg_buf[256]; /* holds "FFT:" + 30 comma-separated 5-digit mags */
#endif                   /* CONFIG_XD58C_FFT_DEBUG */
#endif                   /* CONFIG_XD58C_FFT_BPM */
} _this = {
    .adc_chan = ADC_DT_SPEC_GET(DT_PATH(zephyr_user)),
    .uart = DEVICE_DT_GET(DT_NODELABEL(uart0)),
    .shift = 7,
};

/**
 * @brief ADC sampling callback.
 *
 * This function is called by the ADC driver after each sample is collected.
 * It performs DC offset removal (high-pass filtering) and enqueues the
 * resulting AC signal into the message queue for the application layer.
 *
 * @param dev Pointer to the ADC device.
 * @param sequence Pointer to the ADC sequence.
 * @param sampling_index Index of the current sample.
 * @return ADC_ACTION_REPEAT to continue sampling automatically.
 */
static enum adc_action _callback(const struct device *dev,
                                 const struct adc_sequence *sequence,
                                 uint16_t sampling_index) {

  int16_t raw = *(int16_t *)sequence->buffer;

  /* Detect a large step in the DC level (finger just placed or removed) and
   * snap the tracker straight to it, instead of waiting several time
   * constants (~2s at shift=7) for the leaky integrator to slowly converge. */
  int32_t dc_est = _this.dc >> _this.shift;
  int32_t step = (int32_t)raw - dc_est;
  if (step < 0) {
    step = -step;
  }
  if (step > DC_RESET_THRESHOLD) {
    _this.dc = ((int32_t)raw) << _this.shift;
  }

  _this.dc = _this.dc - (_this.dc >> _this.shift) + raw;
  int16_t ac = raw - (int16_t)(_this.dc >> _this.shift);

  int err = k_msgq_put(&_this.queue, &ac, K_NO_WAIT);
  if (err) {
    LOG_ERR("enqueue (err %d)", err);
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

static void _uart_write(int16_t raw) {
  int len = snprintk(_this.tx_buf, sizeof(_this.tx_buf), "%d\r\n", raw);
  if (len < 0 || !len || (size_t)len >= sizeof(_this.tx_buf)) {
    LOG_ERR("snprintk (err %d)", len);
    return;
  }
  _uart_send(_this.tx_buf, (size_t)len);
}

#if defined(CONFIG_XD58C_FFT_BPM)
static void _uart_write_bpm(uint32_t bpm) {
  int len = snprintk(_this.tx_buf, sizeof(_this.tx_buf), "BPM:%u\r\n", bpm);
  if (len < 0 || (size_t)len >= sizeof(_this.tx_buf)) {
    LOG_ERR("snprintk BPM (err %d)", len);
    return;
  }
  _uart_send(_this.tx_buf, (size_t)len);
}

static void _compute_bpm(void) {
  /* Step 1: compute mean of ring buffer for DC removal */
  float32_t mean = 0.0f;

  for (uint32_t i = 0U; i < FFT_SIZE; i++) {
    mean += (float32_t)_this.fft_ibuf[i];
  }
  mean /= (float32_t)FFT_SIZE;

  /* Step 2: unroll circular buffer into fft_input, subtract mean and apply Hann
   * window */
  uint32_t rd = _this.fft_write_idx;
  for (uint32_t i = 0U; i < FFT_SIZE; i++) {
    _this.fft_input[i] =
        ((float32_t)_this.fft_ibuf[rd] - mean) * _this.fft_hann[i];
    rd = (rd + 1U) & (FFT_SIZE - 1U);
  }

  /* Step 3: forward real FFT — transforms fft_input (time domain) into
   * fft_output (frequency domain).*/
  arm_rfft_fast_f32(&_this.fft_rfft, _this.fft_input, _this.fft_output, 0);

#if defined(CONFIG_XD58C_FFT_DEBUG)
  /* Emit CSV of sqrtf magnitudes for bins 1-FFT_DEBUG_BIN_MAX so offline
   * analysis can reconstruct full 3-harmonic HPS for every k up to
   * BPM_BIN_MAX, not just k where 2k,3k <= BPM_BIN_MAX. */
  int pos = snprintk(_this.fft_dbg_buf, sizeof(_this.fft_dbg_buf), "FFT:");
  for (uint32_t k = BPM_BIN_MIN; k <= FFT_DEBUG_BIN_MAX; k++) {
    float32_t re = _this.fft_output[2U * k];
    float32_t im = _this.fft_output[2U * k + 1U];
    uint32_t mag = (uint32_t)sqrtf(re * re + im * im);
    pos += snprintk(_this.fft_dbg_buf + pos, sizeof(_this.fft_dbg_buf) - pos,
                    k < FFT_DEBUG_BIN_MAX ? "%u," : "%u\r\n", mag);
  }
  _uart_send(_this.fft_dbg_buf, (size_t)pos);
#endif /* CONFIG_XD58C_FFT_DEBUG */

  /* Step 4: Harmonic Product Spectrum peak search.
   * H[k] = |X[k]|^2 * |X[2k]|^2 * |X[3k]|^2
   * Suppresses harmonics: the fundamental is the only bin where all three
   * harmonic slots are simultaneously large. Uses squared magnitude to
   * avoid sqrtf. Harmonics up to 3k=30 are well within the 256-bin output.
   * hps_vals[] keeps every H[k] (not just the peak) so Step 5 can look up
   * the peak's neighbors for parabolic interpolation. */
  float32_t hps_vals[BPM_BIN_MAX - BPM_BIN_MIN + 1U];
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

    hps_vals[k - BPM_BIN_MIN] = hps;

    if (hps > hps_peak) {
      hps_peak = hps;
      peak_bin = k;
    }
  }

  /* Step 5: parabolic interpolation around peak_bin for sub-bin accuracy.
   * Fits a parabola through (peak_bin-1, peak_bin, peak_bin+1) in the HPS
   * array and solves for its vertex — the FFT resolution (23.4 BPM/bin) only
   * gives an integer bin, but the true fundamental usually sits between
   * bins. Skipped at the search-range edges, where one neighbor doesn't
   * exist. offset is mathematically bounded to (-0.5, 0.5) whenever it's
   * computed (peak_bin has the largest H[k] of the three by construction),
   * so no extra clamping is needed. */
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

  /* Step 6: convert interpolated bin to BPM and transmit. */
  uint32_t bpm = (uint32_t)(k_true * (float32_t)(FFT_SAMPLE_RATE * 60U) /
                                 (float32_t)FFT_SIZE +
                             0.5f);
  _uart_write_bpm(bpm);
}
#endif /* CONFIG_XD58C_FFT_BPM */

/**
 * @brief Fetches a sample from the message queue and transmits it.
 *
 * @return 0 on success, negative error code on failure.
 */
int xd58c_process(void) {
  int16_t sample;
  int err = k_msgq_get(&_this.queue, &sample, K_FOREVER);
  if (err) {
    LOG_ERR("dequeue (err %d)", err);
    return err;
  }

#if defined(CONFIG_XD58C_FFT_BPM)
  _this.fft_ibuf[_this.fft_write_idx] = sample;
  _this.fft_write_idx = (_this.fft_write_idx + 1U) & (FFT_SIZE - 1U);
  _this.fft_sample_count++;

  /* Fire after warmup (FFT_SIZE samples), then every FFT_HOP samples */
  if (_this.fft_sample_count >= FFT_SIZE &&
      ((_this.fft_sample_count - FFT_SIZE) % FFT_HOP) == 0U) {
    _compute_bpm();
  }
#else
  _uart_write(sample);
#endif /* CONFIG_XD58C_FFT_BPM */

  return 0;
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
  _this.adc_seq.buffer = &_this.adc_buf;
  _this.adc_seq.buffer_size = sizeof(_this.adc_buf);
  _this.adc_seq.resolution = _this.adc_chan.resolution;
  _this.adc_seq.oversampling = _this.adc_chan.oversampling;
  _this.adc_seq.calibrate = false;

  err = adc_sequence_init_dt(&_this.adc_chan, &_this.adc_seq);
  if (err < 0) {
    LOG_ERR("init ADC sequence (err %d)", err);
    return err;
  }

  k_sem_init(&_this.tx_sem, 0, 1);

#if defined(CONFIG_XD58C_FFT_BPM)
  arm_status arm_err = arm_rfft_fast_init_f32(&_this.fft_rfft, FFT_SIZE);
  if (arm_err != ARM_MATH_SUCCESS) {
    LOG_ERR("RFFT init (arm_err %d)", arm_err);
    return -ENOTSUP;
  }
  arm_hanning_f32(_this.fft_hann, FFT_SIZE);
  _this.fft_write_idx = 0U;
  _this.fft_sample_count = 0U;
#endif /* CONFIG_XD58C_FFT_BPM */

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
