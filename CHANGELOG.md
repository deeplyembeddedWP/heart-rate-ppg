# Changelog

## 1.1.0

- Replaced raw-sample UART streaming with on-device BPM computation: 512-sample FFT blocks, Hann windowing, Harmonic Product Spectrum peak detection, parabolic sub-bin interpolation, and a 5-block median filter — more stable and accurate readings than the old host-side approach.
- UART now sends `"BPM:<value>\r\n"` instead of raw samples.
- `ppg_monitor.py`: shows the raw BPM directly plus a trend-based confidence score (see README).

## 1.0.0

- Initial release: ADC sampling, HPF/LPF filtering, raw AC samples streamed over UART for host-side heart rate processing.
