"""
PPG Heart Rate Monitor
======================
Reads "BPM:xx" lines from the firmware over serial (the nRF52840 already
computes BPM on-device via FFT + Harmonic Product Spectrum — see
lib/xd58c/xd58c.c) and displays each reading as-is, with no smoothing or
outlier rejection:
  • BPM trend
  • Large BPM readout
  • START button  – begins listening and plotting
  • STOP button   – halts, resets all data and plots

Usage
-----
  python ppg_monitor.py --port /dev/ttyUSB0 --baud 1000000

Firmware assumptions
--------------------
  UART format: "BPM:xx\\r\\n" per HPS update (~every 1.28 s once warmed up).
  Non-BPM lines are ignored.
"""

import argparse
import collections
import math
import statistics
import sys
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MultipleLocator
from matplotlib.widgets import Button

# ── constants ────────────────────────────────────────────────────────────────
BPM_MIN         = 20
BPM_MAX         = 240
BPM_HISTORY     = 30   # readings kept for the trend plot

CONFIDENCE_WINDOW     = 5   # recent readings used as the "past trend" baseline
CONFIDENCE_MAX_DEVIATION = 5   # BPM; a reading further than this from the
                            # trailing trend mean counts as a deviation

PULSE_MIN_SIZE    = 22   # heart glyph fontsize at rest
PULSE_MAX_SIZE    = 34   # heart glyph fontsize at the peak of each beat

# Shape of one beat: a classic ECG PQRST complex — small P wave, a sharp
# QRS spike (with a dip just before and a deeper dip just after, both below
# baseline), a rounded T wave, then a flat isoelectric segment. Modelled as
# a sum of Gaussian bumps, each (phase center, width, amplitude relative to
# the R peak's 1.0); amplitude is negative for the below-baseline dips.
PULSE_ECG_COMPONENTS = [
    (0.10,  0.045, 0.15),   # P wave
    (0.215, 0.010, -0.12),  # Q dip
    (0.25,  0.012,  1.0),   # R spike
    (0.29,  0.020, -0.32),  # S dip
    (0.50,  0.09,   0.28),  # T wave
]

PULSE_WINDOW_SEC  = 4.0  # width of the scrolling pulse-waveform strip
PULSE_STRIP_POINTS = 240 # the strip is regenerated analytically at this
                          # fixed resolution every frame, so it stays smooth
                          # regardless of actual animation-timer jitter —
                          # like a hospital monitor's sweep, not a live plot
                          # that accretes one point per tick

C = {
    "bpm_line":        "#e6484f",
    "pulse":           "#e6484f",
    "pulse_trace":     "#00e0ff",
    "pulse_trace_core": "#e8feff",
    "pulse_bg":        "#000000",
    "pulse_grid":      "#0f4c66",
    "bg":              "#0a0e1a",
    "panel":           "#141a2c",
    "panel_edge":      "#232a42",
    "text":            "#f0f2fa",
    "subtext":         "#8892aa",
    "bpm_value":       "#2ecc71",
    "bar_track":       "#232a42",
    "confidence_low":  "#e6484f",
    "btn_start_on":    "#1d9e75",
    "btn_start_hover": "#25c48e",
    "btn_stop_on":     "#e24b4a",
    "btn_stop_hover":  "#ff6b6b",
    "btn_off":         "#2a2f45",
    "btn_txt":         "#e0e4ef",
}


# ── data source ──────────────────────────────────────────────────────────────
class SerialSource:
    """Reads "BPM:xx" lines from a UART port in a background thread."""

    def __init__(self, port, baud):
        import serial
        self._ser    = serial.Serial(port, baud, timeout=1)
        self._q      = collections.deque()
        self._lock   = threading.Lock()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while True:
            try:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
            except OSError:
                continue
            if not line.startswith("BPM:"):
                continue
            try:
                bpm = int(line[4:])
            except ValueError:
                continue
            with self._lock:
                self._q.append(bpm)

    def drain(self):
        with self._lock:
            items = list(self._q)
            self._q.clear()
        return items

    def start(self):
        """Flush stale bytes that arrived while stopped."""
        with self._lock:
            self._q.clear()

    def reset(self):
        with self._lock:
            self._q.clear()


# ── main application ──────────────────────────────────────────────────────────
class PPGMonitor:

    def __init__(self, source):
        self.source     = source
        self._measuring = False
        self._start_time = None

        self._reset_state()
        self._build_ui()

        self._anim = FuncAnimation(
            self.fig, self._update,
            interval=20,  # smoother pulse animation; new BPM data still only
                          # arrives every ~1.28s regardless of this rate
            blit=False,
            cache_frame_data=False,
        )

    # ── state ─────────────────────────────────────────────────────────────────
    def _reset_state(self):
        self._bpm_times  = collections.deque(maxlen=BPM_HISTORY)
        self._bpm_values = collections.deque(maxlen=BPM_HISTORY)  # raw readings
        self._trend_ref  = collections.deque(maxlen=CONFIDENCE_WINDOW)  # recent
                                # raw readings, used only as the trend baseline
        self._stable_flags = collections.deque(maxlen=CONFIDENCE_WINDOW)  # was
                                # each of the last CONFIDENCE_WINDOW readings
                                # within CONFIDENCE_MAX_DEVIATION of the trend
                                # at the time it arrived?
        self._current_bpm = None
        self._pulse_phase = 0.0   # 0..1 fraction through the current beat cycle
        self._last_frame_time = None

    def _confidence(self):
        """Fraction (0.0-1.0) of the last CONFIDENCE_WINDOW readings that
        stayed within CONFIDENCE_MAX_DEVIATION BPM of the trend mean at the
        time they arrived -- i.e. how stable the BPM trend has been recently.
        None until there's at least one such comparison to judge."""
        if not self._stable_flags:
            return None
        return sum(self._stable_flags) / len(self._stable_flags)

    @staticmethod
    def _pulse_shape(phase):
        """Amplitude at `phase` (0..1) through one beat cycle: a classic ECG
        PQRST complex, built as a sum of Gaussian bumps (PULSE_ECG_COMPONENTS)
        — most of the cycle sits flat at the isoelectric baseline (0)."""
        total = 0.0
        for center, width, amp in PULSE_ECG_COMPONENTS:
            d = phase - center
            if d > 0.5:
                d -= 1.0
            elif d < -0.5:
                d += 1.0
            total += amp * math.exp(-(d / width) ** 2)
        return total

    def _process_bpm(self, raw_bpm):
        """Receive one BPM reading from the firmware and store it directly
        for display, with no smoothing or outlier rejection. Separately,
        check it against the trailing trend for the confidence readout."""
        if self._trend_ref:
            trend = statistics.mean(self._trend_ref)
            self._stable_flags.append(abs(raw_bpm - trend) <= CONFIDENCE_MAX_DEVIATION)
        self._trend_ref.append(raw_bpm)

        elapsed = time.perf_counter() - self._start_time
        self._bpm_times.append(elapsed)
        self._bpm_values.append(raw_bpm)
        self._current_bpm = raw_bpm

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(10, 6), facecolor=C["bg"])
        self.fig.canvas.manager.set_window_title("PPG Heart Rate Monitor")

        gs = gridspec.GridSpec(
            2, 2,
            figure=self.fig,
            left=0.08, right=0.96,
            top=0.88,  bottom=0.22,
            wspace=0.3, hspace=0.4,
            height_ratios=[1, 1.8],
            width_ratios=[2.3, 1],
        )

        # pulse waveform — styled like a hospital ECG monitor: black
        # background, bright blue graph-paper grid, glowing cyan trace ──
        self.ax_pulse = self.fig.add_subplot(gs[0, 0])
        self._style_ax(self.ax_pulse, "Pulse")
        self._round_card(self.ax_pulse, facecolor=C["pulse_bg"])
        self.ax_pulse.set_ylim(-0.55, 1.15)
        self.ax_pulse.set_xlim(0, PULSE_WINDOW_SEC)
        self.ax_pulse.xaxis.set_major_locator(MultipleLocator(0.4))
        self.ax_pulse.yaxis.set_major_locator(MultipleLocator(0.2))
        self.ax_pulse.tick_params(length=0, labelbottom=False, labelleft=False)
        self.ax_pulse.grid(True, color=C["pulse_grid"], linewidth=0.7, alpha=0.9)
        for sp in self.ax_pulse.spines.values():
            sp.set_visible(False)

        # a few stacked, widening, fading copies of the same line fake a
        # neon/CRT glow without a real blur filter (keeps this
        # dependency-free) — a wide soft halo plus a hot near-white core is
        # what actually reads as "electric" rather than just a colored line
        self._ln_pulse_layers = [
            self.ax_pulse.plot(
                [], [], color=color, lw=lw, alpha=alpha, zorder=zorder,
                solid_joinstyle="round", solid_capstyle="round")[0]
            for lw, alpha, color, zorder in [
                (9.0, 0.05, C["pulse_trace"], 3),
                (6.0, 0.10, C["pulse_trace"], 3),
                (3.5, 0.22, C["pulse_trace"], 3),
                (1.6, 0.85, C["pulse_trace"], 4),
                (0.7, 1.00, C["pulse_trace_core"], 5),
            ]
        ]
        # glowing "cursor" dot at the leading edge of the trace, like a
        # monitor's live sweep position — same stacked-circles glow trick
        self._dot_layers = [
            self.ax_pulse.scatter(
                [], [], s=s, color=color, alpha=alpha, zorder=zorder,
                edgecolors="none")
            for s, alpha, color, zorder in [
                (900, 0.05, C["pulse_trace"], 3),
                (450, 0.12, C["pulse_trace"], 3),
                (160, 0.30, C["pulse_trace_core"], 4),
                (45,  1.00, C["pulse_trace_core"], 5),
            ]
        ]
        # shown whenever the signal isn't locked yet — the window/firmware
        # warm-up together take several seconds, so this makes clear it's
        # progressing rather than stuck (mirrors real monitors' "acquiring"/
        # "no signal" banners)
        self._txt_pulse_status = self.ax_pulse.text(
            0.5, 0.5, "",
            ha="center", va="center", transform=self.ax_pulse.transAxes,
            fontsize=11, color=C["subtext"], alpha=0.9, zorder=5)

        # BPM trend ───────────────────────────────────────────────────────────
        self.ax_bpm = self.fig.add_subplot(gs[1, 0])
        self._style_ax(self.ax_bpm, "BPM Trend", ylabel="BPM")
        self._round_card(self.ax_bpm)
        self.ax_bpm.set_ylim(BPM_MIN, BPM_MAX)
        self.ax_bpm.grid(color="#262c40", linewidth=0.6, alpha=0.6)
        self._ln_bpm, = self.ax_bpm.plot(
            [], [], color=C["bpm_line"], lw=1.6, zorder=3,
            marker="o", markersize=3.2, markerfacecolor=C["bpm_line"],
            markeredgecolor=C["bpm_line"], markeredgewidth=0)
        self.ax_bpm.set_xlabel("time (s)", color=C["subtext"], fontsize=9)

        self._idle_text = self.ax_bpm.text(
            0.5, 0.5,
            "Press   ▶  START   to begin measurement",
            ha="center", va="center",
            transform=self.ax_bpm.transAxes,
            fontsize=11, color=C["subtext"], style="italic",
            bbox=dict(boxstyle="round,pad=0.5",
                      facecolor=C["panel"], edgecolor="none", alpha=0.88),
            zorder=4,
        )

        # numeric readout ─────────────────────────────────────────────────────
        self.ax_num = self.fig.add_subplot(gs[1, 1])
        self.ax_num.set_facecolor("none")
        self.ax_num.set_axis_off()
        self.ax_num.set_xlim(0, 1)
        self.ax_num.set_ylim(0, 1)

        self._txt_pulse = self.ax_num.text(
            0.5, 0.84, "♥",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=PULSE_MIN_SIZE, color=C["pulse"])
        self._txt_bpm = self.ax_num.text(
            0.5, 0.52, "---",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=44, fontweight="bold", color=C["bpm_value"],
            fontfamily="monospace")
        self.ax_num.text(
            0.5, 0.30, "BPM",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=13, color=C["subtext"])

        # confidence bar — fraction of the last CONFIDENCE_WINDOW readings
        # that stayed within CONFIDENCE_MAX_DEVIATION BPM of the trailing
        # trend, i.e. how stable the BPM trend has been recently
        self._txt_confidence = self.ax_num.text(
            0.5, 0.155, "confidence --",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=9, color=C["subtext"])
        self._bar_x0, self._bar_w = 0.18, 0.64
        bar_y, bar_h = 0.06, 0.035
        self.ax_num.add_patch(mpatches.Rectangle(
            (self._bar_x0, bar_y), self._bar_w, bar_h,
            transform=self.ax_num.transAxes,
            facecolor=C["bar_track"], edgecolor="none", zorder=1))
        self._bar_fill = mpatches.Rectangle(
            (self._bar_x0, bar_y), 0.0, bar_h,
            transform=self.ax_num.transAxes,
            facecolor=C["bpm_value"], edgecolor="none", zorder=2)
        self.ax_num.add_patch(self._bar_fill)

        # buttons ─────────────────────────────────────────────────────────────
        btn_w, btn_h = 0.15, 0.08
        gap          = 0.06
        cx           = 0.5
        y0           = 0.06

        ax_s = self.fig.add_axes([cx - btn_w - gap / 2, y0, btn_w, btn_h])
        ax_e = self.fig.add_axes([cx + gap / 2,         y0, btn_w, btn_h])

        self._btn_start = Button(ax_s, "▶  START",
                                 color=C["btn_start_on"],
                                 hovercolor=C["btn_start_hover"])
        self._btn_stop  = Button(ax_e, "■  STOP",
                                 color=C["btn_off"],
                                 hovercolor=C["btn_stop_on"])

        for btn, col in [(self._btn_start, C["btn_txt"]),
                         (self._btn_stop,  C["subtext"])]:
            btn.label.set_fontsize(11)
            btn.label.set_fontweight("bold")
            btn.label.set_color(col)

        self._btn_start.on_clicked(self._on_start)
        self._btn_stop.on_clicked(self._on_stop)

    @staticmethod
    def _style_ax(ax, title, ylabel=""):
        ax.set_facecolor("none")
        ax.set_title(title, color=C["text"], fontsize=11, pad=6)
        if ylabel:
            ax.set_ylabel(ylabel, color=C["subtext"], fontsize=9)
        ax.tick_params(colors=C["subtext"], labelsize=8)
        for sp in ax.spines.values():
            sp.set_color(C["panel"])

    @staticmethod
    def _round_card(ax, facecolor=None):
        """Draw a rounded 'card' background behind an axes, matplotlib's
        square facecolor otherwise looks flat/dated next to rounded buttons."""
        card = mpatches.FancyBboxPatch(
            (0.0, 0.0), 1.0, 1.0,
            boxstyle="round,pad=0.02,rounding_size=0.06",
            transform=ax.transAxes,
            facecolor=facecolor or C["panel"], edgecolor=C["panel_edge"],
            linewidth=1.0, zorder=0, clip_on=False,
        )
        ax.add_patch(card)

    # ── button callbacks ──────────────────────────────────────────────────────
    def _on_start(self, _event):
        if self._measuring:
            return
        self._measuring = True

        self._btn_start.color      = C["btn_off"]
        self._btn_start.hovercolor = C["btn_off"]
        self._btn_start.label.set_color(C["subtext"])
        self._btn_stop.color       = C["btn_stop_on"]
        self._btn_stop.hovercolor  = C["btn_stop_hover"]
        self._btn_stop.label.set_color(C["btn_txt"])

        self._idle_text.set_visible(False)
        self._reset_state()
        self._start_time = time.perf_counter()
        self.source.start()
        self.fig.canvas.draw_idle()
        print("[ppg] Measurement started")

    def _on_stop(self, _event):
        if not self._measuring:
            return
        self._measuring = False

        self.source.reset()
        self._reset_state()
        self._clear_plots()
        self._idle_text.set_visible(True)

        self._btn_start.color      = C["btn_start_on"]
        self._btn_start.hovercolor = C["btn_start_hover"]
        self._btn_start.label.set_color(C["btn_txt"])
        self._btn_stop.color       = C["btn_off"]
        self._btn_stop.hovercolor  = C["btn_stop_on"]
        self._btn_stop.label.set_color(C["subtext"])

        self.fig.canvas.draw_idle()
        print("[ppg] Measurement stopped and reset")

    def _clear_plots(self):
        self._ln_bpm.set_data([], [])
        self.ax_bpm.set_ylim(BPM_MIN, BPM_MAX)
        self.ax_bpm.set_xlim(0, 10)

        for ln in self._ln_pulse_layers:
            ln.set_data([], [])
        for dot in self._dot_layers:
            dot.set_visible(False)
        self.ax_pulse.set_xlim(0, PULSE_WINDOW_SEC)
        self._txt_pulse_status.set_text("")

        self._txt_bpm.set_text("---")
        self._txt_bpm.set_color(C["subtext"])
        self._txt_pulse.set_fontsize(PULSE_MIN_SIZE)
        self._txt_pulse.set_alpha(0.25)
        self._txt_pulse.set_color(C["subtext"])
        self._bar_fill.set_width(0.0)
        self._txt_confidence.set_text("confidence --")
        self._txt_confidence.set_color(C["subtext"])

    # ── animation ─────────────────────────────────────────────────────────────
    def _update(self, _frame):
        if not self._measuring:
            return

        for raw_bpm in self.source.drain():
            self._process_bpm(raw_bpm)

        # BPM trend
        if self._bpm_values:
            xs = list(self._bpm_times)
            ys = list(self._bpm_values)
            self._ln_bpm.set_data(xs, ys)
            # BPM_HISTORY caps the deque, so once readings start aging out
            # the left edge must track xs[0] — pinning it at 0 would leave a
            # growing blank gap before the oldest surviving point.
            self.ax_bpm.set_xlim(xs[0], max(xs[-1], xs[0] + 10))
            self.ax_bpm.set_ylim(max(BPM_MIN, min(ys) - 10),
                                 min(BPM_MAX, max(ys) + 10))

        locked = self._current_bpm is not None

        self._txt_pulse_status.set_text("" if locked else "waiting for signal…")

        # numeric readout
        if locked:
            self._txt_bpm.set_text(f"{self._current_bpm:.0f}")
            self._txt_bpm.set_color(C["bpm_value"])
        else:
            self._txt_bpm.set_text("--")
            self._txt_bpm.set_color(C["subtext"])

        # confidence bar — how stable the BPM trend has been recently
        confidence = self._confidence()
        if confidence is None:
            self._bar_fill.set_width(0.0)
            self._txt_confidence.set_text("confidence --")
            self._txt_confidence.set_color(C["subtext"])
        else:
            self._bar_fill.set_width(self._bar_w * confidence)
            self._bar_fill.set_facecolor(
                C["bpm_value"] if confidence >= 0.5 else C["confidence_low"])
            label = "low" if confidence < 0.5 else f"{confidence:.2f}"
            self._txt_confidence.set_text(f"confidence {label}")
            self._txt_confidence.set_color(
                C["text"] if confidence >= 0.5 else C["confidence_low"])

        # pulse — simulated at the current BPM rate, not detected per-beat
        # (the firmware only reports a periodic aggregate, not individual
        # beat timestamps). Only animates while `locked`, so no finger on
        # the sensor means no pulse shown.
        now = time.perf_counter()
        elapsed_full = now - self._start_time
        if self._last_frame_time is None:
            self._last_frame_time = now
        dt = now - self._last_frame_time
        self._last_frame_time = now

        win_start = max(0.0, elapsed_full - PULSE_WINDOW_SEC)
        win_end   = max(elapsed_full, PULSE_WINDOW_SEC)

        if locked:
            # A continuously accumulated phase (rather than recomputing
            # elapsed % period every frame) keeps the beat smooth even as
            # the BPM estimate drifts — recomputing from absolute elapsed
            # time made the phase snap discontinuously on every period
            # change, which is what made the waveform look jagged.
            rate = self._current_bpm / 60.0  # cycles/sec
            self._pulse_phase = (self._pulse_phase + dt * rate) % 1.0
            intensity = self._pulse_shape(self._pulse_phase)

            # The whole visible strip is regenerated analytically every
            # frame — rather than accreting one sample per animation tick —
            # so it renders at a fixed, dense resolution independent of
            # actual frame timing jitter. Earlier phases are reconstructed
            # by walking the current phase backwards at the current rate;
            # BPM changes slowly enough relative to the 4s window that this
            # is visually indistinguishable from tracking the true history.
            span = elapsed_full - win_start
            ts = [win_start + i * span / (PULSE_STRIP_POINTS - 1)
                  for i in range(PULSE_STRIP_POINTS)]
            vals = [self._pulse_shape((self._pulse_phase - (elapsed_full - t) * rate) % 1.0)
                    for t in ts]
        else:
            self._pulse_phase = 0.0  # next lock starts clean, on a rising edge
            intensity = 0.0
            ts, vals = [win_start, elapsed_full], [0.0, 0.0]

        size = PULSE_MIN_SIZE + (PULSE_MAX_SIZE - PULSE_MIN_SIZE) * intensity
        self._txt_pulse.set_fontsize(size)
        self._txt_pulse.set_alpha(0.55 + 0.45 * intensity if locked else 0.25)
        self._txt_pulse.set_color(C["pulse"] if locked else C["subtext"])

        for ln in self._ln_pulse_layers:
            ln.set_data(ts, vals)
        self.ax_pulse.set_xlim(win_start, win_end)

        # cursor dot at the trace's leading edge, only while locked
        for dot in self._dot_layers:
            dot.set_visible(locked)
            if locked:
                dot.set_offsets([[ts[-1], vals[-1]]])

    def run(self):
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Real-time PPG heart rate monitor (firmware-computed BPM)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--port", required=True, metavar="PORT",
                   help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    p.add_argument("--baud", type=int, default=1000000)
    return p.parse_args()


def main():
    args = parse_args()
    try:
        src = SerialSource(args.port, args.baud)
        print(f"[ppg] Ready on {args.port} @ {args.baud} baud  -- press START")
    except Exception as e:
        sys.exit(f"[ppg] Cannot open serial port: {e}")

    PPGMonitor(src).run()


if __name__ == "__main__":
    main()
