"""
PPG Heart Rate Monitor
======================
Reads "BPM:xx" lines from the firmware over serial (the nRF52840 already
computes BPM on-device via FFT + Harmonic Product Spectrum — see
lib/xd58c/xd58c.c), rolling-averages the last ROLLING_WINDOW readings, and
displays:
  • BPM trend
  • Large BPM readout
  • START button  – begins listening and plotting
  • STOP button   – halts, resets all data and plots

Outlier rejection
------------------
A reading more than OUTLIER_MAX_JUMP BPM away from the current window's
average is held back for one reading (catches brief contact-transient/
signal-loss blips). If OUTLIER_RESET_STREAK consecutive readings disagree
with the baseline the same way, the window is cleared and reseeded instead —
the baseline is treated as stale (e.g. a real change, not a glitch), so it
doesn't reject forever.

Usage
-----
  python ppg_monitor.py --port /dev/ttyUSB0 --baud 1000000

Firmware assumptions
--------------------
  UART format: "BPM:xx\\r\\n" per HPS update (~every 1.28 s once warmed up).
  Non-BPM lines (e.g. stray "FFT:" CSV if CONFIG_XD58C_FFT_DEBUG is left on)
  are ignored.
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
from matplotlib.widgets import Button

# ── constants ────────────────────────────────────────────────────────────────
BPM_MIN         = 20
BPM_MAX         = 240
BPM_HISTORY     = 30   # readings kept for the trend plot
ROLLING_WINDOW  = 5    # most recent readings averaged for display
OUTLIER_MAX_JUMP     = 45  # BPM; bigger than one firmware bin (~23.4), smaller
                            # than contact-transient / signal-loss jumps
OUTLIER_RESET_STREAK = 2   # consecutive disagreements before the baseline is
                            # treated as stale and reseeded

PULSE_DEFAULT_BPM = 70   # pulse rate used before the first reading arrives
PULSE_MIN_SIZE    = 30   # heart glyph fontsize at rest
PULSE_MAX_SIZE    = 46   # heart glyph fontsize at the peak of each beat
PULSE_DECAY       = 6.0  # higher = sharper pop, faster fade within each beat

C = {
    "bpm_line":        "#ff5c7a",
    "pulse":           "#ff5c7a",
    "bg":              "#0b0d14",
    "panel":           "#181d2b",
    "panel_edge":      "#262c40",
    "text":            "#f0f2fa",
    "subtext":         "#8892aa",
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
            interval=40,  # smoother pulse animation; new BPM data still only
                          # arrives every ~1.28s regardless of this rate
            blit=False,
            cache_frame_data=False,
        )

    # ── state ─────────────────────────────────────────────────────────────────
    def _reset_state(self):
        self._bpm_times  = collections.deque(maxlen=BPM_HISTORY)
        self._bpm_values = collections.deque(maxlen=BPM_HISTORY)  # rolling-averaged
        self._window     = collections.deque(maxlen=ROLLING_WINDOW)  # raw readings
        self._reject_streak = 0
        self._current_bpm = None

    def _process_bpm(self, raw_bpm):
        """Receive one BPM reading from the firmware, reject transient
        outliers, rolling-average the last ROLLING_WINDOW readings, and store
        the result for display."""
        if self._window:
            baseline = statistics.mean(self._window)
            if abs(raw_bpm - baseline) > OUTLIER_MAX_JUMP:
                self._reject_streak += 1
                if self._reject_streak < OUTLIER_RESET_STREAK:
                    return  # one-off transient: reject, hold last displayed value
                # Multiple consecutive disagreements — the baseline is stale
                # (a real change), not the incoming readings. Reseed instead
                # of rejecting forever.
                self._window.clear()
                self._reject_streak = 0
            else:
                self._reject_streak = 0

        self._window.append(raw_bpm)
        avg = statistics.mean(self._window)

        elapsed = time.perf_counter() - self._start_time
        self._bpm_times.append(elapsed)
        self._bpm_values.append(avg)
        self._current_bpm = avg

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(10, 6), facecolor=C["bg"])
        self.fig.canvas.manager.set_window_title("PPG Heart Rate Monitor")

        gs = gridspec.GridSpec(
            1, 2,
            figure=self.fig,
            left=0.08, right=0.96,
            top=0.88,  bottom=0.22,
            wspace=0.3,
        )

        # BPM trend ───────────────────────────────────────────────────────────
        self.ax_bpm = self.fig.add_subplot(gs[0, 0])
        self._style_ax(self.ax_bpm, "BPM Trend", ylabel="BPM")
        self._round_card(self.ax_bpm)
        self.ax_bpm.set_ylim(BPM_MIN, BPM_MAX)
        self.ax_bpm.grid(color="#262c40", linewidth=0.6, alpha=0.6)
        self._ln_bpm, = self.ax_bpm.plot(
            [], [], color=C["bpm_line"], lw=2.2, zorder=3,
            marker="o", markersize=4.5, markerfacecolor=C["bpm_line"],
            markeredgecolor=C["bg"], markeredgewidth=0.6)
        self._fill_bpm = None  # created/updated per-frame (fill_between has no set_data)
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
        self.ax_num = self.fig.add_subplot(gs[0, 1])
        self.ax_num.set_facecolor("none")
        self.ax_num.set_axis_off()
        self._round_card(self.ax_num)

        self._txt_pulse = self.ax_num.text(
            0.5, 0.86, "♥",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=PULSE_MIN_SIZE, color=C["pulse"])
        self._txt_bpm = self.ax_num.text(
            0.5, 0.52, "---",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=64, fontweight="bold", color=C["text"],
            fontfamily="monospace")
        self.ax_num.text(
            0.5, 0.24, "BPM",
            ha="center", va="center", transform=self.ax_num.transAxes,
            fontsize=15, color=C["subtext"])

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
    def _round_card(ax):
        """Draw a rounded 'card' background behind an axes, matplotlib's
        square facecolor otherwise looks flat/dated next to rounded buttons."""
        card = mpatches.FancyBboxPatch(
            (0.0, 0.0), 1.0, 1.0,
            boxstyle="round,pad=0.02,rounding_size=0.06",
            transform=ax.transAxes,
            facecolor=C["panel"], edgecolor=C["panel_edge"], linewidth=1.0,
            zorder=0, clip_on=False,
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
        if self._fill_bpm is not None:
            self._fill_bpm.remove()
            self._fill_bpm = None

        self._txt_bpm.set_text("---")
        self._txt_bpm.set_color(C["text"])
        self._txt_pulse.set_fontsize(PULSE_MIN_SIZE)
        self._txt_pulse.set_alpha(0.5)

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
            self.ax_bpm.set_xlim(0, max(xs[-1], 10))
            self.ax_bpm.set_ylim(max(BPM_MIN, min(ys) - 10),
                                 min(BPM_MAX, max(ys) + 10))
            if self._fill_bpm is not None:
                self._fill_bpm.remove()
            self._fill_bpm = self.ax_bpm.fill_between(
                xs, ys, min(BPM_MIN, min(ys)),
                color=C["bpm_line"], alpha=0.12, zorder=2, linewidth=0)

        # numeric readout
        if self._current_bpm is not None:
            self._txt_bpm.set_text(f"{self._current_bpm:.0f}")

        # pulsing heart — simulated at the current BPM rate, not detected
        # per-beat (the firmware only reports a periodic aggregate, not
        # individual beat timestamps)
        pulse_bpm = self._current_bpm or PULSE_DEFAULT_BPM
        period = 60.0 / pulse_bpm
        t_in_cycle = (time.perf_counter() - self._start_time) % period
        intensity = math.exp(-PULSE_DECAY * t_in_cycle / period)
        size = PULSE_MIN_SIZE + (PULSE_MAX_SIZE - PULSE_MIN_SIZE) * intensity
        self._txt_pulse.set_fontsize(size)
        self._txt_pulse.set_alpha(0.55 + 0.45 * intensity)

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
