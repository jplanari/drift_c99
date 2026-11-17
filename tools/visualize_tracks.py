#!/usr/bin/env python3
"""visualize_tracks.py - Visualise hotspot trajectories and tracker output.

Host-side tool only (not part of the firmware). Reads:

  --truth  trajectory CSV written by golden/scenario_ref.cpp (or scenario_orig.cpp):
           frame,obj,x,y,vx_rel,vy_rel,vx_abs,vy_abs,mach,temp_K
  --tracks optional tracker CSV (e.g. golden_check / golden_gen output):
           frame,id,x,y,vx,vy        (vx,vy in px/frame, relative to the satellite)

Interactive mode shows the image plane (wrap-aware trails, relative velocity arrows,
tracker boxes with ids), a per-hotspot readout, temperature over time with the 1000 K
detection threshold, and the tracking error (distance from each tracker point to the
nearest true hotspot). A frame slider and a Play/Pause button drive playback.

Headless mode renders one frame to a PNG and exits:

  python3 tools/visualize_tracks.py --truth ref_scenario.csv --png frame_150.png --frame 150

Requires: numpy, matplotlib.
"""

import argparse
import sys

import numpy as np

if "--png" in sys.argv:
    # Headless rendering: select the non-interactive backend before pyplot is imported.
    import matplotlib
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.widgets import Button, Slider  # noqa: E402

TEMP_THRESH_K = 1000.0   # same threshold as main.cpp
ARROW_SCALE = 20.0       # same visual scale as main.cpp (velocity * 20)
WRAP_JUMP_FRAC = 0.5     # trail segments are split when a step exceeds this fraction of W/H


def load_truth(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.ndim == 0:
        data = data.reshape(1)
    return data


def load_tracks(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.ndim == 0:
        data = data.reshape(1)
    return data


def wrap_delta(d, size):
    """Shortest signed difference on a wrapped axis."""
    return d - size * np.round(d / size)


def tracking_error(trk_frame, truth_frame, W, H):
    """Distance from each tracker point to its nearest true hotspot (wrap-aware)."""
    if len(trk_frame) == 0 or len(truth_frame) == 0:
        return np.array([])
    dx = wrap_delta(trk_frame["x"][:, None] - truth_frame["x"][None, :], W)
    dy = wrap_delta(trk_frame["y"][:, None] - truth_frame["y"][None, :], H)
    return np.hypot(dx, dy).min(axis=1)


class Viewer:
    def __init__(self, truth, tracks, W, H, start_frame):
        self.truth = truth
        self.tracks = tracks
        self.W, self.H = W, H

        self.frames = np.unique(truth["frame"]).astype(int)
        self.n_frames = len(self.frames)
        self.objs = np.unique(truth["obj"]).astype(int)
        cmap = plt.get_cmap("tab10")
        self.colors = {int(o): cmap(i % 10) for i, o in enumerate(self.objs)}

        # Precompute tracker error per frame (NaN where there are no tracks).
        self.err_mean = np.full(self.n_frames, np.nan)
        self.err_max = np.full(self.n_frames, np.nan)
        self.n_tracks = np.zeros(self.n_frames, dtype=int)
        if tracks is not None:
            for i, f in enumerate(self.frames):
                trk = tracks[tracks["frame"].astype(int) == f]
                tru = truth[truth["frame"].astype(int) == f]
                self.n_tracks[i] = len(trk)
                err = tracking_error(trk, tru, W, H)
                if err.size:
                    self.err_mean[i] = err.mean()
                    self.err_max[i] = err.max()

        self.fig = plt.figure(figsize=(15, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[2.2, 1.0],
                                   left=0.05, right=0.98, top=0.95, bottom=0.14,
                                   hspace=0.45, wspace=0.12)
        self.ax_img = self.fig.add_subplot(gs[:, 0])
        self.ax_info = self.fig.add_subplot(gs[0, 1])
        self.ax_temp = self.fig.add_subplot(gs[1, 1])
        self.ax_err = self.fig.add_subplot(gs[2, 1])
        self.ax_info.axis("off")

        self._draw_static_timeseries()
        self._cursor_temp = self.ax_temp.axvline(self.frames[0], color="k", lw=1)
        self._cursor_err = self.ax_err.axvline(self.frames[0], color="k", lw=1)

        self.start_index = int(np.clip(np.searchsorted(self.frames, start_frame),
                                       0, self.n_frames - 1))
        self.slider = None
        self.playing = False
        self.timer = None
        self.button = None
        self._build_controls()
        self.set_index(self.start_index)

    # ------------------------------------------------------------ static panels
    def _draw_static_timeseries(self):
        for o in self.objs:
            rows = self.truth[self.truth["obj"].astype(int) == o]
            self.ax_temp.plot(rows["frame"], rows["temp_K"], color=self.colors[int(o)],
                              lw=1.2, label=f"obj {o}")
        self.ax_temp.axhline(TEMP_THRESH_K, color="gray", ls="--", lw=1)
        self.ax_temp.set_title("Temperatura (K)", fontsize=9)
        self.ax_temp.set_xlabel("frame", fontsize=8)
        self.ax_temp.tick_params(labelsize=7)
        self.ax_temp.legend(fontsize=7, loc="upper right")

        self.ax_err.set_title("Error de tracking (px, mitjana i màxim)", fontsize=9)
        self.ax_err.set_xlabel("frame", fontsize=8)
        self.ax_err.tick_params(labelsize=7)
        if self.tracks is None:
            self.ax_err.text(0.5, 0.5, "sense --tracks", ha="center", va="center",
                             transform=self.ax_err.transAxes, fontsize=9, color="gray")
        else:
            self.ax_err.plot(self.frames, self.err_mean, color="C0", lw=1.2, label="mitjana")
            self.ax_err.plot(self.frames, self.err_max, color="C3", lw=1, label="màxim")
            self.ax_err.legend(fontsize=7, loc="upper right")

    # ------------------------------------------------------------ image panel
    def _trail_segments(self, xs, ys):
        """Split a wrapped trajectory into segments so no line crosses the sensor."""
        segs, cur_x, cur_y = [], [xs[0]], [ys[0]]
        for i in range(1, len(xs)):
            if (abs(xs[i] - xs[i - 1]) > WRAP_JUMP_FRAC * self.W or
                    abs(ys[i] - ys[i - 1]) > WRAP_JUMP_FRAC * self.H):
                segs.append((cur_x, cur_y))
                cur_x, cur_y = [], []
            cur_x.append(xs[i])
            cur_y.append(ys[i])
        segs.append((cur_x, cur_y))
        return segs

    def _draw_image(self, frame):
        ax = self.ax_img
        ax.cla()
        ax.set_xlim(0, self.W)
        ax.set_ylim(self.H, 0)   # image coordinates: y grows downwards
        ax.set_aspect("equal")
        ax.set_title(f"Frame {frame}  (t = {frame / 30:.2f} s @ 30 fps)", fontsize=10)
        ax.grid(alpha=0.2)

        now = self.truth[self.truth["frame"].astype(int) == frame]
        for o in self.objs:
            o = int(o)
            col = self.colors[o]
            hist = self.truth[(self.truth["obj"].astype(int) == o) &
                              (self.truth["frame"].astype(int) <= frame)]
            for sx, sy in self._trail_segments(hist["x"], hist["y"]):
                ax.plot(sx, sy, color=col, lw=1, alpha=0.6)

            row = now[now["obj"].astype(int) == o]
            if len(row) == 0:
                continue
            r = row[0]
            ax.scatter([r["x"]], [r["y"]], s=90, color=col, edgecolors="k", zorder=5)
            ax.annotate(f"T{o}", (r["x"], r["y"]), xytext=(8, -10),
                        textcoords="offset points", fontsize=8, color=col, weight="bold")
            ax.arrow(r["x"], r["y"], ARROW_SCALE * r["vx_rel"], ARROW_SCALE * r["vy_rel"],
                     color=col, width=0.4, head_width=3, length_includes_head=True, zorder=4)

        if self.tracks is not None:
            trk = self.tracks[self.tracks["frame"].astype(int) == frame]
            if len(trk):
                ax.scatter(trk["x"], trk["y"], marker="s", s=120, facecolors="none",
                           edgecolors="k", linewidths=1.3, zorder=6)
                for t in trk:
                    ax.annotate(f"id{int(t['id'])}", (t["x"], t["y"]), xytext=(-30, 8),
                                textcoords="offset points", fontsize=7)

        ax.set_xlabel("x (px)", fontsize=8)
        ax.set_ylabel("y (px)", fontsize=8)
        ax.tick_params(labelsize=7)
        legend_text = f"Fletxa: velocitat relativa x{ARROW_SCALE:g} (px/frame)"
        if self.tracks is not None:
            legend_text += "    Quadrat: tracker"
        ax.text(0.01, 0.01, legend_text, transform=ax.transAxes, fontsize=7,
                color="dimgray", va="bottom")

    # ------------------------------------------------------------ info panel
    def _draw_info(self, frame):
        ax = self.ax_info
        ax.cla()
        ax.axis("off")
        lines = [f"frame {frame}   t = {frame / 30:.2f} s", ""]
        lines.append(f"{'obj':>3} {'x':>7} {'y':>7} {'vrel':>7} {'vabs':>7} {'Mach':>6} {'T(K)':>7}")
        now = self.truth[self.truth["frame"].astype(int) == frame]
        for o in self.objs:
            row = now[now["obj"].astype(int) == int(o)]
            if len(row) == 0:
                continue
            r = row[0]
            vrel = np.hypot(r["vx_rel"], r["vy_rel"])
            vabs = np.hypot(r["vx_abs"], r["vy_abs"])
            lines.append(f"{int(o):>3} {r['x']:7.1f} {r['y']:7.1f} {vrel:7.3f} "
                         f"{vabs:7.3f} {r['mach']:6.2f} {r['temp_K']:7.1f}")
        if self.tracks is not None:
            i = int(np.searchsorted(self.frames, frame))
            lines.append("")
            lines.append(f"tracks: {self.n_tracks[i]}   "
                         f"err mitjana: {self.err_mean[i]:.2f} px   "
                         f"màxim: {self.err_max[i]:.2f} px"
                         if self.n_tracks[i] else "tracks: 0")
        ax.text(0.0, 1.0, "\n".join(lines), transform=ax.transAxes, va="top",
                family="monospace", fontsize=8)

    # ------------------------------------------------------------ state
    def set_index(self, idx):
        idx = int(np.clip(idx, 0, self.n_frames - 1))
        self.index = idx
        frame = int(self.frames[idx])
        self._draw_image(frame)
        self._draw_info(frame)
        self._cursor_temp.set_xdata([frame, frame])
        self._cursor_err.set_xdata([frame, frame])
        if self.slider is not None and int(self.slider.val) != idx:
            self.slider.eventson = False
            self.slider.set_val(idx)
            self.slider.eventson = True
        self.fig.canvas.draw_idle()

    def _on_slider(self, val):
        self.set_index(int(val))

    def _tick(self):
        nxt = self.index + 1
        if nxt >= self.n_frames:
            nxt = 0
        self.set_index(nxt)

    def _toggle(self, _event):
        self.playing = not self.playing
        self.button.label.set_text("Pause" if self.playing else "Play")
        if self.playing:
            self.timer.start()
        else:
            self.timer.stop()

    def _build_controls(self):
        ax_slider = self.fig.add_axes([0.08, 0.05, 0.62, 0.03])
        self.slider = Slider(ax_slider, "frame", 0, self.n_frames - 1,
                             valinit=self.start_index, valstep=1)
        self.slider.on_changed(self._on_slider)

        ax_btn = self.fig.add_axes([0.80, 0.04, 0.08, 0.05])
        self.button = Button(ax_btn, "Play")
        self.button.on_clicked(self._toggle)

        self.timer = self.fig.canvas.new_timer(interval=60)
        self.timer.add_callback(self._tick)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--truth", required=True, help="trajectory CSV from scenario_ref / scenario_orig")
    p.add_argument("--tracks", help="optional tracker CSV: frame,id,x,y,vx,vy")
    p.add_argument("--width", type=int, default=640, help="sensor width in px (default 640)")
    p.add_argument("--height", type=int, default=480, help="sensor height in px (default 480)")
    p.add_argument("--frame", type=int, default=0, help="initial frame")
    p.add_argument("--png", metavar="FILE", help="save the frame --frame to FILE and exit")
    args = p.parse_args()

    truth = load_truth(args.truth)
    tracks = load_tracks(args.tracks) if args.tracks else None

    if args.png:
        viewer = Viewer(truth, tracks, args.width, args.height, args.frame)
        viewer.fig.savefig(args.png, dpi=110)
        print(f"wrote {args.png}")
        return

    viewer = Viewer(truth, tracks, args.width, args.height, args.frame)
    plt.show()


if __name__ == "__main__":
    main()
