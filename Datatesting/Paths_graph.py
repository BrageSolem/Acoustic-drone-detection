#!/usr/bin/env python3
"""
DOA Validation Plot — Bachelor Thesis
Overlays drone EKF flight path with azimuth direction-of-arrival estimates.

Requirements:
    pip install pymavlink pandas numpy matplotlib

Usage:
    python doa_validation_plot.py

Edit the paths at the top if needed.
"""

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from pymavlink import DFReader

# ─── Paths ────────────────────────────────────────────────────────────────────
LOG_PATH      = "Test_Data/DOA test/log_3.bin"
AZIMUTH_PATH  = "Test_Data/DOA test/azimuth_timestamps.csv"
OUTPUT_PATH   = "doa_validation.pdf"   # also saves a .png alongside

# ─── Platform position (room coords: X=north, Y=east) ────────────────────────
PLATFORM_X = 3.5   # metres north of SW corner
PLATFORM_Y = 3   # metres east  of SW corner  (approximate)

# ─── Sync parameters (derived from arming event) ─────────────────────────────
AZ_UNIX_START  = 1778753055.1670084   # Unix time of first azimuth sample
LOG_ARMED_T    = 313.639304           # log time (s) of arming event
BOOT_UNIX      = AZ_UNIX_START - LOG_ARMED_T

# ─── Distance threshold for "reliable" bearing estimates ─────────────────────
RELIABLE_DIST_M = 2.0

# ══════════════════════════════════════════════════════════════════════════════
#  Load data
# ══════════════════════════════════════════════════════════════════════════════

print("Loading flight log …")
log = DFReader.DFReader_binary(LOG_PATH)
xkf1 = []
while True:
    msg = log.recv_msg()
    if msg is None:
        break
    if msg.get_type() == "XKF1":
        xkf1.append({"log_t": msg._timestamp, "PN": msg.PN, "PE": msg.PE, "PD": msg.PD})

pos_df = (pd.DataFrame(xkf1)
            .drop_duplicates(subset="log_t")
            .reset_index(drop=True))
pos_df["unix_t"] = BOOT_UNIX + pos_df["log_t"]

print("Loading azimuth CSV …")
az = pd.read_csv(AZIMUTH_PATH)
t0 = az["Time"].values[0]
az["t_rel"] = az["Time"].values - t0

# ─── Interpolate drone XY at each azimuth timestamp ──────────────────────────
az["drone_x"] = np.interp(az["Time"].values, pos_df["unix_t"].values, pos_df["PN"].values)
az["drone_y"] = np.interp(az["Time"].values, pos_df["unix_t"].values, pos_df["PE"].values)

# ─── Flight path during azimuth window ───────────────────────────────────────
az_log_t_start = az["Time"].min() - BOOT_UNIX
az_log_t_end   = az["Time"].max() - BOOT_UNIX
flight = pos_df[(pos_df["log_t"] >= az_log_t_start) &
                (pos_df["log_t"] <= az_log_t_end)].copy()
flight["t_rel"] = flight["log_t"] - az_log_t_start

# ─── Compute predicted bearing and error for each sample ─────────────────────
def wrap(d):
    return (d + 180) % 360 - 180

dx = az["drone_x"].values - PLATFORM_X
dy = az["drone_y"].values - PLATFORM_Y
az["dist"]      = np.sqrt(dx**2 + dy**2)
az["predicted"] = np.degrees(np.arctan2(-dy, dx))   # 0=N, neg=CW(east), pos=CCW(west)
az["error"]     = np.array([wrap(a - p)
                             for a, p in zip(az["Azimuth"].values, az["predicted"].values)])
az["reliable"]  = az["dist"] >= RELIABLE_DIST_M

# ─── Azimuth arrow endpoints (from platform) ─────────────────────────────────
ARROW_LEN = 1.2   # metres — visual length of DOA arrow
az["arrow_dx"] = ARROW_LEN * np.cos(np.radians(az["Azimuth"].values))   # convention: 0=N=+X
az["arrow_dy"] = ARROW_LEN * -np.sin(np.radians(az["Azimuth"].values))  # mirror Y

# ── Predicted arrow endpoints ─────────────────────────────────────────────────
az["pred_dx"] = ARROW_LEN * np.cos(np.radians(az["predicted"].values))
az["pred_dy"] = ARROW_LEN * -np.sin(np.radians(az["predicted"].values))

# ══════════════════════════════════════════════════════════════════════════════
#  Figure
# ══════════════════════════════════════════════════════════════════════════════

plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["DejaVu Serif"],
    "axes.spines.top":  False,
    "axes.spines.right":False,
    "figure.dpi":       150,
})

fig = plt.figure(figsize=(14, 7))
fig.patch.set_facecolor("#F7F6F2")

# ── Two panels: floor plan left, error plot right ─────────────────────────────
gs   = fig.add_gridspec(1, 2, width_ratios=[1.4, 1], wspace=0.08,
                         left=0.06, right=0.97, top=0.91, bottom=0.10)
ax1  = fig.add_subplot(gs[0])
ax2  = fig.add_subplot(gs[1])

for ax in (ax1, ax2):
    ax.set_facecolor("#F7F6F2")

# ════════════════════════════════════════════════════════════════════════════
#  LEFT PANEL: top-down floor plan
# ════════════════════════════════════════════════════════════════════════════

# ── Coloured flight path by time ─────────────────────────────────────────────
points  = np.array([flight["PN"].values, flight["PE"].values]).T.reshape(-1, 1, 2)
segs    = np.concatenate([points[:-1], points[1:]], axis=1)
norm    = Normalize(vmin=0, vmax=flight["t_rel"].max())
lc      = LineCollection(segs, cmap="plasma", norm=norm, linewidth=1.8,
                          alpha=0.75, zorder=2)
lc.set_array(flight["t_rel"].values[:-1])
ax1.add_collection(lc)

# ── DOA arrows from platform ──────────────────────────────────────────────────
for _, row in az.iterrows():
    color  = "#E05C2A" if row["reliable"] else "#AAAAAA"
    alpha  = 0.9      if row["reliable"] else 0.4
    zorder = 5        if row["reliable"] else 3
    lw     = 1.6      if row["reliable"] else 1.0

    # Measured DOA arrow
    ax1.annotate(
        "", xy=(PLATFORM_Y + row["arrow_dy"], PLATFORM_X + row["arrow_dx"]),
        xytext=(PLATFORM_Y, PLATFORM_X),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, alpha=alpha),
        zorder=zorder,
    )

    # Predicted (ground truth) arrow — only for reliable samples
    if row["reliable"]:
        ax1.annotate(
            "", xy=(PLATFORM_Y + row["pred_dy"], PLATFORM_X + row["pred_dx"]),
            xytext=(PLATFORM_Y, PLATFORM_X),
            arrowprops=dict(arrowstyle="-|>", color="#2A7BE0", lw=1.4,
                            alpha=0.75, linestyle="dashed"),
            zorder=4,
        )

# ── Drone positions at each sample ───────────────────────────────────────────
sc = ax1.scatter(az["drone_y"], az["drone_x"], c=az["t_rel"], cmap="plasma",
                 norm=norm, s=40, zorder=6, edgecolors="white", linewidths=0.5)

# Label a few sample numbers
for i, row in az.iterrows():
    if i % 5 == 0:
        ax1.text(row["drone_y"] + 0.06, row["drone_x"] + 0.06,
                 str(i + 1), fontsize=6.5, color="#444", zorder=7)

# ── Platform marker ───────────────────────────────────────────────────────────
ax1.scatter([PLATFORM_Y], [PLATFORM_X], s=160, marker="*",
            color="#1A1A2E", zorder=8, label="Platform (mic array)")
ax1.text(PLATFORM_Y + 0.12, PLATFORM_X - 0.15, "Platform",
         fontsize=8, fontweight="bold", color="#1A1A2E")

# ── Room boundary (approx from flight data) ───────────────────────────────────
room_x = [0, 0, 5.0, 5.0, 0]
room_y = [0, 6.0, 6.0, 0, 0]
ax1.plot(room_y, room_x, color="#BBBBBB", lw=1.2, ls="--", zorder=1, label="Room boundary (approx)")

# ── Compass rose (small, top-right corner) ────────────────────────────────────
cx, cy, cr = 5.5, 4.8, 0.25
for label, dx_r, dy_r in [("N", cr, 0), ("S", -cr, 0), ("E", 0, cr), ("W", 0, -cr)]:
    ax1.annotate("", xy=(cy + dy_r, cx + dx_r), xytext=(cy, cx),
                 arrowprops=dict(arrowstyle="-|>", color="#888", lw=1.0))
    ax1.text(cy + dy_r * 1.5, cx + dx_r * 1.5, label,
             ha="center", va="center", fontsize=7, color="#666")

# ── Colorbar ──────────────────────────────────────────────────────────────────
cbar = fig.colorbar(sc, ax=ax1, fraction=0.03, pad=0.02)
cbar.set_label("Time into flight (s)", fontsize=8)
cbar.ax.tick_params(labelsize=7)

# ── Legend ────────────────────────────────────────────────────────────────────
legend_elements = [
    mpatches.Patch(color="#E05C2A", label=f"DOA estimate (dist ≥ {RELIABLE_DIST_M}m)"),
    mpatches.Patch(color="#AAAAAA", label=f"DOA estimate (dist < {RELIABLE_DIST_M}m, unreliable)"),
    mpatches.Patch(color="#2A7BE0", label="Predicted bearing from EKF (reliable only)"),
    plt.Line2D([0], [0], color="purple", lw=1.8, alpha=0.75, label="Drone flight path"),
]
ax1.legend(handles=legend_elements, fontsize=7.5, loc="lower right",
           framealpha=0.85, edgecolor="#CCC")

ax1.set_xlabel("East →  (m)", fontsize=9)
ax1.set_ylabel("North →  (m)", fontsize=9)
ax1.set_title("Top-down view: flight path & DOA estimates", fontsize=10, fontweight="bold", pad=8)
ax1.set_xlim(-0.3, 6.3)
ax1.set_ylim(-0.3, 5.5)
ax1.set_aspect("equal")
ax1.tick_params(labelsize=8)
ax1.grid(True, color="#E0E0E0", lw=0.6, zorder=0)

# ════════════════════════════════════════════════════════════════════════════
#  RIGHT PANEL: bearing error vs time, coloured by distance
# ════════════════════════════════════════════════════════════════════════════

colors_err = ["#E05C2A" if r else "#BBBBBB" for r in az["reliable"]]

ax2.axhline(0, color="#999", lw=0.8, ls="--", zorder=1)
ax2.axhspan(-30, 30, color="#2A7BE0", alpha=0.07, zorder=0, label="±30° band")

for i, row in az.iterrows():
    c = "#E05C2A" if row["reliable"] else "#BBBBBB"
    ax2.plot(row["t_rel"], row["error"], "o", color=c,
             markersize=6 if row["reliable"] else 4,
             zorder=4 if row["reliable"] else 2,
             markeredgecolor="white", markeredgewidth=0.5)

# Connect dots with a faint line
ax2.plot(az["t_rel"], az["error"], color="#CCCCCC", lw=0.8, zorder=1)

# Annotate reliable samples with error value
for _, row in az[az["reliable"]].iterrows():
    ax2.text(row["t_rel"] + 0.8, row["error"] + 4,
             f'{row["error"]:+.0f}°', fontsize=7, color="#C04010")

# Distance on secondary y-axis
ax2b = ax2.twinx()
ax2b.plot(az["t_rel"], az["dist"], color="#1A7A4A", lw=1.4,
          ls=":", alpha=0.7, label="Drone–platform dist")
ax2b.axhline(RELIABLE_DIST_M, color="#1A7A4A", lw=0.8, ls="--", alpha=0.5)
ax2b.set_ylabel("Drone–platform distance (m)", fontsize=8.5, color="#1A7A4A")
ax2b.tick_params(labelsize=7.5, colors="#1A7A4A")
ax2b.set_ylim(0, 6)

# Stats box for reliable samples only
rel = az[az["reliable"]]
if len(rel) > 0:
    stats_txt = (f"Reliable samples (dist ≥ {RELIABLE_DIST_M}m):  n={len(rel)}\n"
                 f"Mean error = {rel['error'].mean():+.1f}°\n"
                 f"Std  = {rel['error'].std():.1f}°\n"
                 f"RMSE = {np.sqrt((rel['error']**2).mean()):.1f}°")
    ax2.text(0.97, 0.97, stats_txt, transform=ax2.transAxes,
             fontsize=7.5, va="top", ha="right",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                       edgecolor="#CCC", alpha=0.9))

ax2.set_xlabel("Time into flight (s)", fontsize=9)
ax2.set_ylabel("Bearing error: measured − predicted (°)", fontsize=8.5)
ax2.set_title("DOA bearing error over time", fontsize=10, fontweight="bold", pad=8)
ax2.set_xlim(-2, 102)
ax2.set_ylim(-200, 200)
ax2.tick_params(labelsize=8)
ax2.grid(True, color="#E0E0E0", lw=0.6, zorder=0)

legend_err = [
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#E05C2A",
               markersize=7, label=f"Reliable (dist ≥ {RELIABLE_DIST_M}m)"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#BBBBBB",
               markersize=5, label=f"Unreliable (dist < {RELIABLE_DIST_M}m)"),
    mpatches.Patch(color="#2A7BE0", alpha=0.2, label="±30° band"),
    plt.Line2D([0], [0], color="#1A7A4A", lw=1.4, ls=":", label="Drone–platform dist"),
]
ax2.legend(handles=legend_err, fontsize=7.5, loc="lower left",
           framealpha=0.85, edgecolor="#CCC")

# ── Overall title ──────────────────────────────────────────────────────────────
fig.suptitle(
    "Direction-of-Arrival Estimation vs. EKF Ground Truth",
    fontsize=13, fontweight="bold", y=0.97, color="#1A1A2E"
)
fig.text(0.5, 0.01,
         f"Platform position: ({PLATFORM_X}m N, {PLATFORM_Y}m E) ± ~0.5m  |  "
         f"Sync: arming event (±2s)  |  EKF position accuracy: 10–20 cm",
         ha="center", fontsize=7.5, color="#888")

# ── Save ───────────────────────────────────────────────────────────────────────
plt.savefig(OUTPUT_PATH, bbox_inches="tight", facecolor=fig.get_facecolor())
png_path = OUTPUT_PATH.replace(".pdf", ".png")
plt.savefig(png_path, bbox_inches="tight", facecolor=fig.get_facecolor(), dpi=180)
print(f"Saved: {OUTPUT_PATH}")
print(f"Saved: {png_path}")
plt.show()