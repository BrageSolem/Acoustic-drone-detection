import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch
import matplotlib.colors as mcolors

# ── Data ──────────────────────────────────────────────────────────────────────
data = {
    "time": [
        1778753055.167, 1778753064.266, 1778753067.396, 1778753071.193,
        1778753074.442, 1778753078.201, 1778753081.495, 1778753085.077,
        1778753087.993, 1778753091.664, 1778753095.088, 1778753097.986,
        1778753101.658, 1778753104.941, 1778753107.752, 1778753110.984,
        1778753114.114, 1778753117.155, 1778753120.486, 1778753123.373,
        1778753126.689, 1778753130.110, 1778753132.973, 1778753135.843,
        1778753139.045, 1778753142.283, 1778753145.647, 1778753148.505,
        1778753151.401, 1778753154.850,
    ],
    "azimuth": [
        44.1, -3.6, -37.0, -102.1, -140.2, 157.0, 97.0, 29.1,
        -22.8, -93.7, -146.0, 162.9, 92.5, 30.1, -5.9, -38.8,
        -123.9, -150.4, 135.6, 99.3, 37.7, -6.7, -31.5, -54.5,
        -107.4, -142.3, 169.3, 120.4, 94.3, 16.2,
    ],
}

df = pd.DataFrame(data)
df["t_rel"] = df["time"] - df["time"].iloc[0]   # seconds from start
n = len(df)

# ── Colour map: early = light blue, late = dark blue ─────────────────────────
cmap   = plt.cm.Blues
norm   = mcolors.Normalize(vmin=0, vmax=n - 1)
colors = [cmap(norm(i)) for i in range(n)]


# ══════════════════════════════════════════════════════════════════════════════
# 1.  ANIMATED VIDEO  (drone_azimuth_animation.mp4)
# ══════════════════════════════════════════════════════════════════════════════
fig_anim, axes = plt.subplots(1, 2, figsize=(13, 6))
fig_anim.patch.set_facecolor("#0f0f0f")

# ── Left: compass ─────────────────────────────────────────────────────────────
ax_c = axes[0]
ax_c.set_facecolor("#0f0f0f")
ax_c.set_xlim(-1.35, 1.35)
ax_c.set_ylim(-1.35, 1.35)
ax_c.set_aspect("equal")
ax_c.axis("off")

# Compass rings
for r in [0.4, 0.7, 1.0]:
    circle = plt.Circle((0, 0), r, color="white", fill=False,
                         linewidth=0.5, alpha=0.15)
    ax_c.add_patch(circle)

# Tick marks and cardinal labels
cardinals = {"N": 0, "NE": 45, "E": 90, "SE": 135,
             "S": 180, "SW": 225, "W": 270, "NW": 315}
for label, deg in cardinals.items():
    rad = np.radians(deg)
    x, y = np.sin(rad), np.cos(rad)
    ax_c.text(x * 1.22, y * 1.22, label,
              ha="center", va="center", fontsize=10,
              color="#E24B4A" if label == "N" else "white",
              fontweight="bold" if label == "N" else "normal")
    ax_c.plot([x * 0.97, x * 1.05], [y * 0.97, y * 1.05],
              color="white", linewidth=0.8, alpha=0.4)

for deg in range(0, 360, 10):
    rad = np.radians(deg)
    x, y = np.sin(rad), np.cos(rad)
    ax_c.plot([x * 0.98, x * 1.0], [y * 0.98, y * 1.0],
              color="white", linewidth=0.4, alpha=0.25)

# Platform dot
ax_c.plot(0, 0, "o", color="white", markersize=6, zorder=5)

# Arrow (will be updated)
arrow_line, = ax_c.plot([], [], color="#4A90D9", linewidth=2.5, zorder=4)
arrow_head, = ax_c.plot([], [], "^", color="#4A90D9", markersize=10, zorder=4)
drone_dot,  = ax_c.plot([], [], "o", color="#E87040", markersize=8, zorder=5)

deg_text = ax_c.text(0, -1.28, "", ha="center", va="center",
                     fontsize=22, fontweight="bold", color="white")
time_text = ax_c.text(0, 1.28, "", ha="center", va="center",
                      fontsize=11, color="#aaaaaa")
sample_text = ax_c.text(0, -1.18, "", ha="center", va="center",
                        fontsize=10, color="#aaaaaa")
ax_c.set_title("Compass", color="white", fontsize=13, pad=8)

# ── Right: azimuth over time ──────────────────────────────────────────────────
ax_t = axes[1]
ax_t.set_facecolor("#0f0f0f")
ax_t.tick_params(colors="white")
for spine in ax_t.spines.values():
    spine.set_edgecolor("#444")

ax_t.plot(df["t_rel"], df["azimuth"], color="#4A90D9",
          linewidth=1.2, alpha=0.5, zorder=1)
ax_t.axhline(0, color="#555", linewidth=0.5, linestyle="--")
ax_t.set_xlabel("Time (s)", color="white", fontsize=11)
ax_t.set_ylabel("Azimuth (°)", color="white", fontsize=11)
ax_t.set_ylim(-200, 200)
ax_t.set_xlim(df["t_rel"].min() - 2, df["t_rel"].max() + 2)
ax_t.set_title("Azimuth over time", color="white", fontsize=13)
ax_t.grid(True, color="#222", linewidth=0.5)

current_dot, = ax_t.plot([], [], "o", color="#E87040", markersize=8, zorder=5)
current_vline = ax_t.axvline(x=0, color="#E87040",
                              linewidth=1.2, linestyle="--", alpha=0.5)

plt.tight_layout(pad=2)

def update(frame):
    az  = df["azimuth"].iloc[frame]
    t   = df["t_rel"].iloc[frame]
    rad = np.radians(az)
    ex, ey = np.sin(rad) * 0.88, np.cos(rad) * 0.88

    arrow_line.set_data([0, ex * 0.82], [0, ey * 0.82])

    # Rotate triangle marker to point in arrow direction
    arrow_head.set_data([ex], [ey])
    arrow_head.set_marker((3, 0, az - 90))   # rotated triangle

    drone_dot.set_data([ex], [ey])

    sign = "+" if az >= 0 else ""
    deg_text.set_text(f"{sign}{az:.1f}°")
    time_text.set_text(f"t = {t:.1f} s")
    sample_text.set_text(f"sample {frame + 1} / {n}")

    current_dot.set_data([t], [az])
    current_vline.set_xdata([t, t])

    return (arrow_line, arrow_head, drone_dot,
            deg_text, time_text, sample_text,
            current_dot, current_vline)

ani = animation.FuncAnimation(
    fig_anim, update, frames=n,
    interval=600,    # ms between frames (≈ real time sped up 5×)
    blit=True, repeat=True
)

writer = animation.PillowWriter(fps=2)
ani.save("drone_azimuth_animation.gif", writer=writer)
print("Saved: drone_azimuth_animation.gif")
plt.close(fig_anim)


# ══════════════════════════════════════════════════════════════════════════════
# 2.  STATIC TRAIL PLOT  (drone_azimuth_trail.png)
# ══════════════════════════════════════════════════════════════════════════════
fig_trail, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(8, 8))
fig_trail.patch.set_facecolor("#0f0f0f")
ax.set_facecolor("#0f0f0f")

# Polar plot: theta = azimuth in radians, r = 1 (fixed distance)
# Matplotlib polar: 0 = right (East), we want 0 = top (North)
# So theta = pi/2 - az_rad, and set theta_zero_location="N"
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)   # clockwise = positive azimuth

radii = np.ones(n)
thetas = np.radians(df["azimuth"].values)

# Draw trail line (faded)
ax.plot(thetas, radii, color="#4A90D9", linewidth=0.8,
        alpha=0.25, linestyle="--", zorder=1)

# Draw arrows from origin to each point
for i in range(n):
    th = thetas[i]
    c  = colors[i]
    ax.annotate(
        "",
        xy=(th, 0.95), xytext=(th, 0.0),
        arrowprops=dict(arrowstyle="-|>", color=c,
                        lw=1.8, mutation_scale=14),
        zorder=3,
    )

# Draw dots at tip
sc = ax.scatter(thetas, radii * 0.95, c=range(n),
                cmap="Blues", norm=norm,
                s=50, zorder=4, edgecolors="white", linewidths=0.4)

# Labels: show time for every 3rd sample to avoid clutter
for i in range(0, n, 3):
    th  = thetas[i]
    az  = df["azimuth"].iloc[i]
    t   = df["t_rel"].iloc[i]
    sign = "+" if az >= 0 else ""
    ax.text(th, 1.10,
            f"{t:.0f}s\n{sign}{az:.0f}°",
            ha="center", va="center",
            fontsize=8, color="white", alpha=0.85)

# Styling
ax.set_rticks([])
ax.set_rlim(0, 1.25)
ax.tick_params(colors="white", labelsize=10)
ax.set_facecolor("#0f0f0f")
for spine in ax.spines.values():
    spine.set_visible(False)
ax.grid(color="#333", linewidth=0.5)
ax.set_xticklabels(["N", "NE", "E", "SE", "S", "SW", "W", "NW"],
                   color="white", fontsize=10)

# Colourbar
sm = plt.cm.ScalarMappable(cmap="Blues", norm=norm)
sm.set_array([])
cbar = fig_trail.colorbar(sm, ax=ax, pad=0.08, fraction=0.03)
cbar.set_label("Sample index (early → late)", color="white", fontsize=10)
cbar.ax.yaxis.set_tick_params(color="white")
plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")

ax.set_title("Drone azimuth trail\n(all 30 samples, colour = time)",
             color="white", fontsize=13, pad=20)

# Platform label at centre
ax.text(0, 0, "platform", ha="center", va="center",
        fontsize=8, color="#aaa",
        bbox=dict(boxstyle="round,pad=0.3", fc="#0f0f0f", ec="#555", lw=0.5))

plt.tight_layout()
plt.savefig("drone_azimuth_trail.png", dpi=300,
            bbox_inches="tight", facecolor="#0f0f0f")
print("Saved: drone_azimuth_trail.png")
plt.close(fig_trail)

print("\nDone. Both files saved.")