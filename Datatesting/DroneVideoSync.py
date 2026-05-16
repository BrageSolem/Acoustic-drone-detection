#!/usr/bin/env python3
"""
Drone DOA Sync Player
Plays a video and azimuth compass side by side in sync.

Controls:
    SPACE      — pause / resume
    Q / ESC    — quit
    A / LEFT   — rewind 5 seconds
    D / RIGHT  — skip forward 5 seconds
    + / =      — shift DOA forward 0.5s (drone data starts later)
    -          — shift DOA backward 0.5s (drone data starts earlier)
"""
import datetime
import cv2
import numpy as np
import time

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURE THESE PATHS
# ══════════════════════════════════════════════════════════════════════════════

VIDEO_PATH = r"Test_Data/DOA test/DOA_video.mp4"   # ← change this

# Offset in seconds: positive = DOA data starts this many seconds INTO the video
# Use +/- keys while running to fine-tune, then set it here once you know the value
SYNC_OFFSET = 12

# ══════════════════════════════════════════════════════════════════════════════
#  AZIMUTH DATA
# ══════════════════════════════════════════════════════════════════════════════

AZIMUTH_TIMES = [
    0.0,   9.1,  12.2,  16.0,  19.3,  23.0,  26.3,  29.9,
    32.8,  36.5,  39.9,  42.8,  46.5,  49.8,  52.6,  55.8,
    59.0,  62.0,  65.3,  68.2,  71.5,  75.0,  77.8,  80.7,
    84.0,  87.1,  90.5,  93.3,  96.2,  99.7,
]

AZIMUTH_VALUES = [
    44.1,  -3.6, -37.0, -102.1, -140.2, 157.0,  97.0,  29.1,
   -22.8, -93.7, -146.0, 162.9,  92.5,  30.1,  -5.9, -38.8,
  -123.9, -150.4, 135.6,  99.3,  37.7,  -6.7, -31.5, -54.5,
  -107.4, -142.3, 169.3, 120.4,  94.3,  16.2,
]

DATA_DURATION = AZIMUTH_TIMES[-1]   # seconds
N_SAMPLES     = len(AZIMUTH_VALUES)

# ══════════════════════════════════════════════════════════════════════════════
#  DISPLAY SETTINGS
# ══════════════════════════════════════════════════════════════════════════════

DISPLAY_HEIGHT = 1080   # ↑ from 540 — full HD output
COMPASS_SIZE   = 900    # ↑ from 480 — scale compass to match


# ─────────────────────────────────────────────────────────────────────────────

def get_az_at_time(t_rel):
    """Linearly interpolate azimuth at time t_rel (seconds from data start)."""
    if t_rel <= AZIMUTH_TIMES[0]:
        return AZIMUTH_VALUES[0], 0
    if t_rel >= AZIMUTH_TIMES[-1]:
        return AZIMUTH_VALUES[-1], N_SAMPLES - 1
    for i in range(len(AZIMUTH_TIMES) - 1):
        if AZIMUTH_TIMES[i] <= t_rel <= AZIMUTH_TIMES[i + 1]:
            frac = ((t_rel - AZIMUTH_TIMES[i]) /
                    (AZIMUTH_TIMES[i + 1] - AZIMUTH_TIMES[i]))
            a1 = AZIMUTH_VALUES[i]
            a2 = AZIMUTH_VALUES[i + 1]
            # FIX: take the shortest path across the ±180° boundary
            diff = a2 - a1
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            az = a1 + frac * diff
            return az, i
    return AZIMUTH_VALUES[-1], N_SAMPLES - 1


def draw_compass(az_deg, t_rel, sample_idx):
    """Draw a compass frame showing the current azimuth."""
    S  = COMPASS_SIZE
    CX = S // 2
    CY = S // 2
    R  = S // 2 - 30

    img = np.zeros((S, S, 3), dtype=np.uint8)
    img[:] = (18, 18, 18)

    # Rings
    for frac, alpha in [(1.0, 60), (0.67, 40), (0.33, 30)]:
        cv2.circle(img, (CX, CY), int(R * frac),
                   (alpha * 3, alpha * 3, alpha * 3), 1)

    # Tick marks
    for deg in range(0, 360, 10):
        rad   = np.radians(deg)
        is_major = deg % 30 == 0
        r_out = R
        r_in  = R - (10 if is_major else 5)
        x1 = int(CX + r_out * np.sin(rad))
        y1 = int(CY - r_out * np.cos(rad))
        x2 = int(CX + r_in  * np.sin(rad))
        y2 = int(CY - r_in  * np.cos(rad))
        cv2.line(img, (x1, y1), (x2, y2),
                 (100, 100, 100) if is_major else (55, 55, 55), 1)

    # Cardinal labels

    cardinals = [("N", 0), ("NW", 45), ("W", 90), ("SW", 135),
                 ("S", 180), ("SE", 225), ("E", 270), ("NE", 315)]

    for label, deg in cardinals:
        rad = np.radians(deg)
        lx  = int(CX + (R - 20) * np.sin(rad))
        ly  = int(CY - (R - 20) * np.cos(rad))
        color = (60, 60, 220) if label == "N" else (170, 170, 170)
        fs    = 0.55 if len(label) == 1 else 0.4
        fw    = 2    if len(label) == 1 else 1
        # Centre text
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, fw)
        cv2.putText(img, label, (lx - tw // 2, ly + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, color, fw)

    # Platform dot
    cv2.circle(img, (CX, CY), 6, (230, 230, 230), -1)

    # Arrow
    rad       = np.radians(-az_deg)
    arrow_len = int(R * 0.80)
    ex = int(CX + arrow_len * np.sin(rad))
    ey = int(CY - arrow_len * np.cos(rad))
    cv2.arrowedLine(img, (CX, CY), (ex, ey),
                    (80, 160, 230), 3, tipLength=0.2)
    cv2.circle(img, (ex, ey), 7, (50, 130, 220), -1)
    cv2.circle(img, (ex, ey), 7, (255, 255, 255), 1)

    # Large degree readout
    sign    = "+" if az_deg >= 0 else ""
    deg_str = f"{sign}{az_deg:.1f}\xb0"
    (tw, _), _ = cv2.getTextSize(deg_str, cv2.FONT_HERSHEY_DUPLEX, 1.3, 2)
    tx = (S - tw) // 2
    cv2.putText(img, deg_str, (tx, S - 55),
                cv2.FONT_HERSHEY_DUPLEX, 1.3, (30, 30, 30), 4)
    cv2.putText(img, deg_str, (tx, S - 55),
                cv2.FONT_HERSHEY_DUPLEX, 1.3, (80, 160, 230), 2)

    # Time info
    cv2.putText(img, f"t = {max(0, t_rel):.1f}s",
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (140, 140, 140), 1)
    cv2.putText(img, f"sample {sample_idx + 1} / {N_SAMPLES}",
                (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (110, 110, 110), 1)

    # Title
    cv2.putText(img, "Direction of Arrival",
                (12, S - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (100, 100, 100), 1)

    return img


def scale_to_height(img, height):
    h, w = img.shape[:2]
    new_w = int(w * height / h)
    return cv2.resize(img, (new_w, height))


def main():
    global SYNC_OFFSET

    # ── Open video ────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"ERROR: could not open video:\n  {VIDEO_PATH}")
        print("Check that the VIDEO_PATH at the top of the script is correct.")
        return

    # ── Output video writer ───────────────────────────────────────────────────
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = f"doa_sync_{ts}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    # Writer is initialised on first frame once we know the combined frame size
    out_writer = None

    vid_fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    vid_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    vid_dur   = vid_total / vid_fps
    print(f"Opened: {VIDEO_PATH}")
    print(f"  {vid_total} frames  |  {vid_fps:.1f} fps  |  {vid_dur:.1f}s")
    print(f"  DOA data spans {DATA_DURATION:.1f}s  |  {N_SAMPLES} samples")
    print(f"  Initial sync offset: {SYNC_OFFSET:+.1f}s")
    print()
    print("Controls:  SPACE=pause  Q=quit  A/D or LEFT/RIGHT=seek 5s  +/-=offset")

    paused  = False
    t_start = time.time()
    t_pause = 0.0

    while True:
        # ── Current time ──────────────────────────────────────────────────────
        elapsed = t_pause if paused else time.time() - t_start
        elapsed = max(0.0, min(elapsed, vid_dur))

        # ── Read video frame ──────────────────────────────────────────────────
        target_frame = int(elapsed * vid_fps)
        target_frame = min(target_frame, vid_total - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
        ret, vid_img = cap.read()
        if not ret:
            print("End of video.")
            break
        vid_img = scale_to_height(vid_img, DISPLAY_HEIGHT)

        # ── Build compass ─────────────────────────────────────────────────────
        az_time            = elapsed - SYNC_OFFSET
        az_deg, sample_idx = get_az_at_time(az_time)
        compass_img        = draw_compass(az_deg, az_time, sample_idx)
        compass_img        = scale_to_height(compass_img, DISPLAY_HEIGHT)

        # ── Info bar ──────────────────────────────────────────────────────────
        total_w  = vid_img.shape[1] + compass_img.shape[1]
        info_bar = np.zeros((36, total_w, 3), dtype=np.uint8)
        info_bar[:] = (28, 28, 28)

        status    = "|| PAUSED" if paused else "> PLAYING"
        sign      = "+" if az_deg >= 0 else ""
        doa_active = 0 <= az_time <= DATA_DURATION
        doa_status = f"{sign}{az_deg:.1f}\xb0" if doa_active else "-- no data --"
        info = (f"{status}   video: {elapsed:.1f}s / {vid_dur:.1f}s   "
                f"DOA: {doa_status}   offset: {SYNC_OFFSET:+.1f}s   "
                f"[+/-] adjust sync   [SPACE] pause   [Q] quit")
        cv2.putText(info_bar, info, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)

        # ── Display ───────────────────────────────────────────────────────────
        combined = np.hstack([vid_img, compass_img])
        display  = np.vstack([combined, info_bar])
        cv2.imshow("Drone DOA Sync Player  —  Q to quit", display)
        # ── Write frame to output ───────
        # ──────────────────────────────────────────
        if out_writer is None:
            h, w = display.shape[:2]
            out_writer = cv2.VideoWriter(output_path, fourcc, vid_fps, (w, h))
            print(f"Recording to: {output_path}")
        out_writer.write(display)

        # ── Key handling ──────────────────────────────────────────────────────
        key = cv2.waitKey(30) & 0xFF

        if key in (ord('q'), 27):           # Q or ESC — quit
            break

        elif key == ord(' '):               # SPACE — pause/resume
            if paused:
                t_start = time.time() - t_pause
                paused  = False
            else:
                t_pause = elapsed
                paused  = True

        elif key in (81, ord('a')):         # LEFT / A — rewind 5s
            new_t = max(0.0, elapsed - 5.0)
            t_start = time.time() - new_t
            if paused: t_pause = new_t

        elif key in (83, ord('d')):         # RIGHT / D — skip 5s
            new_t = min(vid_dur, elapsed + 5.0)
            t_start = time.time() - new_t
            if paused: t_pause = new_t

        elif key in (ord('+'), ord('=')):   # + — DOA later
            SYNC_OFFSET += 0.5
            print(f"Offset: {SYNC_OFFSET:+.1f}s  (DOA starts later in video)")

        elif key == ord('-'):               # - — DOA earlier
            SYNC_OFFSET -= 0.5
            print(f"Offset: {SYNC_OFFSET:+.1f}s  (DOA starts earlier in video)")

        # ── Timing ───────────────────────────────────────────────────────────
        if not paused:
            time.sleep(max(0.0, 1.0 / vid_fps - 0.015))

    cap.release()
    if out_writer is not None:
        out_writer.release()
        print(f"Saved: {output_path}")

    cv2.destroyAllWindows()
    print(f"\nFinal sync offset: {SYNC_OFFSET:+.1f}s")
    print("Set SYNC_OFFSET at the top of the script to this value to save it.")


if __name__ == "__main__":
    main()