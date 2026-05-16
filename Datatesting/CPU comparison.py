import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

# -----------------------------
# CONFIG
# -----------------------------
HAILO_DIR   = "Test_Data/HailoTest"
NOHAILO_DIR = "Test_Data/NoHailo"
OUTPUT_DIR  = "plots/final"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Only these 4 metrics
METRICS = {
    "avg_inference_ms": "Inference Time (ms)",
    "cpu_usage_pct":    "CPU Usage (%)",
    "cpu_temp_c":       "CPU Temperature (°C)",
    "ram_used_mb":      "RAM Usage (MB)",
}

# -----------------------------
# LOAD CSV FILES
# -----------------------------
def load_folder(folder_path, label):
    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))
    if not csv_files:
        raise RuntimeError(f"No CSV files found in {folder_path}")
    dfs = []
    for file in csv_files:
        df = pd.read_csv(file)
        df["test_type"] = label
        # Skip first warmup row
        df = df.iloc[1:].reset_index(drop=True)
        dfs.append(df)
        print(f"Loaded: {file}")
    return pd.concat(dfs, ignore_index=True)

hailo_df   = load_folder(HAILO_DIR,   "Hailo")
nohailo_df = load_folder(NOHAILO_DIR, "NoHailo")

# Use seconds from start as x-axis
hailo_x   = range(len(hailo_df))
nohailo_x = range(len(nohailo_df))

# -----------------------------
# PLOT
# -----------------------------
for col, ylabel in METRICS.items():
    if col not in hailo_df.columns or col not in nohailo_df.columns:
        print(f"Column '{col}' not found, skipping.")
        continue

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(hailo_x,   hailo_df[col],   label="Hailo",    alpha=0.85, linewidth=1.2)
    ax.plot(nohailo_x, nohailo_df[col], label="CPU only", alpha=0.85, linewidth=1.2)

    # Add average lines
    hailo_avg   = hailo_df[col].mean()
    nohailo_avg = nohailo_df[col].mean()
    ax.axhline(hailo_avg,   color="C0", linestyle="--", linewidth=0.9,
               label=f"Hailo avg: {hailo_avg:.1f}")
    ax.axhline(nohailo_avg, color="C1", linestyle="--", linewidth=0.9,
               label=f"CPU avg: {nohailo_avg:.1f}")

    ax.set_title(f"{ylabel} — Hailo vs CPU Only", fontsize=13)
    ax.set_xlabel("Time (seconds)", fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    plt.tight_layout()
    save_path = os.path.join(OUTPUT_DIR, f"{col}_comparison.png")
    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"Saved: {save_path}")

print("\nDone. 4 plots saved to", OUTPUT_DIR)