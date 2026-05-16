import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

# -----------------------------
# CONFIG
# -----------------------------
HAILO_DIR = "Test_Data/HailoTest"
NOHAILO_DIR = "Test_Data/NoHailo"

OUTPUT_DIR = "plots"
os.makedirs(OUTPUT_DIR, exist_ok=True)

EXCLUDED_COLUMNS = ["timestamp", "frame_number"]

# -----------------------------
# LOAD CSV FILES
# -----------------------------
def load_folder(folder_path, label):
    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))

    if not csv_files:
        raise RuntimeError(f"No CSV files found in {folder_path}")

    dataframes = []

    for file in csv_files:
        df = pd.read_csv(file)

        # Add test label
        df["test_type"] = label

        dataframes.append(df)

        print(f"Loaded: {file}")

    return pd.concat(dataframes, ignore_index=True)

hailo_df = load_folder(HAILO_DIR, "Hailo")
nohailo_df = load_folder(NOHAILO_DIR, "NoHailo")

# -----------------------------
# FIND NUMERIC METRICS
# -----------------------------
numeric_columns = [
    col for col in hailo_df.columns
    if col not in EXCLUDED_COLUMNS
    and col != "test_type"
    and pd.api.types.is_numeric_dtype(hailo_df[col])
]

print("\nMetrics found:")
for col in numeric_columns:
    print(f" - {col}")

# -----------------------------
# PLOT EACH METRIC BETWEEN TESTS
# -----------------------------
for metric in numeric_columns:

    plt.figure(figsize=(10, 5))

    # Plot Hailo
    plt.plot(
        hailo_df.index,
        hailo_df[metric],
        label="Hailo",
        alpha=0.8
    )

    # Plot NoHailo
    plt.plot(
        nohailo_df.index,
        nohailo_df[metric],
        label="NoHailo",
        alpha=0.8
    )

    plt.title(f"{metric} Comparison")
    plt.xlabel("Sample")
    plt.ylabel(metric)

    plt.legend()
    plt.grid(True)

    save_path = os.path.join(
        OUTPUT_DIR,
        f"{metric}_comparison.png"
    )

    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"Saved: {save_path}")

print("\nDone generating comparison plots.")