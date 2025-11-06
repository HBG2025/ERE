
"""
plotar.py — Field/Park plotting utility (English comments)
---------------------------------------------------------
Generates plots from the requested experiment folder with fixed Y-limits and a
300-second time window. Outputs PNGs in the SAME experiment folder.

HOW TO USE
1) Set DATA_SUBFOLDER to the name of the experiment directory you want to plot.
   By default it's set to the last folder we used:
       "PruebaCampo2-cono160-setpoint1metro-distanciacultivo1metro"

   Examples you can switch to (if present in your tree):
       - "prueba1_parque"
       - "Pruebaenparque_cono160_conos2metroseparados_setpiont1metro"
       - "Pruebaenparque_30cmConos90gradosapertura"
       - "Prueba2_parque_4fotosrviz"
       - "Prueba3_4 fotos"
       - "PruebaconconosIncreible"
       - "PruebaconosIncreible2"

2) Run:
       python3 plotar.py

This script searches *recursively* inside the chosen folder (and its subfolders)
but will NEVER leave that folder.
"""

import os, glob
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --------------- CONFIGURATION ---------------
HERE = Path(__file__).resolve().parent

# 👉 Change this to the folder you want to plot (relative to this script).
DATA_SUBFOLDER = "PruebaCampo2-cono160-setpoint1metro-distanciacultivo1metro"

# Build absolute paths
DATA_DIR = HERE / DATA_SUBFOLDER

# Save plots directly into the selected experiment folder:
OUT = DATA_DIR
OUT.mkdir(exist_ok=True, parents=True)

# Fixed Y-limits (as requested)
YLIM_DIST_ERR   = (-1.0, 1.0)   # Distance error [m]
YLIM_YAW_ERR    = (-2.0, 2.0)   # Yaw error [rad]
YLIM_VALID_READ = (0, 100)      # Valid readings [count]
YLIM_LIDAR      = (0.0, 3.0)    # Lidar left [m]
YLIM_COMPARISON = (0.0, 6.28)   # Desired vs Measured [rad/s]

# Plot only the first 300 seconds
TMAX = 300.0

# Filename prefixes to look for (CSV files)
PREFIXES = [
    "csv_ErrorDist_",
    "csv_ImuError_",
    "csv_imu_",
    "csv_omega1_",
    "csv_omega2_",
    "csv_LidarLeft_",
    "csv_LecVal_",
    "csv_VelDes_",
]

# --------------- HELPERS ---------------
def find_latest(prefix: str) -> Path:
    """
    Recursively find the *latest* CSV that starts with the given prefix
    INSIDE the selected experiment folder (DATA_DIR).
    """
    pattern = str(DATA_DIR / "**" / f"{prefix}*.csv")
    matches = glob.glob(pattern, recursive=True)
    if not matches:
        raise FileNotFoundError(f"No CSV files matching: {pattern}")
    latest = max(matches, key=os.path.getmtime)
    print(f"[OK] {prefix} -> {latest}")
    return Path(latest)

def load_csv(path: Path) -> pd.DataFrame:
    """
    Load the CSV, normalize time to start at 0, and truncate to [0, TMAX].
    """
    df = pd.read_csv(path)
    if "ros_time_s" in df.columns:
        t0 = df["ros_time_s"].iloc[0]
        df["t"] = df["ros_time_s"] - t0
    elif "time_s" in df.columns:
        t0 = df["time_s"].iloc[0]
        df["t"] = df["time_s"] - t0
    else:
        df["t"] = np.arange(len(df)) * 1.0
    df = df[(df["t"] >= 0) & (df["t"] <= TMAX)].copy()
    return df

def save(fig_name: str):
    """
    Save current Matplotlib figure to the selected experiment folder.
    """
    plt.savefig(OUT / fig_name, dpi=160, bbox_inches="tight")
    plt.close()

# --------------- LOAD DATASETS ---------------
# Get the latest file per prefix from the selected folder
files = {p: find_latest(p) for p in PREFIXES}
dfs = {k: load_csv(v) for k, v in files.items()}

# Parse desired velocities "w_left,w_right" -> two columns
if "csv_VelDes_" in dfs and "Vel_Des" in dfs["csv_VelDes_"].columns:
    parts = dfs["csv_VelDes_"]["Vel_Des"].astype(str).str.split(",", expand=True)
    dfs["csv_VelDes_"]["Vel_Des_left"]  = pd.to_numeric(parts[0], errors="coerce")
    dfs["csv_VelDes_"]["Vel_Des_right"] = pd.to_numeric(parts[1], errors="coerce")

# --------------- PLOTS ---------------

# Distance error
plt.figure()
plt.plot(dfs["csv_ErrorDist_"]["t"], dfs["csv_ErrorDist_"]["Error_Distancia"], label="Distance_Error")
plt.xlabel("Time [s]"); plt.ylabel("Distance error [m]")
plt.grid(True); plt.legend()
plt.ylim(*YLIM_DIST_ERR); plt.xlim(0, TMAX)
save("Distance_Error.png")

# IMU yaw error  (IMU heading plot intentionally REMOVED)
plt.figure()
plt.plot(dfs["csv_ImuError_"]["t"], dfs["csv_ImuError_"]["Imu_Error_yaw"], label="IMU_Yaw_Error")
plt.xlabel("Time [s]"); plt.ylabel("Yaw error [rad]")
plt.grid(True); plt.legend()
plt.ylim(*YLIM_YAW_ERR); plt.xlim(0, TMAX)
save("IMU_Yaw_Error.png")

# Lidar left
plt.figure()
plt.plot(dfs["csv_LidarLeft_"]["t"], dfs["csv_LidarLeft_"]["Lidar_Left"], label="Lidar Left")
plt.xlabel("Time [s]"); plt.ylabel("Left distance [m]")
plt.grid(True); plt.legend()
plt.ylim(*YLIM_LIDAR); plt.xlim(0, TMAX)
save("Lidar_Left.png")

# Valid readings
plt.figure()
plt.plot(dfs["csv_LecVal_"]["t"], dfs["csv_LecVal_"]["Lecturas_Validas"], label="Valid Readings")
plt.xlabel("Time [s]"); plt.ylabel("Valid readings")
plt.grid(True); plt.legend()
plt.ylim(*YLIM_VALID_READ); plt.xlim(0, TMAX)
save("Valid_Readings.png")

# Comparisons (no single omega_* plots)
if "csv_VelDes_" in dfs and "Vel_Des_left" in dfs["csv_VelDes_"].columns:
    # Left wheel comparison
    plt.figure()
    plt.plot(dfs["csv_VelDes_"]["t"], dfs["csv_VelDes_"]["Vel_Des_left"],  label="Desired left")
    plt.plot(dfs["csv_omega1_"]["t"],  dfs["csv_omega1_"]["omega_1"],      label="Measured left")
    plt.xlabel("Time [s]"); plt.ylabel("Velocity [rad/s]")
    plt.title("Comparison: Desired vs Measured (Left)")
    plt.grid(True); plt.legend()
    plt.ylim(*YLIM_COMPARISON); plt.xlim(0, TMAX)
    save("comparison_desired_vs_measured_left.png")

    # Right wheel comparison
    plt.figure()
    plt.plot(dfs["csv_VelDes_"]["t"], dfs["csv_VelDes_"]["Vel_Des_right"], label="Desired right")
    plt.plot(dfs["csv_omega2_"]["t"], dfs["csv_omega2_"]["omega_2"],       label="Measured right")
    plt.xlabel("Time [s]"); plt.ylabel("Velocity [rad/s]")
    plt.title("Comparison: Desired vs Measured (Right)")
    plt.grid(True); plt.legend()
    plt.ylim(*YLIM_COMPARISON); plt.xlim(0, TMAX)
    save("comparison_desired_vs_measured_right.png")

print(f"✅ Using data folder: {DATA_DIR}")
print(f"✅ Plots saved in: {OUT}")