import os, re
import numpy as np
import matplotlib.pyplot as plt

# ─── CONFIG ──────────────────────────────────────────────────────────────────
DATA_DIRS = {
    "Dynamic Test 1": r"Test_Results_V2/Dynamic/Dynamic_5_8_11.00",
    "Dynamic Test 2": r"Test_Results_V2/Dynamic/Dynamic_5_8_14.54",
    "Dynamic Test 3": r"Test_Results_V2/Dynamic/Dynamic_5_12_08.51",
}

TARGET_RATE = 5.0                                   # Only plot the 5 deg/s case.
INITIAL_ROLL_CMD = 30.0                             # Commanded roll angle at t=0s.

PITCH_BIAS_MEAN_DEG = 0.0                           # Systematic bias to remove.
PITCH_BIAS_STD_DEG  = 0.0                           # 1σ bias to remove in quadrature.

ROLL_BIAS_MEAN_DEG  = 0.0                           # Systematic bias to remove.
ROLL_BIAS_STD_DEG   = 0.2779194649983036            # 1σ bias to remove in quadrature.

# Column Indices
COL_PITCH     = 770   # pitch measurement
COL_ROLL      = 771   # roll  measurement
COL_TIMESTAMP = 774   # timestamp in ms

# match filenames like "s5.0_p30.csv"
rx_dyn = re.compile(r"s(?P<rate>[-\d.]+)_p(?P<pitch>[-\d.]+)\.csv$", re.I)
# ─────────────────────────────────────────────────────────────────────────────

def load_dynamic(folder, target_rate):
    """
    Returns a dict mapping pitch_cmd to (roll_cmd, roll_err, pitch_err).
    Does this for every csv file in the given folder that matches the target rate.
    """
    out = {}
    for fname in sorted(os.listdir(folder)):
        m = rx_dyn.fullmatch(fname)
        if not m:
            continue
        
        rate      = float(m.group("rate"))      # Roll rate of this file.
        pitch_cmd = float(m.group("pitch"))     # Commanded pitch angle in this file.

        if not np.isclose(rate, target_rate):   # Skip if not the target rate.
            continue

        data = np.loadtxt(os.path.join(folder, fname), delimiter=",", skiprows=1)   # Load the data file.

        ts = data[:, COL_TIMESTAMP]
        t  = (ts - ts[0]) / 1000.0

        roll_cmd = INITIAL_ROLL_CMD + rate * t

        roll_err  = data[:, COL_ROLL ] - roll_cmd - ROLL_BIAS_MEAN_DEG
        pitch_err = data[:, COL_PITCH] - pitch_cmd - PITCH_BIAS_MEAN_DEG

        out[pitch_cmd] = (roll_cmd, roll_err, pitch_err)
    
    if not out:
        raise RuntimeError(f"No files at {target_rate}°/s in {folder}")
    return out

def average_runs(dyn_runs, n_points):
    """
    dyn_runs: dict[pitch_cmd] → (angle_cmd, roll_err, pitch_err)
    Returns: (angle_common, roll_mean, pitch_mean)
    """
    # find the smallest max angle among runs
    max_angles = [angle_arr[-1] for (angle_arr, _, _) in dyn_runs.values()]
    # roll_max  = min(max_angles)
    roll_max = 105

    # common commanded-angle axis
    roll_common = np.linspace(INITIAL_ROLL_CMD, roll_max, n_points)

    roll_mat  = []
    pitch_mat = []

    for (angle_arr, r_err, p_err) in dyn_runs.values():
        # interpolate each run's errors onto angle_common
        roll_mat.append(np.interp(roll_common, angle_arr, r_err))
        pitch_mat.append(np.interp(roll_common, angle_arr, p_err))

    roll_mat  = np.vstack(roll_mat)   # shape (n_runs, n_points)
    pitch_mat = np.vstack(pitch_mat)

    return (
        roll_common,
        roll_mat.mean(axis=0),
        pitch_mat.mean(axis=0),
    )

def tile_curve(x, y, period=90, full_span=360):
    """
    Repeat a curve every `period` degrees and return a single
    (x_tiled, y_tiled) that runs monotonically from 0 to `full_span`.
    """
    tiled_x, tiled_y = [], []
    for shift in range(0, full_span, period):
        x_shift = (x + shift) % full_span
        tiled_x.append(x_shift)
        tiled_y.append(y)

    tiled_x = np.concatenate(tiled_x)
    tiled_y = np.concatenate(tiled_y)

    # sort so the x-array is monotonic 0→360 for plotting
    order = np.argsort(tiled_x)
    return tiled_x[order], tiled_y[order]

def mirror_curve(x, y, full_span=360):
    """
    Produce the left–right mirror of a (x,y) curve about 180°:
        x_mirror = (full_span - x) mod full_span
    Returns (x_sorted, y_sorted) ready for plotting.
    """
    x_mir = (full_span - x) % full_span
    order = np.argsort(x_mir)
    return x_mir[order], y[order]

if __name__ == "__main__":

    # Number of points to use for averaging and tiling.
    n_points = 1000

    # Accumulators for gathering values for grand averages (data analysis).
    roll_errs_global  = []
    pitch_errs_global = []

    # Accumulators for plotting per-campaign averages (plotting).
    test_campaign_roll_graph  = []
    test_campaign_pitch_graph = []

    for label, folder in DATA_DIRS.items():
        # For every test campaign in the config: computes the errors in the angles.
        
        runs = load_dynamic(folder, TARGET_RATE)    # Returns a dict mapping columns containg the commanded roll, roll error and the pitch error.

        # Campaign data analysis part. ----------------------
        # Separate roll and pitch errors.
        roll_errs_campaign  = np.hstack([r_err for (_, r_err, _) in runs.values()])
        pitch_errs_campaign = np.hstack([p_err for (_, _, p_err) in runs.values()])

        # Append the roll and pitch errors to the global accumulators.
        roll_errs_global.append(roll_errs_campaign)
        pitch_errs_global.append(pitch_errs_campaign)
        # -------------------------------------------------

        # Campaign plotting part. -------------------------
        # Compute each campaign's average across all pitches.
        roll_common, roll_avg, pitch_avg = average_runs(runs, n_points)
        test_campaign_roll_graph.append(roll_avg)
        test_campaign_pitch_graph.append(pitch_avg)

        # Tile the roll and pitch curves to cover 0–360°.
        roll_common_tile, roll_avg_tile  = tile_curve(roll_common,  roll_avg)
        roll_common_tile, pitch_avg_tile = tile_curve(roll_common, pitch_avg)

        # Miror the tiled roll and pitch curves.
        roll_common_mirror, roll_avg_mirror  = mirror_curve(roll_common_tile,  roll_avg_tile)
        roll_common_mirror, pitch_avg_mirror = mirror_curve(roll_common_tile, pitch_avg_tile)

        # Plot the tiled roll and pitch errors for this campaign.
        plt.plot(roll_common_tile, roll_avg_tile, linestyle="-",  label=f"{label} (Roll)")
        plt.plot(roll_common_tile, pitch_avg_tile, linestyle="--",  label=f"{label} (Pitch)")

        # Plot the mirrored roll and pitch errors for this campaign.
        plt.plot(roll_common_mirror, roll_avg_mirror, linestyle=":", color=plt.gca().lines[-1].get_color(), alpha=0.5)
        plt.plot(roll_common_mirror, pitch_avg_mirror, linestyle=":", color=plt.gca().lines[-1].get_color(), alpha=0.5)
        # -------------------------------------------------

    # Global data analysis part. --------------------------
    # Concatenate across campaign for pooled stats.
    roll_errs_global  = np.hstack(roll_errs_global)
    pitch_errs_global = np.hstack(pitch_errs_global)

    # Calculate overall mean and std for roll and pitch errors.
    overall_roll_mean  = np.nanmean(roll_errs_global)
    overall_roll_std   = np.nanstd(roll_errs_global, ddof=1)
    overall_pitch_mean = np.nanmean(pitch_errs_global)
    overall_pitch_std  = np.nanstd(pitch_errs_global,  ddof=1)

    print(f"Overall roll error:  {overall_roll_mean:.3f} ± {overall_roll_std:.3f}°")
    print(f"Overall pitch error: {overall_pitch_mean:.3f} ± {overall_pitch_std:.3f}°")
    # -----------------------------------------------------

    # Global plotting part. -------------------------------
    # Grand-average curve across the three test campaigns.
    grand_roll_avg  = np.mean(test_campaign_roll_graph,  axis=0)
    grand_pitch_avg = np.mean(test_campaign_pitch_graph, axis=0)

    plt.figure(figsize=(10, 6))
    plt.plot(roll_common, grand_roll_avg, color="tab:blue", linestyle="-", lw=2, label="Average All Campaigns (Roll)")
    plt.plot(roll_common, grand_pitch_avg, color="tab:orange", linestyle="--", lw=2, label="Average All Campaigns (Pitch)")
    plt.xlabel("Commanded Roll Angle (deg)")
    plt.ylabel("Error Angle (deg)")
    plt.title(f"Dynamic Test Errors @ {TARGET_RATE}°/s\n(All Campaigns & Average)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    # -----------------------------------------------------

    # Tiled and mirrored grand average curves. ------------
    # Tile the grand average curves to cover 0–360°.
    roll_common_tile, grand_roll_avg_tile = tile_curve(roll_common,  grand_roll_avg)
    roll_common_tile, grand_pitch_avg_tile = tile_curve(roll_common, grand_pitch_avg)

    # Mirror the tiled grand average curves.
    roll_common_mirror, grand_roll_avg_mirror  = mirror_curve(roll_common_tile,  grand_roll_avg_tile)
    roll_common_mirror, grand_pitch_avg_mirror = mirror_curve(roll_common_tile, grand_pitch_avg_tile)

    # Createa common roll range for the envelope. This range is used to interpolate both curves onto the same x-axis.
    roll_range = np.linspace(0, 360, 4*n_points)

    # Interpolate both the tiled and mirrored curves onto a single axis (roll range).
    grand_roll_avg_tile_interp = np.interp(roll_range, roll_common_tile, grand_roll_avg_tile, left=np.nan, right=np.nan)
    grand_roll_avg_mirror_interp = np.interp(roll_range, roll_common_mirror, grand_roll_avg_mirror, left=np.nan, right=np.nan)

    # Repeat the same for pitch.
    grand_pitch_avg_tile_interp = np.interp(roll_range, roll_common_tile, grand_pitch_avg_tile, left=np.nan, right=np.nan)
    grand_pitch_avg_mirror_interp = np.interp(roll_range, roll_common_mirror, grand_pitch_avg_mirror, left=np.nan, right=np.nan)

    # Take the absolute value of both roll curves and determine which one to use for the envelope.
    abs_roll_tile = np.abs(grand_roll_avg_tile_interp)
    abs_roll_mirror = np.abs(grand_roll_avg_mirror_interp)
    use_roll_tile = abs_roll_tile <= abs_roll_mirror

    # Rpeate the same for pitch.
    abs_pitch_tile = np.abs(grand_pitch_avg_tile_interp)
    abs_pitch_mirror = np.abs(grand_pitch_avg_mirror_interp)
    use_pitch_tile = abs_pitch_tile <= abs_pitch_mirror

    # Build the envelope: use the signed value from the curve with the smaller absolute value.
    grand_roll_env = np.where(use_roll_tile, grand_roll_avg_tile_interp, grand_roll_avg_mirror_interp)
    grand_pitch_env = np.where(use_pitch_tile, grand_pitch_avg_tile_interp, grand_pitch_avg_mirror_interp)
    # -----------------------------------------------------

    # Envelope statistics. --------------------------------
    overall_env_roll_mean  = np.nanmean(grand_roll_env)
    overall_env_pitch_mean = np.nanmean(grand_pitch_env)

    overall_env_roll_std   = np.nanstd(grand_roll_env, ddof=1)
    overall_env_pitch_std  = np.nanstd(grand_pitch_env, ddof=1)

    print("Maximum Roll Envelope Value:", np.nanmax(grand_roll_env))
    print("Minimum Roll Envelope Value:", np.nanmin(grand_roll_env))

    print("Maximum Pitch Envelope Value:", np.nanmax(grand_pitch_env))
    print("Minimum Pitch Envelope Value:", np.nanmin(grand_pitch_env))

    print(f"Overall roll error (4 EHS): {overall_env_roll_mean:.3f} ± {overall_env_pitch_std:.3f}°")
    print(f"Overall pitch error (4 EHS): {overall_env_pitch_mean:.3f} ± {overall_env_pitch_std:.3f}°")
    # -----------------------------------------------------

    # Envelope plotting. --------------------------------
    # Plot the tiled, mirrored and envelope average curve for roll.
    plt.figure(figsize=(10, 6))
    plt.plot(roll_common_tile, grand_roll_avg_tile, color="tab:blue", linestyle="--", lw=0.5, alpha=1, label="Tiled Average All Campaigns (Roll)")
    plt.plot(roll_common_mirror, grand_roll_avg_mirror, color="tab:red", linestyle="--", lw=0.5, alpha=1, label="Mirrored Average All Campaigns (Roll)")
    plt.plot(roll_range, grand_roll_env, color="k", lw=2, linestyle="-", label="Envelope = min(original, mirror) (Roll)")
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 361, 45))
    plt.xlabel("Commanded Roll Angle (deg)")
    plt.ylabel("Error Angle (deg)")
    plt.title(f"Dynamic Roll Test Errors @ {TARGET_RATE}°/s\n(Tiled every 90°, x-axis 0–360°)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    # Plot the tiled, mirrored and envelope average curve for pitch.
    plt.figure(figsize=(10, 6))
    plt.plot(roll_common_tile, grand_pitch_avg_tile, color="tab:orange", linestyle="--", lw=0.5, alpha=1, label="Tiled Average All Campaigns (Pitch)")
    plt.plot(roll_common_mirror, grand_pitch_avg_mirror, color="tab:purple", linestyle="--", lw=0.5, alpha=1, label="Mirrored Average All Campaigns (Pitch)")
    plt.plot(roll_range, grand_pitch_env, color="k", lw=2, linestyle="-", label="Envelope = min(original, mirror) (Pitch)")
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 361, 45))
    plt.xlabel("Commanded Roll Angle (deg)")
    plt.ylabel("Error Angle (deg)")
    plt.title(f"Dynamic Pitch Test Errors @ {TARGET_RATE}°/s\n(Tiled every 90°, x-axis 0–360°)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    # -----------------------------------------------------


    # Number of sensors with roll error < 5 deg. ----------
    # Count at each roll angle how many of the two curves have both abs(roll error) < 5 deg AND abs(pitch error) < 5 deg
    abs_roll_errors = np.vstack([np.abs(grand_roll_avg_tile_interp), np.abs(grand_roll_avg_mirror_interp)])  # shape (2, N)
    abs_pitch_errors = np.vstack([np.abs(grand_pitch_avg_tile_interp), np.abs(grand_pitch_avg_mirror_interp)])  # shape (2, N)
    both_below_5deg = np.logical_and(abs_roll_errors < 5, abs_pitch_errors < 5)
    num_both_below_5deg = np.sum(both_below_5deg, axis=0)  # shape (N,)

    # Plot this as a new figure or subplot
    plt.figure(figsize=(10, 3))
    plt.plot(roll_range, num_both_below_5deg, color="tab:green", lw=2)
    plt.xlabel("Commanded Roll Angle (deg)")
    plt.ylabel("Count")
    plt.title("Number of Sensors with |Roll error| < 5° and |Pitch error| < 5° vs Commanded Roll Angle")
    plt.xlim(0, 360)
    plt.ylim(-0.1, 2.1)
    plt.yticks([0, 1, 2])
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    # -----------------------------------------------------


    # Polar plotting. -------------------------------------
    # Circular (polar) plot of grand_roll_env
    plt.figure(figsize=(7, 7))
    ax = plt.subplot(111, polar=True)

    # Convert roll_range from degrees to radians
    theta = np.deg2rad(roll_range)

    # Plot the envelope
    ax.plot(theta, grand_roll_env, color="c", lw=2, label="Envelope = min(original, mirror) (Roll)")
    ax.plot(theta, grand_pitch_env, color="m", lw=2, label="Envelope = min(original, mirror) (Pitch)")

    # Plot the zero-error line as a dashed circle
    ax.plot(theta, np.zeros_like(theta), 'k--', lw=1, label="Zero Error")

    # Optional: set the direction and location of zero
    ax.set_theta_zero_location("S")
    ax.set_theta_direction(-1)       # Clockwise
    

    # Set r label and title
    ax.set_rlabel_position(135)
    ax.set_title(f"Roll and Pitch Error Envelope (Circular View) @ {TARGET_RATE}°/s", va='bottom')
    ax.legend(loc='upper right', bbox_to_anchor=(1.2, 1.1))
    ax.set_rlim(-5, 3)
    plt.tight_layout()
    # -----------------------------------------------------

    plt.show()