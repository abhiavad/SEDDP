import os, re, math
import numpy as np
import matplotlib.pyplot as plt

# ─── CONFIG ──────────────────────────────────────────────────────────────────
DATA_DIRS = {
    "Static Test 1": r"Test_Results_V2/Static/Static_5_8_10.44",
    "Static Test 2": r"Test_Results_V2/Static/Static_5_8_14.41",
    "Static Test 3": r"Test_Results_V2/Static/Static_5_9_14.31",
}

PITCH_BIAS_MEAN_DEG = 0.0                           # Systematic bias to remove.
PITCH_BIAS_STD_DEG  = 0.0                           # 1σ bias to remove in quadrature.

ROLL_BIAS_MEAN_DEG  = 0.0                           # Systematic bias to remove.
ROLL_BIAS_STD_DEG   = 0.2779194649983036            # 1σ bias to remove in quadrature.

COL_PITCH  = 770                                    # Column index holding pitch measurements.
COL_ROLL   = 771                                    # Column index holding roll measurements.

rx = re.compile(r"p(?P<pitch>[-\d.]+)_r(?P<roll>[-\d.]+)\.csv$", re.I)
# ─────────────────────────────────────────────────────────────────────────────

def process_dir(folder):
    """Return (roll_array, roll_err_mean, roll_err_sigma, pitch_err_mean, pitch_err_sigma) with bias removed in mean and σ for one folder."""
    files = [f for f in os.listdir(folder) if rx.fullmatch(f)]
    if not files:
        raise RuntimeError(f"No static test files in {folder!s}")

    pitch_vals = sorted({float(rx.fullmatch(f)["pitch"]) for f in files})
    roll_vals  = sorted({float(rx.fullmatch(f)["roll" ]) for f in files})
    pi = {p:i for i,p in enumerate(pitch_vals)}
    ri = {r:j for j,r in enumerate(roll_vals)}

    res = np.full((len(pitch_vals), len(roll_vals), 4), np.nan)

    campaign_roll_errors = []
    campaign_pitch_errors = []

    for fname in files:
        m = rx.fullmatch(fname)
        pitch_cmd = float(m["pitch"])
        roll_cmd = float(m["roll"])
        # if roll_cmd < 40.0:
        #     continue  # Skip roll commands below 40°.
        dat = np.loadtxt(os.path.join(folder, fname), delimiter=",", skiprows=1)

        # Remove mean bias:
        pitch_errs = dat[:,COL_PITCH] - pitch_cmd - PITCH_BIAS_MEAN_DEG     # Remove systematic bias pitch.
        roll_errs  = dat[:,COL_ROLL ] - roll_cmd - ROLL_BIAS_MEAN_DEG      # Remove systematic bias roll.

        campaign_roll_errors.append(roll_errs)
        campaign_pitch_errors.append(pitch_errs)

        # Mean error
        res[pi[pitch_cmd], ri[roll_cmd], 0] = pitch_errs.mean()        # Mean error pitch.
        res[pi[pitch_cmd], ri[roll_cmd], 2] = roll_errs.mean()         # Mean error roll.

        # Remove bias-σ in quadrature from each run’s variance
        var_p = pitch_errs.var(ddof=1) 
        var_r = roll_errs.var(ddof=1)

        corr_var_p = max(var_p - PITCH_BIAS_STD_DEG**2, 0.0)    # Corrected variance pitch.
        corr_var_r = max(var_r - ROLL_BIAS_STD_DEG**2, 0.0)     # Corrected variance roll.

        res[pi[pitch_cmd], ri[roll_cmd], 1] = math.sqrt(corr_var_p)
        res[pi[pitch_cmd], ri[roll_cmd], 3] = math.sqrt(corr_var_r)

    # collapse pitch dimension → functions of roll
    roll_mean  = np.nanmean(res[:,:,2], axis=0)
    roll_sigma = np.nanmean(res[:,:,3], axis=0)

    pitch_mean  = np.nanmean(res[:,:,0], axis=0)
    pitch_sigma = np.nanmean(res[:,:,1], axis=0)

    return np.array(roll_vals), roll_mean, roll_sigma, pitch_mean, pitch_sigma, campaign_roll_errors, campaign_pitch_errors

if __name__ == "__main__":
    plt.figure(figsize=(10, 6))

    marker_cycle = ["o", "s", "^", "D", "v"]

    # Accumulators for gathering values for grand averages (data analysis).
    roll_errs_global  = []
    pitch_errs_global = []

    # Accumulators for plotting per-campaign averages (plotting).
    test_campaign_roll_graph  = []
    test_campaign_pitch_graph = [] 

    for k, (label, folder) in enumerate(DATA_DIRS.items()):
        roll_common, roll_avg, roll_std, pitch_avg, pitch_std, roll_errs_campaign, pitch_errs_campaign = process_dir(folder)

        # Campaign data analysis part. ------------------------
        roll_errs_global.extend(roll_errs_campaign)
        pitch_errs_global.extend(pitch_errs_campaign)
        # -----------------------------------------------------

        # Campaign plotting part. -----------------------------
        mk = marker_cycle[k % len(marker_cycle)]
        test_campaign_roll_graph.append(roll_avg)
        test_campaign_pitch_graph.append(pitch_avg)

        plt.errorbar(roll_common, roll_avg,  yerr=roll_std, marker=f"{mk}", linestyle="-", lw=1, label=f"{label} (Roll)")
        plt.errorbar(roll_common, pitch_avg, yerr=pitch_std, marker=f"{mk}", linestyle="--", lw=1, label=f"{label} (Pitch)")
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
    # -----------------------------------------------------

    # Global plotting part. -------------------------------
    grand_roll_average = np.vstack(test_campaign_roll_graph)
    grand_pitch_average = np.vstack(test_campaign_pitch_graph)

    grand_roll_average = np.nanmean(grand_roll_average, axis=0)
    grand_pitch_average = np.nanmean(grand_pitch_average, axis=0)

    plt.errorbar(roll_common, grand_roll_average,  yerr=roll_std, color="tab:blue", marker="o", linestyle="-", lw=2, label="Average All Campaigns (Roll)")
    plt.errorbar(roll_common, grand_pitch_average, yerr=pitch_std, color="tab:orange", marker="o", linestyle="--", lw=2, label="Average All Campaigns (Pitch)")
    # -----------------------------------------------------

    # Finalize the plot and print the results.
    plt.xlabel("Commanded Roll Angle (deg)")
    plt.ylabel("Error Angle (deg)")
    plt.title("Static Test Errors\n(All Campaigns & Average)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

    print(f"Overall roll error:  {overall_roll_mean:.3f} ± {overall_roll_std:.3f}°")
    print(f"Overall pitch error: {overall_pitch_mean:.3f} ± {overall_pitch_std:.3f}°")