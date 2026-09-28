#!/usr/bin/env python3
"""
fermentation_kalman.py

Processes batched sensor-log CSVs from a fermentation monitor and Kalman-filters
the pressure, DS18B20 temperature, and thermistor temperature channels.

------------------------------------------------------------------------------
BACKGROUND / ASSUMPTIONS (please re-check these against your actual hardware!)
------------------------------------------------------------------------------
1. BATCHING & TIMESTAMPS
   The device takes several readings, holds them in memory, then uploads the
   whole batch at once. Every row in a batch shares the same `timestamp`
   column value, and `uptime_ms` counts milliseconds within that batch
   (resetting to 0 at the start of each batch).

   The `timestamp` is recorded at UPLOAD time, and upload happens right after
   the LAST reading in the batch is taken. So, for each batch:

       true_time(row) = timestamp - (max(uptime_ms in batch) - uptime_ms(row))

   This pushes every row in a batch backward in time except the last one,
   which lands exactly on the batch's timestamp.

2. CALIBRATION
   - DS18B20:      T_true = 1.054 * ds18b20_tempC - 1.296
   - Thermistor:   T_true = 0.003293 * thermistor_mV + 19.557
     (computed fresh from thermistor_mV -- the thermistor_tempC column in the
     input CSV is ignored, since it appears to use a different/uncalibrated
     formula.)
   - LPS33HW's onboard temperature (lps33hw_tempC) is carried through to the
     output CSV for reference only. It is NOT filtered and NOT used in the
     combined fermentation index, since it's reportedly not well-calibrated.
     pressure_hPa from the same chip IS used/filtered -- only its temperature
     reading is excluded.

3. KALMAN FILTER MODEL
   Each of the three channels (pressure_hPa, calibrated ds18b20, calibrated
   thermistor) is filtered independently with a constant-velocity ("trend")
   1D Kalman filter:
       state = [value, rate_of_change]
   This lets the filter track a steady warm-up/cool-down ramp instead of
   just smoothing toward a flat average. Time steps (dt) are computed from
   the *back-calculated* true_time, not the upload timestamp, so gaps
   between upload batches are handled correctly (the filter's uncertainty
   grows across a gap, so it leans on the new measurement rather than
   blindly extrapolating a trend across silence).

   Measurement noise (R) is auto-estimated per channel from the spread of
   readings *within* each upload batch (since the true value shouldn't
   change much across ~1 batch's worth of seconds). You can override this
   with --pressure-r / --ds18b20-r / --thermistor-r if you have a better
   estimate (e.g. from a datasheet or a static/no-fermentation test run).

   Process noise (Q intensity, "how fast is the trend allowed to change")
   defaults are conservative starting points -- see the --*-q flags. If a
   filtered curve looks too laggy behind real changes, raise its -q value.
   If it looks too jumpy/noisy, lower it (or raise -r).

4. COMBINED FERMENTATION INDEX
   avg_temp_filtered = mean(ds18b20_filtered, thermistor_filtered)
   Both avg_temp_filtered and pressure_filtered are z-score normalized over
   the whole run, then combined:
       activity_index = temp_weight * norm_temp + pressure_weight * norm_pressure
   Default weights are 0.5 / 0.5 (equal). This is a relative, unitless index
   meant to show *when* temperature and pressure are rising/falling together
   (a reasonable proxy for fermentation activity), not an absolute measurement
   of anything physical. Adjust weights with --temp-weight / --pressure-weight.

5. TEMPERATURE-ADJUSTED PRESSURE
   In a sealed vessel, pressure rises and falls partly just from temperature
   changes (a hotter headspace has higher pressure even with zero gas
   production) -- Gay-Lussac's Law for a fixed volume/mole count says
   P / T_kelvin = constant. To separate that purely thermal effect from
   pressure changes actually caused by gas production/consumption (e.g. CO2
   from fermentation), each reading is rescaled to a single reference
   temperature:

       P_adjusted = P_filtered * (T_ref_kelvin / T_filtered_kelvin)

   This is done twice, independently, using each temperature source as the
   basis (--pressure-ref selects the reference temperature: 'mean' [default],
   'first', or 'last' reading of that channel). The result approximates "what
   pressure would this be if temperature had stayed constant" -- residual
   trends in the adjusted curve are more likely attributable to actual gas
   production than to thermal expansion.

   Caveat: this assumes a rigid, sealed, constant-volume vessel with no gas
   escaping (e.g. no airlock venting once a threshold pressure is reached).
   If that's not the case, treat the adjustment as approximate.

------------------------------------------------------------------------------
USAGE
------------------------------------------------------------------------------
    python fermentation_kalman.py your_data.csv
    python fermentation_kalman.py your_data.csv --output-dir results --show
    python fermentation_kalman.py your_data.csv --temp-weight 0.7 --pressure-weight 0.3

Requires: pandas, numpy, matplotlib   (pip install pandas numpy matplotlib)
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

REQUIRED_COLUMNS = [
    "timestamp", "pressure_hPa", "lps33hw_tempC", "ds18b20_tempC",
    "thermistor_mV", "thermistor_tempC", "uptime_ms",
]


# ------------------------------------------------------------------------- #
# Kalman filter: constant-velocity ("trend") model
# ------------------------------------------------------------------------- #
class ConstantVelocityKalman:
    """
    1D Kalman filter with state = [value, rate_of_change].

    F(dt) = [[1, dt], [0, 1]]
    Q(dt)  = q * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]   (discretized white-noise
             acceleration model -- standard choice for a "trend" filter with
             irregular time steps)
    H = [1, 0]
    """

    def __init__(self, initial_value, measurement_variance, process_variance,
                 initial_rate=0.0, initial_value_variance=None,
                 initial_rate_variance=1.0):
        self.R = measurement_variance
        self.q = process_variance
        if initial_value_variance is None:
            initial_value_variance = measurement_variance * 10
        self.x = np.array([[initial_value], [initial_rate]], dtype=float)
        self.P = np.diag([initial_value_variance, initial_rate_variance]).astype(float)
        self.H = np.array([[1.0, 0.0]])

    def predict(self, dt):
        dt = max(dt, 1e-6)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = self.q
        Q = q * np.array([
            [dt**4 / 4, dt**3 / 2],
            [dt**3 / 2, dt**2],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        H = self.H
        y = z - (H @ self.x)[0, 0]
        S = (H @ self.P @ H.T)[0, 0] + self.R
        K = (self.P @ H.T) / S
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ H) @ self.P

    @property
    def value(self):
        return self.x[0, 0]

    @property
    def rate(self):
        return self.x[1, 0]


def kalman_filter_series(times_sec, values, measurement_variance, process_variance):
    """Run the constant-velocity Kalman filter over a time-ordered series."""
    values = np.asarray(values, dtype=float)
    times_sec = np.asarray(times_sec, dtype=float)
    filtered = np.zeros_like(values)

    kf = ConstantVelocityKalman(values[0], measurement_variance, process_variance)
    filtered[0] = kf.value
    for i in range(1, len(values)):
        dt = times_sec[i] - times_sec[i - 1]
        kf.predict(dt)
        kf.update(values[i])
        filtered[i] = kf.value
    return filtered


def temperature_adjusted_pressure(pressure_filtered_hPa, temp_filtered_C, ref_temp_C):
    """
    Rescale pressure to a reference temperature using Gay-Lussac's Law
    (P / T_kelvin = constant at fixed volume/moles):

        P_adjusted = P_filtered * (T_ref_K / T_filtered_K)

    This removes the pressure swing explained by thermal expansion alone,
    leaving a residual that's a better proxy for actual gas production.
    """
    T_K = temp_filtered_C + 273.15
    T_ref_K = ref_temp_C + 273.15
    return pressure_filtered_hPa * (T_ref_K / T_K)


def estimate_measurement_noise(df, column, batch_col="timestamp", fallback=None):
    """
    Estimate measurement variance from the spread of readings within each
    upload batch (rows sharing the same upload timestamp). Falls back to the
    overall series variance if no batch has more than one reading.
    """
    batch_variances = df.groupby(batch_col)[column].var(ddof=1).dropna()
    if len(batch_variances) > 0:
        return float(batch_variances.mean())
    if fallback is not None:
        return fallback
    overall_var = df[column].var(ddof=1)
    return float(overall_var) if pd.notna(overall_var) and overall_var > 0 else 1.0


# ------------------------------------------------------------------------- #
# Data loading / time back-calculation / calibration
# ------------------------------------------------------------------------- #
def load_and_prepare(csv_path):
    df = pd.read_csv(csv_path)

    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"Input CSV is missing required column(s): {missing}")

    before = len(df)
    df = df.dropna(subset=["timestamp", "pressure_hPa", "ds18b20_tempC",
                            "thermistor_mV", "uptime_ms"]).copy()
    if len(df) < before:
        print(f"Warning: dropped {before - len(df)} row(s) with missing required values.")

    df["timestamp"] = pd.to_datetime(df["timestamp"])

    # --- back-calculate true measurement time ---
    # timestamp = upload time = time of the LAST reading in its batch
    df["batch_max_uptime_ms"] = df.groupby("timestamp")["uptime_ms"].transform("max")
    df["offset_ms"] = df["batch_max_uptime_ms"] - df["uptime_ms"]
    df["true_time"] = df["timestamp"] - pd.to_timedelta(df["offset_ms"], unit="ms")

    df = df.sort_values("true_time").reset_index(drop=True)
    df["t_sec"] = (df["true_time"] - df["true_time"].iloc[0]).dt.total_seconds()

    # --- calibration ---
    df["ds18b20_tempC_corrected"] = 1.054 * df["ds18b20_tempC"] - 1.296
    df["thermistor_tempC_calc"] = 0.003293 * df["thermistor_mV"] + 19.557

    return df


# ------------------------------------------------------------------------- #
# Plotting
# ------------------------------------------------------------------------- #
def plot_channel(df, raw_col, filtered_col, ylabel, title, out_path):
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(df["true_time"], df[raw_col], "o", ms=3, alpha=0.4,
            color="tab:gray", label="Raw")
    ax.plot(df["true_time"], df[filtered_col], "-", lw=2,
            color="tab:blue", label="Kalman filtered")
    ax.set_xlabel("Time (back-calculated from uptime)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    return fig


def plot_pressure_adjustment(df, temp_col, adjusted_col, temp_label, out_path):
    fig, ax1 = plt.subplots(figsize=(10, 5))

    ax1.plot(df["true_time"], df["pressure_hPa_filtered"], "-", lw=1.5,
              color="tab:blue", alpha=0.6, label="Pressure (filtered, thermal effects included)")
    ax1.plot(df["true_time"], df[adjusted_col], "-", lw=2.5,
              color="tab:red", label=f"Pressure adjusted for temp ({temp_label})")
    ax1.set_xlabel("Time (back-calculated from uptime)")
    ax1.set_ylabel("Pressure (hPa)")
    ax1.legend(loc="upper left", fontsize=9)

    ax2 = ax1.twinx()
    ax2.plot(df["true_time"], df[temp_col], "--", lw=1, color="gray",
              alpha=0.5, label=f"{temp_label} (filtered)")
    ax2.set_ylabel("Temperature (°C)")
    ax2.legend(loc="upper right", fontsize=9)

    ax1.set_title(f"Pressure adjusted for temperature — using {temp_label}")
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    return fig


def plot_combined(df, out_path):
    fig, ax = plt.subplots(figsize=(11, 6))

    ax.plot(df["true_time"], df["fermentation_activity_index"], "-",
            lw=2.5, color="tab:red", label="Fermentation activity index")
    ax.plot(df["true_time"], df["temp_norm"], "--", lw=1, alpha=0.6,
            color="tab:orange", label="Normalized temp (ds18b20 + thermistor avg)")
    ax.plot(df["true_time"], df["pressure_norm"], "--", lw=1, alpha=0.6,
            color="tab:green", label="Normalized pressure")

    ax.axhline(0, color="black", lw=0.7, alpha=0.5)
    ax.set_xlabel("Time (back-calculated from uptime)")
    ax.set_ylabel("Z-score / index (unitless)")
    ax.set_title("Combined fermentation activity index (temp + pressure)")
    ax.legend()
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    return fig


# ------------------------------------------------------------------------- #
# Main
# ------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(
        description="Kalman-filter fermentation sensor logs and plot a combined activity index.")
    parser.add_argument("input_csv", help="Path to input CSV file")
    parser.add_argument("--output-dir", default="fermentation_output",
                         help="Directory to save filtered CSV + plots (default: ./fermentation_output)")

    parser.add_argument("--pressure-r", type=float, default=None,
                         help="Override measurement variance for pressure_hPa (default: auto-estimated)")
    parser.add_argument("--pressure-q", type=float, default=1e-8,
                         help="Process noise intensity for pressure trend (default: 1e-8)")

    parser.add_argument("--ds18b20-r", type=float, default=None,
                         help="Override measurement variance for calibrated ds18b20 temp (default: auto-estimated)")
    parser.add_argument("--ds18b20-q", type=float, default=1e-7,
                         help="Process noise intensity for ds18b20 trend (default: 1e-7)")

    parser.add_argument("--thermistor-r", type=float, default=None,
                         help="Override measurement variance for calibrated thermistor temp (default: auto-estimated)")
    parser.add_argument("--thermistor-q", type=float, default=1e-7,
                         help="Process noise intensity for thermistor trend (default: 1e-7)")

    parser.add_argument("--temp-weight", type=float, default=0.5,
                         help="Weight of the (averaged) temperature signal in the combined index (default: 0.5)")
    parser.add_argument("--pressure-weight", type=float, default=0.5,
                         help="Weight of the pressure signal in the combined index (default: 0.5)")

    parser.add_argument("--pressure-ref", choices=["mean", "first", "last"], default="mean",
                         help="Reference point used for temperature-adjusted pressure: "
                              "'mean'/'first'/'last' reading of that temperature channel (default: mean)")

    parser.add_argument("--show", action="store_true",
                         help="Also display plots interactively (in addition to saving them)")

    args = parser.parse_args()

    df = load_and_prepare(args.input_csv)
    os.makedirs(args.output_dir, exist_ok=True)

    # --- noise estimation ---
    pressure_r = args.pressure_r if args.pressure_r is not None else \
        estimate_measurement_noise(df, "pressure_hPa")
    ds18b20_r = args.ds18b20_r if args.ds18b20_r is not None else \
        estimate_measurement_noise(df, "ds18b20_tempC_corrected")
    thermistor_r = args.thermistor_r if args.thermistor_r is not None else \
        estimate_measurement_noise(df, "thermistor_tempC_calc")

    print("Measurement noise variance (R) in use:")
    print(f"  pressure_hPa               : {pressure_r:.6g}")
    print(f"  ds18b20_tempC_corrected    : {ds18b20_r:.6g}")
    print(f"  thermistor_tempC_calc      : {thermistor_r:.6g}")

    # --- Kalman filter each channel individually ---
    df["pressure_hPa_filtered"] = kalman_filter_series(
        df["t_sec"], df["pressure_hPa"], pressure_r, args.pressure_q)
    df["ds18b20_tempC_filtered"] = kalman_filter_series(
        df["t_sec"], df["ds18b20_tempC_corrected"], ds18b20_r, args.ds18b20_q)
    df["thermistor_tempC_filtered"] = kalman_filter_series(
        df["t_sec"], df["thermistor_tempC_calc"], thermistor_r, args.thermistor_q)

    # --- combined fermentation activity index (ds18b20 + thermistor only; lps33hw temp excluded) ---
    df["avg_temp_filtered"] = df[["ds18b20_tempC_filtered", "thermistor_tempC_filtered"]].mean(axis=1)

    temp_std = df["avg_temp_filtered"].std(ddof=0)
    pressure_std = df["pressure_hPa_filtered"].std(ddof=0)
    df["temp_norm"] = (df["avg_temp_filtered"] - df["avg_temp_filtered"].mean()) / (temp_std if temp_std > 0 else 1.0)
    df["pressure_norm"] = (df["pressure_hPa_filtered"] - df["pressure_hPa_filtered"].mean()) / (pressure_std if pressure_std > 0 else 1.0)

    df["fermentation_activity_index"] = (
        args.temp_weight * df["temp_norm"] + args.pressure_weight * df["pressure_norm"]
    )

    # --- temperature-adjusted pressure (removes thermal-expansion component) ---
    def pick_ref(series):
        if args.pressure_ref == "mean":
            return series.mean()
        elif args.pressure_ref == "first":
            return series.iloc[0]
        else:
            return series.iloc[-1]

    ref_thermistor = pick_ref(df["thermistor_tempC_filtered"])
    ref_ds18b20 = pick_ref(df["ds18b20_tempC_filtered"])

    df["pressure_adjusted_thermistor"] = temperature_adjusted_pressure(
        df["pressure_hPa_filtered"], df["thermistor_tempC_filtered"], ref_thermistor)
    df["pressure_adjusted_ds18b20"] = temperature_adjusted_pressure(
        df["pressure_hPa_filtered"], df["ds18b20_tempC_filtered"], ref_ds18b20)

    print(f"\nTemperature-adjusted pressure reference ({args.pressure_ref}):")
    print(f"  thermistor : {ref_thermistor:.4f} °C")
    print(f"  ds18b20    : {ref_ds18b20:.4f} °C")

    # --- save filtered CSV ---
    out_cols = [
        "true_time", "timestamp", "uptime_ms",
        "pressure_hPa", "pressure_hPa_filtered",
        "ds18b20_tempC", "ds18b20_tempC_corrected", "ds18b20_tempC_filtered",
        "thermistor_mV", "thermistor_tempC_calc", "thermistor_tempC_filtered",
        "lps33hw_tempC",  # reference only, unfiltered, not used in the index
        "avg_temp_filtered", "temp_norm", "pressure_norm", "fermentation_activity_index",
        "pressure_adjusted_thermistor", "pressure_adjusted_ds18b20",
    ]
    out_csv_path = os.path.join(args.output_dir, "filtered_output.csv")
    df[out_cols].to_csv(out_csv_path, index=False)
    print(f"\nSaved filtered data: {out_csv_path}")

    # --- individual plots ---
    fig1 = plot_channel(df, "pressure_hPa", "pressure_hPa_filtered",
                         "Pressure (hPa)", "Pressure — raw vs. Kalman filtered",
                         os.path.join(args.output_dir, "pressure_filtered.png"))
    fig2 = plot_channel(df, "ds18b20_tempC_corrected", "ds18b20_tempC_filtered",
                         "Temperature (°C)", "DS18B20 (calibrated) — raw vs. Kalman filtered",
                         os.path.join(args.output_dir, "ds18b20_filtered.png"))
    fig3 = plot_channel(df, "thermistor_tempC_calc", "thermistor_tempC_filtered",
                         "Temperature (°C)", "Thermistor (calibrated) — raw vs. Kalman filtered",
                         os.path.join(args.output_dir, "thermistor_filtered.png"))
    fig4 = plot_combined(df, os.path.join(args.output_dir, "fermentation_activity_index.png"))
    fig5 = plot_pressure_adjustment(df, "thermistor_tempC_filtered", "pressure_adjusted_thermistor",
                                     "Thermistor", os.path.join(args.output_dir, "pressure_adjusted_thermistor.png"))
    fig6 = plot_pressure_adjustment(df, "ds18b20_tempC_filtered", "pressure_adjusted_ds18b20",
                                     "DS18B20", os.path.join(args.output_dir, "pressure_adjusted_ds18b20.png"))

    print("Saved plots:")
    for name in ["pressure_filtered.png", "ds18b20_filtered.png",
                 "thermistor_filtered.png", "fermentation_activity_index.png",
                 "pressure_adjusted_thermistor.png", "pressure_adjusted_ds18b20.png"]:
        print(f"  {os.path.join(args.output_dir, name)}")

    if args.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()