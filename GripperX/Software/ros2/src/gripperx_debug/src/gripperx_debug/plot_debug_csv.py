import argparse
import csv
import math
from pathlib import Path


def resolve_csv_path(path_argument: str) -> Path:
    input_path = Path(path_argument).expanduser().resolve()
    if input_path.is_dir():
        csv_files = sorted(input_path.glob("debug_log_*.csv"), key=lambda path: path.stat().st_mtime)
        if not csv_files:
            raise SystemExit(f"No debug CSV files found in {input_path}")
        return csv_files[-1]
    return input_path


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def read_required_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value == "":
        raise SystemExit(f"Required CSV column '{key}' is missing or empty in the selected log.")
    return float(value)


def read_optional_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value != "" else float("nan")


def relative_pose_series(x_values, y_values):
    """Zero out the series based on the FIRST VALID (non-NaN) value."""
    if not x_values or not y_values:
        return [], []
    
    origin_x, origin_y = 0.0, 0.0
    for x, y in zip(x_values, y_values):
        if not math.isnan(x) and not math.isnan(y):
            origin_x = x
            origin_y = y
            break

    return [value - origin_x for value in x_values], [value - origin_y for value in y_values]


def relative_yaw_series(yaw_values):
    if not yaw_values:
        return []

    origin_yaw = 0.0
    for yaw in yaw_values:
        if not math.isnan(yaw):
            origin_yaw = yaw
            break

    return [normalize_angle(value - origin_yaw) for value in yaw_values]


def series_error(series, reference):
    errors = []
    for value, ref in zip(series, reference):
        if math.isnan(value) or math.isnan(ref):
            errors.append(float("nan"))
        else:
            errors.append(value - ref)
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot robot debug CSV logs.")
    parser.add_argument("csv_path", help="Path to the debug CSV log, or a directory containing logs")
    parser.add_argument(
        "--output",
        help="Optional output image path. Defaults to <csv_path stem>_plot.png",
    )
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise SystemExit(
            "matplotlib is required for plotting. Install python3-matplotlib and try again."
        ) from error

    csv_path = resolve_csv_path(args.csv_path)
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else csv_path.with_name(f"{csv_path.stem}_plot.png")
    )

    time_s = []
    gt_x = []
    gt_y = []
    gt_yaw = []
    wheel_x = []
    wheel_y = []
    laser_x = []
    laser_y = []
    ekf_x = []
    ekf_y = []
    ekf_yaw = []

    with csv_path.open("r", encoding="ascii") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            time_s.append(read_required_float(row, "time_s"))
            gt_x.append(read_required_float(row, "gt_x"))
            gt_y.append(read_required_float(row, "gt_y"))
            gt_yaw.append(read_required_float(row, "gt_yaw"))
            ekf_x.append(read_required_float(row, "ekf_x"))
            ekf_y.append(read_required_float(row, "ekf_y"))
            ekf_yaw.append(read_required_float(row, "ekf_yaw"))
            
            # Handle potentially missing optional odometry sources safely
            wheel_x.append(read_optional_float(row, "wheel_x"))
            wheel_y.append(read_optional_float(row, "wheel_y"))
            laser_x.append(read_optional_float(row, "laser_x"))
            laser_y.append(read_optional_float(row, "laser_y"))

    if time_s:
        start = time_s[0]
        time_s = [value - start for value in time_s]

    gt_x_rel, gt_y_rel = relative_pose_series(gt_x, gt_y)
    ekf_x_rel, ekf_y_rel = relative_pose_series(ekf_x, ekf_y)
    wheel_x_rel, wheel_y_rel = relative_pose_series(wheel_x, wheel_y)
    laser_x_rel, laser_y_rel = relative_pose_series(laser_x, laser_y)
    gt_yaw_rel = relative_yaw_series(gt_yaw)
    ekf_yaw_rel = relative_yaw_series(ekf_yaw)

    err_x = [ekf - gt for ekf, gt in zip(ekf_x_rel, gt_x_rel)]
    err_y = [ekf - gt for ekf, gt in zip(ekf_y_rel, gt_y_rel)]
    err_yaw = [normalize_angle(ekf - gt) for ekf, gt in zip(ekf_yaw_rel, gt_yaw_rel)]
    wheel_err_x = series_error(wheel_x_rel, gt_x_rel)
    wheel_err_y = series_error(wheel_y_rel, gt_y_rel)
    laser_err_x = series_error(laser_x_rel, gt_x_rel)
    laser_err_y = series_error(laser_y_rel, gt_y_rel)

    figure, axes = plt.subplots(3, 2, figsize=(14, 12))

    # Re-aligned colors to match your original images + added Laser
    axes[0, 0].plot(gt_x_rel, gt_y_rel, label="Ground truth", color="tab:blue", linewidth=2.0, zorder=10)
    axes[0, 0].plot(ekf_x_rel, ekf_y_rel, label="EKF", color="tab:orange", alpha=0.9)
    axes[0, 0].plot(wheel_x_rel, wheel_y_rel, label="Wheel odom", alpha=0.65, color="tab:green")
    axes[0, 0].plot(laser_x_rel, laser_y_rel, label="Laser odom", alpha=0.65, color="tab:red")
    
    axes[0, 0].set_title("XY Trajectory")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()

    axes[0, 1].plot(time_s, err_x, label="x error")
    axes[0, 1].plot(time_s, err_y, label="y error")
    axes[0, 1].set_title("Position Error (EKF vs Ground Truth)")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("error [m]")
    axes[0, 1].legend()

    axes[1, 0].plot(time_s, err_yaw, label="yaw error")
    axes[1, 0].set_title("Yaw Error (EKF vs Ground Truth)")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("error [rad]")
    axes[1, 0].legend()

    axes[1, 1].plot(time_s, gt_x_rel, label="gt x")
    axes[1, 1].plot(time_s, ekf_x_rel, label="ekf x", alpha=0.8)
    axes[1, 1].plot(time_s, gt_y_rel, label="gt y")
    axes[1, 1].plot(time_s, ekf_y_rel, label="ekf y", alpha=0.8)
    axes[1, 1].set_title("Trajectory Components")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("position [m]")
    axes[1, 1].legend()

    axes[2, 0].plot(time_s, wheel_err_x, label="Wheel odom x", color="tab:green", alpha=0.8)
    axes[2, 0].plot(time_s, laser_err_x, label="Laser odom x", color="tab:red", alpha=0.8)
    axes[2, 0].plot(time_s, err_x, label="EKF x", color="tab:orange", alpha=0.9)
    axes[2, 0].set_title("Source X Error")
    axes[2, 0].set_xlabel("time [s]")
    axes[2, 0].set_ylabel("error [m]")
    axes[2, 0].legend()

    axes[2, 1].plot(time_s, wheel_err_y, label="Wheel odom y", color="tab:green", alpha=0.8)
    axes[2, 1].plot(time_s, laser_err_y, label="Laser odom y", color="tab:red", alpha=0.8)
    axes[2, 1].plot(time_s, err_y, label="EKF y", color="tab:orange", alpha=0.9)
    axes[2, 1].set_title("Source Y Error")
    axes[2, 1].set_xlabel("time [s]")
    axes[2, 1].set_ylabel("error [m]")
    axes[2, 1].legend()

    figure.tight_layout()
    figure.savefig(output_path)
    print(f"Plot saved to: {output_path}")


if __name__ == "__main__":
    main()
