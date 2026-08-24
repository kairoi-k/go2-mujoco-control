"""Print compact metrics for one WBC speed-probe CSV."""
import csv
import math
import statistics
import sys


def main() -> None:
    rows = []
    with open(sys.argv[1], newline="") as handle:
        for row in csv.DictReader(handle):
            numeric = {}
            for key, value in row.items():
                if value == "":
                    continue
                try:
                    numeric[key] = float(value)
                except ValueError:
                    pass
            if "world_velocity_x_mps" in numeric:
                rows.append(numeric)
    cruise = [r for r in rows if r.get("world_velocity_x_mps", 0.0) > 1.0]
    if not cruise:
        print(f"rows={len(rows)} cruise=0")
        return
    def mean(key: str) -> float:
        return statistics.mean(r.get(key, 0.0) for r in cruise)
    p95_angle = max(
        sorted(abs(r.get("imu_roll_rad", 0.0)) for r in cruise)[int(.95 * len(cruise))],
        sorted(abs(r.get("imu_pitch_rad", 0.0)) for r in cruise)[int(.95 * len(cruise))],
    ) * 180.0 / math.pi
    print(
        f"rows={len(rows)} cruise={len(cruise)} "
        f"max_v={max(r['world_velocity_x_mps'] for r in cruise):.3f} "
        f"median_v={statistics.median(r['world_velocity_x_mps'] for r in cruise):.3f} "
        f"angle_p95_deg={p95_angle:.2f} "
        f"qdd_x_mean={mean('wbc_full_id_qdd_x_mps2'):.3f} "
        f"srbd_ax_mean={mean('wbc_full_srbd_acc_x_mps2'):.3f} "
        f"contact_fx_mean={mean('wbc_full_id_contact_force_x_n'):.3f}"
    )


if __name__ == "__main__":
    main()
