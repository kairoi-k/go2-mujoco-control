#!/usr/bin/env python3
"""Offline stateful replay with contact/phase recovery gates and hold auditing."""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path

from analyze_rate_aware_fallback import (
    candidate_ok,
    candidate_reasons,
    finite,
    longest_run,
    read_csv,
    transition_count,
)

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
TORQUE_FIELDS = tuple(
    f"{leg}_{joint}_tau_ff_candidate"
    for leg in LEGS
    for joint in JOINTS
)


def contact_count(row):
    value = finite(row, "selected_contact_count")
    rounded = round(value)
    if abs(value - rounded) > 1e-9 or rounded < 0 or rounded > 4:
        raise ValueError(f"invalid selected contact count: {value}")
    return int(rounded)


def parse_phase_windows(values):
    windows = []
    for raw in values:
        parts = raw.split(":", 1)
        if len(parts) != 2:
            raise ValueError(f"invalid phase window: {raw!r}")
        try:
            start = float(parts[0])
            end = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"invalid phase window: {raw!r}") from exc
        if (
            not math.isfinite(start)
            or not math.isfinite(end)
            or start < 0.0
            or end > 1.0
            or start > end
        ):
            raise ValueError(f"phase window out of range: {raw!r}")
        windows.append((start, end))
    return tuple(windows)


def phase_in_windows(phase, windows):
    return not windows or any(
        start <= phase <= end for start, end in windows
    )


def format_phase_windows(windows):
    if not windows:
        return "none"
    return ",".join(
        f"{start:.6g}:{end:.6g}" for start, end in windows
    )


def torque(row):
    return tuple(finite(row, name) for name in TORQUE_FIELDS)


def cross_rate(candidate, previous, dt_s, rate_limit, tolerance):
    if previous is None:
        return True, 0.0, 0.0
    if dt_s < 0.0:
        raise ValueError("non-positive replay dt")
    delta = max(
        abs(candidate[index] - previous[index])
        for index in range(len(candidate))
    )
    if dt_s <= 1e-12:
        return delta <= tolerance, delta, 0.0
    limit = rate_limit * dt_s
    return delta <= limit + tolerance, delta, limit


def check_fields(replay_fields, fullbody_fields, label):
    replay_required = {
        "row_number",
        "cmd_time_s",
        "phase",
        "shadow_policy_satisfied",
        "shadow_torque_rate_task_active",
        "shadow_torque_rate_satisfied",
        "selected_contact_count",
        *TORQUE_FIELDS,
    }
    fullbody_required = {
        "replay_time_s",
        "task_gate",
        "feasible_candidate",
    }
    missing_replay = replay_required - set(replay_fields)
    missing_fullbody = fullbody_required - set(fullbody_fields)
    if missing_replay or missing_fullbody:
        raise ValueError(
            f"{label} missing replay={','.join(sorted(missing_replay))} "
            f"fullbody={','.join(sorted(missing_fullbody))}"
        )


def collapse_duplicates(rows, truth_rows, tolerance):
    if len(rows) != len(truth_rows):
        raise ValueError("replay/fullbody row counts do not match")
    kept_rows = []
    kept_truth = []
    last_time = None
    for row, truth in zip(rows, truth_rows):
        current_time = finite(row, "cmd_time_s")
        if last_time is not None:
            if current_time < last_time:
                raise ValueError("replay time is not monotonic")
            if current_time - last_time <= tolerance:
                continue
        kept_rows.append(row)
        kept_truth.append(truth)
        last_time = current_time
    return kept_rows, kept_truth


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--moment-replay-csv", type=Path, required=True)
    parser.add_argument("--moment-fullbody-detail-csv", type=Path, required=True)
    parser.add_argument("--force-only-replay-csv", type=Path, required=True)
    parser.add_argument(
        "--force-only-fullbody-detail-csv",
        type=Path,
        required=True,
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
    parser.add_argument("--rate-limit-nm-s", type=float, required=True)
    parser.add_argument("--recovery-rows", type=int, default=2)
    parser.add_argument("--recovery-min-contact-count", type=int, default=0)
    parser.add_argument(
        "--recovery-phase-window",
        action="append",
        default=[],
        metavar="START:END",
    )
    parser.add_argument("--max-hold-rows", type=int, default=0)
    parser.add_argument(
        "--force-only-switch-phase-window",
        action="append",
        default=[],
        metavar="START:END",
    )
    parser.add_argument(
        "--moment-recovery-phase-window",
        action="append",
        default=[],
        metavar="START:END",
    )
    parser.add_argument(
        "--hold-recovery-phase-window",
        action="append",
        default=[],
        metavar="START:END",
    )
    parser.add_argument("--time-tolerance-s", type=float, default=1e-9)
    parser.add_argument("--rate-tolerance-nm", type=float, default=1e-9)
    args = parser.parse_args()
    if (
        args.rate_limit_nm_s <= 0
        or args.recovery_rows <= 0
        or args.recovery_min_contact_count < 0
        or args.recovery_min_contact_count > 4
        or args.max_hold_rows < 0
        or args.time_tolerance_s <= 0
        or args.rate_tolerance_nm < 0
    ):
        print("validation=FAIL: invalid policy parameters")
        return 2

    try:
        recovery_phase_windows = parse_phase_windows(
            args.recovery_phase_window
        )
        force_only_switch_phase_windows = parse_phase_windows(
            args.force_only_switch_phase_window
        )
        moment_recovery_phase_windows = parse_phase_windows(
            args.moment_recovery_phase_window
        )
        hold_recovery_phase_windows = parse_phase_windows(
            args.hold_recovery_phase_window
        )
    except ValueError as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not force_only_switch_phase_windows:
        force_only_switch_phase_windows = recovery_phase_windows
    if not moment_recovery_phase_windows:
        moment_recovery_phase_windows = recovery_phase_windows
    if not hold_recovery_phase_windows:
        hold_recovery_phase_windows = recovery_phase_windows

    recovery_phase_summary = format_phase_windows(recovery_phase_windows)
    force_only_switch_phase_summary = format_phase_windows(
        force_only_switch_phase_windows
    )
    moment_recovery_phase_summary = format_phase_windows(
        moment_recovery_phase_windows
    )
    hold_recovery_phase_summary = format_phase_windows(
        hold_recovery_phase_windows
    )
    try:
        moment, moment_fields = read_csv(args.moment_replay_csv)
        moment_truth, moment_truth_fields = read_csv(
            args.moment_fullbody_detail_csv
        )
        force, force_fields = read_csv(args.force_only_replay_csv)
        force_truth, force_truth_fields = read_csv(
            args.force_only_fullbody_detail_csv
        )
        check_fields(moment_fields, moment_truth_fields, "moment")
        check_fields(force_fields, force_truth_fields, "force-only")
        if len({
            len(moment),
            len(moment_truth),
            len(force),
            len(force_truth),
        }) != 1:
            raise ValueError("replay/fullbody row counts do not match")
        input_rows = len(moment)
        moment, moment_truth = collapse_duplicates(
            moment, moment_truth, args.time_tolerance_s
        )
        force, force_truth = collapse_duplicates(
            force, force_truth, args.time_tolerance_s
        )
        if len(moment) != len(force):
            raise ValueError("candidate unique tick counts do not match")
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    state = "moment"
    previous_time = None
    previous_torque = None
    moment_recovery = 0
    force_recovery = 0
    classes = []
    details = []
    transitions = Counter()
    cross_rejections = Counter()
    hold_run = 0
    hold_timeout_rows = 0
    hold_timeout_events = 0

    for index, (m, mt, f, ft) in enumerate(
        zip(moment, moment_truth, force, force_truth),
        start=1,
    ):
        try:
            mtime = finite(m, "cmd_time_s")
            ftime = finite(f, "cmd_time_s")
            m_phase = finite(m, "phase")
            f_phase = finite(f, "phase")
            if (
                abs(mtime - ftime) > args.time_tolerance_s
                or abs(mtime - finite(mt, "replay_time_s"))
                > args.time_tolerance_s
                or abs(ftime - finite(ft, "replay_time_s"))
                > args.time_tolerance_s
            ):
                raise ValueError("time alignment mismatch")
            if m["row_number"] != f["row_number"]:
                raise ValueError("row number mismatch")
            if abs(m_phase - f_phase) > 1e-9:
                raise ValueError("phase alignment mismatch")
            mtau = torque(m)
            ftau = torque(f)
            m_contact_count = contact_count(m)
            f_contact_count = contact_count(f)
            dt = mtime - previous_time if previous_time is not None else 0.0
            moment_base = candidate_ok(m, mt)
            force_base = candidate_ok(f, ft)
            moment_cont, _, rate_delta = cross_rate(
                mtau,
                previous_torque,
                dt,
                args.rate_limit_nm_s,
                args.rate_tolerance_nm,
            )
            force_cont, _, _ = cross_rate(
                ftau,
                previous_torque,
                dt,
                args.rate_limit_nm_s,
                args.rate_tolerance_nm,
            )
        except (KeyError, ValueError) as exc:
            print(f"validation=FAIL: row {index}: {exc}")
            return 2

        moment_ok = moment_base and moment_cont
        force_ok = force_base and force_cont
        recovery_phase_allowed = phase_in_windows(
            m_phase, recovery_phase_windows
        )
        moment_recovery_phase_allowed = phase_in_windows(
            m_phase, moment_recovery_phase_windows
        )
        force_only_switch_phase_allowed = phase_in_windows(
            f_phase, force_only_switch_phase_windows
        )
        hold_recovery_phase_allowed = phase_in_windows(
            m_phase, hold_recovery_phase_windows
        )
        moment_recovery_ok = (
            moment_ok
            and m_contact_count >= args.recovery_min_contact_count
            and moment_recovery_phase_allowed
        )
        force_only_switch_ok = (
            force_ok
            and f_contact_count >= args.recovery_min_contact_count
            and force_only_switch_phase_allowed
        )
        hold_moment_recovery_ok = (
            moment_ok
            and m_contact_count >= args.recovery_min_contact_count
            and hold_recovery_phase_allowed
        )
        hold_force_only_recovery_ok = (
            force_ok
            and f_contact_count >= args.recovery_min_contact_count
            and hold_recovery_phase_allowed
        )
        if moment_base and not moment_cont:
            cross_rejections["moment"] += 1
        if force_base and not force_cont:
            cross_rejections["force-only"] += 1

        old_state = state
        if state == "moment":
            if moment_ok:
                selected = "moment"
            elif force_only_switch_ok:
                selected, state = "force-only", "force-only"
                moment_recovery = 0
            else:
                selected, state = "hold", "hold"
                moment_recovery = force_recovery = 0
        elif state == "force-only":
            moment_recovery = moment_recovery + 1 if moment_recovery_ok else 0
            if moment_recovery >= args.recovery_rows:
                selected, state = "moment", "moment"
                moment_recovery = 0
            elif force_ok:
                selected = "force-only"
            else:
                selected, state = "hold", "hold"
                moment_recovery = force_recovery = 0
        else:
            moment_recovery = moment_recovery + 1 if hold_moment_recovery_ok else 0
            force_recovery = force_recovery + 1 if hold_force_only_recovery_ok else 0
            if moment_recovery >= args.recovery_rows:
                selected, state = "moment", "moment"
                moment_recovery = force_recovery = 0
            elif force_recovery >= args.recovery_rows:
                selected, state = "force-only", "force-only"
                moment_recovery = force_recovery = 0
            else:
                selected = "hold"

        if selected == "moment":
            selected_torque = mtau
        elif selected == "force-only":
            selected_torque = ftau
        else:
            selected_torque = previous_torque
        if selected_torque is None:
            raise RuntimeError("hold before first selected torque")
        if selected == "hold":
            hold_run += 1
        else:
            hold_run = 0
        hold_timeout = (
            args.max_hold_rows > 0
            and hold_run > args.max_hold_rows
        )
        if hold_timeout:
            hold_timeout_rows += 1
            if hold_run == args.max_hold_rows + 1:
                hold_timeout_events += 1
        selected_delta = (
            max(
                abs(selected_torque[j] - previous_torque[j])
                for j in range(len(selected_torque))
            )
            if previous_torque is not None
            else 0.0
        )
        if (
            selected != "hold"
            and previous_torque is not None
            and selected_delta > rate_delta + args.rate_tolerance_nm
        ):
            print(f"validation=FAIL: selected torque rate at row {index}")
            return 2

        reasons = []
        if not moment_ok:
            reasons.extend(candidate_reasons(m, mt, "moment"))
            if moment_base and not moment_cont:
                reasons.append("moment_cross_rate")
        if not force_ok:
            reasons.extend(candidate_reasons(f, ft, "force_only"))
            if force_base and not force_cont:
                reasons.append("force_only_cross_rate")
        if selected == "hold":
            reasons.append("state_hold")
            if old_state == "moment" and force_ok and not force_only_switch_ok:
                reasons.append("force_only_switch_phase_gate")
            elif old_state == "force-only" and moment_ok and not moment_recovery_ok:
                reasons.append("moment_recovery_phase_gate")
            elif old_state == "hold":
                if moment_ok and not hold_moment_recovery_ok:
                    reasons.append("hold_moment_recovery_phase_gate")
                if force_ok and not hold_force_only_recovery_ok:
                    reasons.append("hold_force_only_recovery_phase_gate")
        transitions[(old_state, state)] += 1
        classes.append(selected)
        details.append(
            {
                "row_number": m["row_number"],
                "cmd_time_s": f"{mtime:.17g}",
                "moment_phase": f"{m_phase:.17g}",
                "force_only_phase": f"{f_phase:.17g}",
                "recovery_phase_allowed": int(recovery_phase_allowed),
                "moment_recovery_phase_allowed": int(moment_recovery_phase_allowed),
                "force_only_switch_phase_allowed": int(force_only_switch_phase_allowed),
                "hold_recovery_phase_allowed": int(hold_recovery_phase_allowed),
                "previous_state": old_state,
                "state": state,
                "classification": selected,
                "moment_candidate_ok": int(moment_ok),
                "force_only_candidate_ok": int(force_ok),
                "moment_recovery_eligible": int(moment_recovery_ok),
                "force_only_switch_eligible": int(force_only_switch_ok),
                "hold_moment_recovery_eligible": int(hold_moment_recovery_ok),
                "hold_force_only_recovery_eligible": int(hold_force_only_recovery_ok),
                "moment_contact_count": m_contact_count,
                "force_only_contact_count": f_contact_count,
                "hold_run_rows": hold_run,
                "hold_timeout": int(hold_timeout),
                "moment_cross_rate_ok": int(moment_cont),
                "force_only_cross_rate_ok": int(force_cont),
                "selected_torque_delta_nm": f"{selected_delta:.17g}",
                "rate_limit_delta_nm": f"{rate_delta:.17g}",
                "moment_recovery_rows": moment_recovery,
                "force_recovery_rows": force_recovery,
                "reasons": "|".join(dict.fromkeys(reasons)),
            }
        )
        previous_time = mtime
        previous_torque = selected_torque

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(details[0]))
        writer.writeheader()
        writer.writerows(details)

    counts = Counter(classes)
    lines = [
        "stateful rate-aware fallback replay",
        "policy=moment_first_force_only_recovery_then_hold_contact_role_phase_gated",
        f"recovery_min_contact_count={args.recovery_min_contact_count}",
        f"recovery_phase_windows={recovery_phase_summary}",
        f"moment_recovery_phase_windows={moment_recovery_phase_summary}",
        f"force_only_switch_phase_windows={force_only_switch_phase_summary}",
        f"hold_recovery_phase_windows={hold_recovery_phase_summary}",
        f"max_hold_rows_policy={args.max_hold_rows}",
        f"hold_timeout_rows={hold_timeout_rows}",
        f"hold_timeout_events={hold_timeout_events}",
        "hold_timeout_is_audit_only=1",
        f"input_rows={input_rows}",
        f"replay_rows={len(details)}",
        f"duplicate_rows_collapsed={input_rows - len(details)}",
        f"moment_rows={counts['moment']}",
        f"force_only_rows={counts['force-only']}",
        f"hold_rows={counts['hold']}",
        f"longest_hold_run={longest_run(classes, 'hold')}",
        f"hold_transition_count={transition_count(classes)}",
        f"moment_cross_rate_rejections={cross_rejections['moment']}",
        f"force_only_cross_rate_rejections={cross_rejections['force-only']}",
    ]
    lines.extend(
        f"transition_{old}_to_{new}={count}"
        for (old, new), count in sorted(transitions.items())
    )
    lines.extend(
        [
            "validation=PASS",
            "interpretation=offline_state_machine_not_runtime_safety_proof",
        ]
    )
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
