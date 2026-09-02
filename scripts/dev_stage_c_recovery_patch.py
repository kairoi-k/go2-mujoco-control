from pathlib import Path


header = Path("example/cpp/terrain/terrain_plan_execution_adapter.h")
text = header.read_text()

old = """        adopted_.reset();
        using_plan_ = false;
        fallback_age_steps_ = 0;
"""
new = """        adopted_.reset();
        using_plan_ = false;
        recovery_pending_ = false;
        fallback_age_steps_ = 0;
"""
if text.count(old) != 1:
    raise SystemExit(f"reset patch site count={text.count(old)}")
text = text.replace(old, new)

old = """        for (bool flight : in_flight_)
            if (flight)
                return false;
        return AtEventBoundary(*adopted_, now_s);
"""
new = """        for (bool flight : in_flight_)
            if (flight)
                return false;
        // A safe-stop/expiry retires timing ownership. While the measured
        // contact guard is still active it must keep the immutable snapshot
        // frozen; once the guard clears, recovery itself is an explicit
        // adapter boundary for atomically adopting a fresh whole snapshot.
        if (recovery_pending_ && !contact_guard_active_)
            return true;
        return AtEventBoundary(*adopted_, now_s);
"""
if text.count(old) != 1:
    raise SystemExit(f"boundary patch site count={text.count(old)}")
text = text.replace(old, new)

old = """        const bool same_plan = candidate_valid && adopted_ &&
            SameIdentity(*candidate, *adopted_);
        const bool boundary = event_boundary && IsLegalBoundary(now_s);
        const bool guard_keeps_snapshot = contact_guard_active_ && adopted_ &&
            !same_plan;
        if (candidate_valid && (!guard_keeps_snapshot) &&
            (!adopted_ || same_plan || boundary))
        {
            if (!same_plan)
            {
                adopted_ = std::make_shared<const TerrainMotionPlan>(*candidate);
                result.adopted = true;
                result.adopted_plan_id = candidate->plan_id;
            }
            using_plan_ = true;
            fallback_age_steps_ = 0;
            last_request_ = MakeRequest(*adopted_);
            result.using_plan = true;
            result.request = last_request_;
        }
        else if (candidate_valid && adopted_ && !boundary && !same_plan)
        {
            result.rejected = true;
            result.rejected_plan_id = candidate->plan_id;
            last_rejected_plan_id_ = candidate->plan_id;
            last_rejection_reason_ = "not_at_event_boundary_or_in_flight";
            result.rejection_reason = last_rejection_reason_;
        }
        const bool guard_safe_stop = contact_guard_active_ &&
            contact_guard_age_ticks_ >= 5;
        if (adopted_ && !guard_safe_stop && std::isfinite(now_s) &&
            now_s <= adopted_->valid_until_s + grace_s_)
        {
            using_plan_ = true;
            last_request_ = MakeRequest(*adopted_);
            result.using_plan = true;
            result.request = last_request_;
        }
        else
        {
            using_plan_ = false;
            if (contact_guard_active_)
                fallback_age_steps_ = contact_guard_age_ticks_;
            else
                ++fallback_age_steps_;
            result.fallback_reason = contact_guard_active_
                ? ContactGuardFallbackReason(fallback_age_steps_)
                : MeasuredSupportFallbackReason(fallback_age_steps_);
            last_fallback_reason_ = result.fallback_reason;
            last_request_ = MakeFallbackRequest(
                measured_support, fallback_pattern, fallback_period_s,
                fallback_duty_factor, fallback_step_length_m,
                fallback_foot_lift_m, result.fallback_reason);
            result.request = last_request_;
        }
        UpdateFlightState(last_request_, now_s);
        return result;
"""
new = """        const bool same_plan = candidate_valid && adopted_ &&
            SameIdentity(*candidate, *adopted_);
        const bool guard_safe_stop = contact_guard_active_ &&
            contact_guard_age_ticks_ >= 5;

        // N+5 measured-contact safety has absolute priority over plan
        // adoption. Retain the immutable snapshot only as provenance and
        // retire its execution ownership before considering any candidate.
        if (guard_safe_stop)
        {
            if (adopted_)
                recovery_pending_ = true;
            if (candidate_valid && adopted_ && !same_plan)
            {
                result.rejected = true;
                result.rejected_plan_id = candidate->plan_id;
                last_rejected_plan_id_ = candidate->plan_id;
                last_rejection_reason_ = "contact_guard_safe_stop";
                result.rejection_reason = last_rejection_reason_;
            }
            using_plan_ = false;
            fallback_age_steps_ = contact_guard_age_ticks_;
            result.fallback_reason =
                ContactGuardFallbackReason(fallback_age_steps_);
            last_fallback_reason_ = result.fallback_reason;
            last_request_ = MakeFallbackRequest(
                measured_support, fallback_pattern, fallback_period_s,
                fallback_duty_factor, fallback_step_length_m,
                fallback_foot_lift_m, result.fallback_reason);
            result.request = last_request_;
            UpdateFlightState(last_request_, now_s);
            return result;
        }

        const bool boundary = event_boundary && IsLegalBoundary(now_s);
        const bool guard_keeps_snapshot = contact_guard_active_ && adopted_ &&
            !same_plan;
        if (guard_keeps_snapshot && candidate_valid)
        {
            result.rejected = true;
            result.rejected_plan_id = candidate->plan_id;
            last_rejected_plan_id_ = candidate->plan_id;
            last_rejection_reason_ = "contact_guard_active";
            result.rejection_reason = last_rejection_reason_;
        }
        if (candidate_valid && !guard_keeps_snapshot &&
            (!adopted_ || same_plan || boundary))
        {
            if (!same_plan)
            {
                adopted_ = std::make_shared<const TerrainMotionPlan>(*candidate);
                recovery_pending_ = false;
                result.adopted = true;
                result.adopted_plan_id = candidate->plan_id;
            }
            using_plan_ = true;
            fallback_age_steps_ = 0;
            last_request_ = MakeRequest(*adopted_);
            result.using_plan = true;
            result.request = last_request_;
        }
        else if (candidate_valid && adopted_ && !same_plan &&
                 !result.rejected)
        {
            result.rejected = true;
            result.rejected_plan_id = candidate->plan_id;
            last_rejected_plan_id_ = candidate->plan_id;
            last_rejection_reason_ = "not_at_event_boundary_or_in_flight";
            result.rejection_reason = last_rejection_reason_;
        }

        const bool adopted_expired = adopted_ && std::isfinite(now_s) &&
            now_s > adopted_->valid_until_s + grace_s_;
        if (adopted_expired)
            recovery_pending_ = true;
        const bool hold_for_recovery = recovery_pending_ && !result.adopted;
        if (adopted_ && !hold_for_recovery && std::isfinite(now_s) &&
            now_s <= adopted_->valid_until_s + grace_s_)
        {
            using_plan_ = true;
            last_request_ = MakeRequest(*adopted_);
            result.using_plan = true;
            result.request = last_request_;
        }
        else
        {
            using_plan_ = false;
            if (contact_guard_active_)
                fallback_age_steps_ = contact_guard_age_ticks_;
            else
                ++fallback_age_steps_;
            result.fallback_reason = contact_guard_active_
                ? ContactGuardFallbackReason(fallback_age_steps_)
                : MeasuredSupportFallbackReason(fallback_age_steps_);
            last_fallback_reason_ = result.fallback_reason;
            last_request_ = MakeFallbackRequest(
                measured_support, fallback_pattern, fallback_period_s,
                fallback_duty_factor, fallback_step_length_m,
                fallback_foot_lift_m, result.fallback_reason);
            result.request = last_request_;
        }
        UpdateFlightState(last_request_, now_s);
        return result;
"""
if text.count(old) != 1:
    raise SystemExit(f"Update patch site count={text.count(old)}")
text = text.replace(old, new)

old = """    bool using_plan_ = false;
    std::size_t fallback_age_steps_ = 0;
"""
new = """    bool using_plan_ = false;
    bool recovery_pending_ = false;
    std::size_t fallback_age_steps_ = 0;
"""
if text.count(old) != 1:
    raise SystemExit(f"state-field patch site count={text.count(old)}")
header.write_text(text.replace(old, new))


test = Path("example/cpp/tests/test_terrain_plan_recovery.cpp")
text = test.read_text()
old = """    if (!adapter.IsLegalBoundary(1.05))
        return Fail("retired execution did not expose a recovery boundary");

    adapter.SetContactGuard(false, 0);
"""
new = """    if (adapter.IsLegalBoundary(1.05))
        return Fail("active contact guard exposed a recovery boundary");

    adapter.SetContactGuard(false, 0);
    if (!adapter.IsLegalBoundary(1.05))
        return Fail("cleared guard did not expose the recovery boundary");
"""
if text.count(old) != 1:
    raise SystemExit(f"recovery-test patch site count={text.count(old)}")
test.write_text(text.replace(old, new))


cmake = Path("example/cpp/CMakeLists.txt")
text = cmake.read_text()
marker = """add_executable(test_terrain_plan_fallback tests/test_terrain_plan_fallback.cpp)
target_link_libraries(test_terrain_plan_fallback unitree_sdk2)
go2_add_ctest(test_terrain_plan_fallback)
"""
addition = marker + """
# Stage-C safe-stop recovery must never resume an interrupted stale plan.
add_executable(test_terrain_plan_recovery tests/test_terrain_plan_recovery.cpp)
target_link_libraries(test_terrain_plan_recovery unitree_sdk2)
go2_add_ctest(test_terrain_plan_recovery)
"""
if "add_executable(test_terrain_plan_recovery" not in text:
    if text.count(marker) != 1:
        raise SystemExit(f"CMake patch site count={text.count(marker)}")
    text = text.replace(marker, addition)
cmake.write_text(text)
