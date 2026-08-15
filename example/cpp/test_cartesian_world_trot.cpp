#include <cmath>
#include <iostream>

#include "cartesian_world_trot.h"

namespace
{

bool Near(double actual, double expected, double tol = 1e-6)
{
    return std::abs(actual - expected) <= tol;
}

bool CheckScheduleWorkspace()
{
    const double speeds[] = {0.15, 0.50, 1.00, 1.50, 2.00, 2.45};
    for (double v : speeds)
    {
        const auto s = go2_control::ScheduleRunningTrot(v);
        const double travel = v * s.stance_time_s;
        if (!(s.period_s >= 0.22 && s.period_s <= 0.60) ||
            !(s.duty_factor >= 0.38 && s.duty_factor <= 0.75) ||
            travel > 0.250)
        {
            std::cerr << "schedule v=" << v
                      << " T=" << s.period_s
                      << " duty=" << s.duty_factor
                      << " Tst=" << s.stance_time_s
                      << " travel=" << travel << "\n";
            return false;
        }
    }
    const auto slow = go2_control::ScheduleRunningTrot(0.15);
    if (!Near(slow.duty_factor, 0.55, 0.02) || !Near(slow.period_s, 0.40, 0.02))
        return false;
    const auto mid = go2_control::ScheduleRunningTrot(1.0);
    if (!(mid.duty_factor < 0.55) || mid.stance_time_s > 0.23)
        return false;
    const auto fast = go2_control::ScheduleRunningTrot(2.0);
    if (!(fast.duty_factor < 0.50) || !(fast.stance_time_s <= 0.13))
        return false;
    return go2_control::CartesianWorkspaceSpeedCap(0.75, 0.60) > 0.50 &&
           go2_control::CartesianWorkspaceSpeedCap(0.40, 0.28) > 1.90;
}

bool CheckWorldToBodyRoundtrip()
{
    const go2::Vec3 base{1.0, -0.2, 0.42};
    const std::array<double, 4> q{1.0, 0.0, 0.0, 0.0};
    const go2::Vec3 world{1.10, -0.05, 0.02};
    const go2::Vec3 body = go2_control::WorldToBody(base, q, world);
    const go2::Vec3 back = go2_control::BodyToWorld(base, q, body);
    return Near(body.x, 0.10) && Near(body.y, 0.15) && Near(body.z, -0.40) &&
           Near(back.x, world.x) && Near(back.y, world.y) && Near(back.z, world.z);
}

bool CheckSlowBodyRearwardTouchdown()
{
    go2_control::WorldTouchdownInput in;
    in.hip_world = {0.0, 0.0, 0.0};
    in.vx_world = 0.20;
    in.vx_des_world = 1.00;
    in.stance_time_s = 0.12;
    in.velocity_gain_s = 0.08;
    in.max_adjustment_m = 0.14;
    in.ground_z = 0.02;
    const go2::Vec3 td = go2_control::PlanWorldTouchdown(in);
    const go2::Vec3 tracked = go2_control::PlanWorldTouchdown(
        []() {
            go2_control::WorldTouchdownInput ok;
            ok.hip_world = {0.0, 0.0, 0.0};
            ok.vx_world = 2.0;
            ok.vx_des_world = 2.0;
            ok.stance_time_s = 0.12;
            ok.ground_z = 0.02;
            return ok;
        }());
    return td.x < 0.5 * 0.20 * 0.12 && tracked.x > 0.10 &&
           Near(tracked.z, 0.02);
}

bool CheckStanceHoldsWorldAnchor()
{
    go2_control::CartesianWorldInput in;
    in.base = {0.0, 0.0, 0.42};
    in.quaternion = {1.0, 0.0, 0.0, 0.0};
    in.phase = 0.10;
    in.duty_factor = 0.75;
    in.period_s = 0.60;
    in.v_cmd_mps = 0.15;
    in.blend = 1.0;
    in.stand_body_feet = go2::AllFootPositions(
        {0.00571868, 0.608813, -1.21763,
         -0.00571868, 0.608813, -1.21763,
         0.00571868, 0.608813, -1.21763,
         -0.00571868, 0.608813, -1.21763});
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        in.actual_world_feet[leg] = go2_control::BodyToWorld(
            in.base, in.quaternion, in.stand_body_feet[leg]);

    go2_control::CartesianWorldState state;
    std::array<go2::Vec3, go2::kLegCount> feet{};
    go2_control::ApplyCartesianWorldTrot(in, state, feet);

    in.base.x = 0.08;
    in.phase = 0.20;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        in.actual_world_feet[leg].x += 0.08;
    go2_control::ApplyCartesianWorldTrot(in, state, feet);

    const std::size_t fr = static_cast<std::size_t>(go2::Leg::FR);
    if (!state.in_stance[fr])
        return false;
    // Body moved +0.08 m; planted foot body x must move -0.08 m.
    const double x0 = in.stand_body_feet[fr].x;
    return Near(feet[fr].x, x0 - 0.08, 0.02);
}

bool CheckSwingVelocityForward()
{
    const go2::Vec3 start{0.0, 0.1, 0.02};
    const go2::Vec3 end{0.20, 0.1, 0.02};
    const go2::Vec3 v = go2_control::SwingWorldVelocity(
        start, end, 0.40, 0.03, 0.18);
    const go2::Vec3 mid = go2_control::SwingWorldTarget(
        start, end, 0.40, 0.03);
    return v.x > 0.4 && mid.x > start.x && mid.x < end.x && mid.z > 0.03;
}

}  // namespace

int main()
{
    if (!CheckScheduleWorkspace() ||
        !CheckWorldToBodyRoundtrip() ||
        !CheckSlowBodyRearwardTouchdown() ||
        !CheckStanceHoldsWorldAnchor() ||
        !CheckSwingVelocityForward())
    {
        std::cerr << "cartesian world trot checks failed\n";
        return 1;
    }
    std::cout << "cartesian world trot checks passed.\n";
    return 0;
}
