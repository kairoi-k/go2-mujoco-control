#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "go2_contact_torque_mapping.h"
#include "contact_wrench_projected_allocator.h"
#include "contact_wrench_lexicographic_allocator.h"
#include "dynamic_acceleration_target.h"

namespace
{
constexpr double kGroundTruthMatchToleranceS = 0.0011;

constexpr std::size_t kMissing = std::numeric_limits<std::size_t>::max();
constexpr std::array<const char *, go2::kLegCount> kLegNames = {
    "FR", "FL", "RR", "RL"};
constexpr std::array<const char *, go2::kJointsPerLeg> kJointNames = {
    "hip", "thigh", "calf"};

struct Options
{
    std::string csv_path;
    std::string ground_truth_csv;
    std::string output_path;
    double period_s = 0.70;
    double duty_factor = 0.75;
    double mass_kg = 15.206408;
    double gravity_mps2 = 9.81;
    double friction_coefficient = 0.5;
    double max_normal_force = 100.0;
    std::string contact_source = "gait";
    std::string wrench_source = "gravity";
    double dynamic_accel_correction_limit_mps2 = 2.0;
    double dynamic_accel_correction_slew_limit_mps3 = 20.0;
    bool reduced_contact_task = false;
    bool force_only_task = false;
    bool contact_conditioned_slack = false;
    double force_tolerance_n = 0.01;
    double moment_slack_nm = 0.05;
    bool torque_rate_shadow = false;
    double torque_rate_limit_nm_s = 3000.0;
    double torque_rate_weight = 1.0;
};

struct Columns
{
    std::size_t phase = kMissing;
    std::size_t plan_valid = kMissing;
    std::size_t actual_contact_count = kMissing;
    std::size_t cmd_time_s = kMissing;
    std::size_t wbc_shadow_desired_force_x_n = kMissing;
    std::array<std::size_t, go2::kLegCount> actual_contact{};
    bool has_actual_contact_mask = false;
    std::array<std::size_t, go2::kLegCount> foot_force{};
    bool has_foot_force = false;
    std::array<std::size_t, 3> imu_acceleration{};
    bool has_imu_acceleration = false;
    std::array<std::array<std::size_t, go2::kJointsPerLeg>, go2::kLegCount>
        q_state{};
};

struct Summary
{
    std::size_t total_rows = 0;
    std::size_t skipped_rows = 0;
    std::size_t analyzed_rows = 0;
    std::size_t solver_failures = 0;
    std::size_t mapping_failures = 0;
    std::size_t wrench_unsatisfied_rows = 0;
    std::size_t task_unsatisfied_rows = 0;
    std::size_t reduced_task_rows = 0;
    std::size_t reduced_task_unsatisfied_rows = 0;
    std::size_t constraint_failures = 0;
    std::size_t contact_count_mismatch_rows = 0;
    std::size_t actual_contact_mask_rows = 0;
    std::size_t contact_mask_mismatch_rows = 0;
    std::size_t selected_two_contact_rows = 0;
    std::size_t selected_four_contact_rows = 0;
    double max_residual = 0.0;
    double max_constraint_violation = 0.0;
    double max_force_component = 0.0;
    double max_abs_torque = 0.0;
    double max_task_residual = 0.0;
    std::size_t measured_wrench_rows = 0;
    double max_measured_fz_error = 0.0;
    std::size_t dynamic_target_failures = 0;
    double max_measured_wrench_residual = 0.0;
    std::size_t ground_truth_match_failures = 0;
    std::size_t shadow_policy_unsatisfied_rows = 0;
    std::size_t shadow_fallback_rows = 0;
    double max_shadow_force_excess = 0.0;
    double max_shadow_moment_excess = 0.0;
    std::size_t shadow_torque_rate_unsatisfied_rows = 0;
    double max_shadow_torque_rate_excess = 0.0;
};

struct ShadowRow
{
    bool active = false;
    bool policy_satisfied = true;
    bool moment_task_active = false;
    bool torque_rate_task_active = false;
    bool torque_rate_satisfied = true;
    double max_torque_rate_excess = 0.0;
    bool fallback_to_force_solution = false;
    double max_force_excess = 0.0;
    double max_moment_excess = 0.0;
    std::array<double, 3> force_slack{};
    std::array<double, 3> moment_slack{};
};

struct GroundTruthColumns
{
    std::size_t time_s = kMissing;
    std::array<std::size_t, 4> quaternion{};
    std::array<std::size_t, 3> qacc_world{};
    std::array<std::size_t, 6> mass_qacc{};
    std::array<std::size_t, 36> mass_matrix{};
    std::array<std::size_t, 6> smooth{};
};

struct GroundTruthRow
{
    double time_s = 0.0;
    std::array<double, 4> quaternion{};
    std::array<double, 3> qacc_world{};
    std::array<double, 6> mass_qacc{};
    std::array<double, 36> mass_matrix{};
    std::array<double, 6> smooth{};
};

bool ParseDouble(const std::string &text, double &value)
{
    try
    {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        return consumed == text.size() && std::isfinite(value);
    }
    catch (...)
    {
        return false;
    }
}

bool ParseOptionDouble(
    int argc,
    char **argv,
    int &index,
    double &value)
{
    if (index + 1 >= argc)
        return false;
    ++index;
    return ParseDouble(argv[index], value);
}

bool ParseOptions(int argc, char **argv, Options &options)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--csv")
        {
            if (index + 1 >= argc)
                return false;
            options.csv_path = argv[++index];
        }
        else if (argument == "--ground-truth-csv")
        {
            if (index + 1 >= argc)
                return false;
            options.ground_truth_csv = argv[++index];
        }
        else if (argument == "--out")
        {
            if (index + 1 >= argc)
                return false;
            options.output_path = argv[++index];
        }
        else if (argument == "--period")
        {
            if (!ParseOptionDouble(argc, argv, index, options.period_s))
                return false;
        }
        else if (argument == "--duty")
        {
            if (!ParseOptionDouble(argc, argv, index, options.duty_factor))
                return false;
        }
        else if (argument == "--mass-kg")
        {
            if (!ParseOptionDouble(argc, argv, index, options.mass_kg))
                return false;
        }
        else if (argument == "--gravity")
        {
            if (!ParseOptionDouble(argc, argv, index, options.gravity_mps2))
                return false;
        }
        else if (argument == "--mu")
        {
            if (!ParseOptionDouble(
                    argc, argv, index, options.friction_coefficient))
                return false;
        }
        else if (argument == "--max-normal-force")
        {
            if (!ParseOptionDouble(
                    argc, argv, index, options.max_normal_force))
                return false;
        }
        else if (argument == "--contact-source")
        {
            if (index + 1 >= argc)
                return false;
            options.contact_source = argv[++index];
        }
        else if (argument == "--wrench-source")
        {
            if (index + 1 >= argc)
                return false;
            options.wrench_source = argv[++index];
        }
        else if (argument == "--dynamic-accel-correction-limit-mps2")
        {
            if (!ParseOptionDouble(
                    argc, argv, index,
                    options.dynamic_accel_correction_limit_mps2))
                return false;
        }
        else if (argument == "--dynamic-accel-correction-slew-mps3")
        {
            if (!ParseOptionDouble(
                    argc, argv, index,
                    options.dynamic_accel_correction_slew_limit_mps3))
                return false;
        }
        else if (argument == "--reduced-contact-task")
        {
            options.reduced_contact_task = true;
        }
        else if (argument == "--force-only-task")
        {
            options.force_only_task = true;
        }
        else if (argument == "--contact-conditioned-slack")
        {
            options.contact_conditioned_slack = true;
        }
        else if (argument == "--force-tolerance-n")
        {
            if (!ParseOptionDouble(argc, argv, index, options.force_tolerance_n))
                return false;
        }
        else if (argument == "--moment-slack-nm")
        {
            if (!ParseOptionDouble(argc, argv, index, options.moment_slack_nm))
                return false;
        }
        else if (argument == "--torque-rate-shadow")
        {
            options.torque_rate_shadow = true;
        }
        else if (argument == "--torque-rate-limit-nm-s")
        {
            if (!ParseOptionDouble(
                    argc, argv, index, options.torque_rate_limit_nm_s))
                return false;
        }
        else if (argument == "--torque-rate-weight")
        {
            if (!ParseOptionDouble(
                    argc, argv, index, options.torque_rate_weight))
                return false;
        }
        else
        {
            return false;
        }
    }

    return !options.csv_path.empty() &&
           !options.output_path.empty() &&
           std::isfinite(options.period_s) &&
           options.period_s > 0.0 &&
           std::isfinite(options.duty_factor) &&
           options.duty_factor > 0.0 &&
           options.duty_factor < 1.0 &&
           std::isfinite(options.mass_kg) &&
           options.mass_kg > 0.0 &&
           std::isfinite(options.gravity_mps2) &&
           options.gravity_mps2 > 0.0 &&
           std::isfinite(options.friction_coefficient) &&
           options.friction_coefficient >= 0.0 &&
           std::isfinite(options.max_normal_force) &&
           options.max_normal_force > 0.0 &&
           (options.contact_source == "gait" ||
            options.contact_source == "actual") &&
           (options.wrench_source == "gravity" ||
            options.wrench_source == "imu" ||
            options.wrench_source == "state" ||
            options.wrench_source == "dynamic-accel" ||
            options.wrench_source == "dynamic-accel-bounded") &&
           (options.wrench_source != "dynamic-accel" &&
            options.wrench_source != "dynamic-accel-bounded" ||
            !options.ground_truth_csv.empty()) &&
           std::isfinite(options.dynamic_accel_correction_limit_mps2) &&
           options.dynamic_accel_correction_limit_mps2 >= 0.0 &&
           std::isfinite(options.dynamic_accel_correction_slew_limit_mps3) &&
           options.dynamic_accel_correction_slew_limit_mps3 >= 0.0 &&
           std::isfinite(options.force_tolerance_n) &&
           options.force_tolerance_n >= 0.0 &&
           std::isfinite(options.moment_slack_nm) &&
           options.moment_slack_nm >= 0.0 &&
           (!options.torque_rate_shadow ||
            options.contact_conditioned_slack) &&
           std::isfinite(options.torque_rate_limit_nm_s) &&
           options.torque_rate_limit_nm_s >= 0.0 &&
           std::isfinite(options.torque_rate_weight) &&
           options.torque_rate_weight > 0.0;
}

std::vector<std::string> SplitCsvLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
    {
        if (!field.empty() && field.back() == '\r')
            field.pop_back();
        fields.push_back(field);
    }
    return fields;
}

bool BuildColumns(
    const std::vector<std::string> &header,
    Columns &columns)
{
    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < header.size(); ++index)
        indices.emplace(header[index], index);

    const auto find = [&](const std::string &name) {
        const auto it = indices.find(name);
        return it == indices.end() ? kMissing : it->second;
    };

    columns.phase = find("phase");
    columns.plan_valid = find("kernel_footstep_plan_valid");
    columns.actual_contact_count = find("contact_count");
    columns.cmd_time_s = find("cmd_time_s");
    columns.wbc_shadow_desired_force_x_n =
        find("wbc_shadow_desired_force_x_n");
    columns.has_actual_contact_mask = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const std::string name =
            std::string("contact_") + kLegNames[leg];
        columns.actual_contact[leg] = find(name);
        if (columns.actual_contact[leg] == kMissing)
            columns.has_actual_contact_mask = false;
    }
    columns.has_foot_force = true;
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        const std::string name =
            std::string("foot_force_") + kLegNames[leg];
        columns.foot_force[leg] = find(name);
        if (columns.foot_force[leg] == kMissing)
            columns.has_foot_force = false;
    }
    columns.has_imu_acceleration = true;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const std::string name =
            std::string("imu_accel_") +
            (axis == 0 ? "x_mps2" : axis == 1 ? "y_mps2" : "z_mps2");
        columns.imu_acceleration[axis] = find(name);
        if (columns.imu_acceleration[axis] == kMissing)
            columns.has_imu_acceleration = false;
    }
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        for (std::size_t joint = 0;
             joint < go2::kJointsPerLeg;
             ++joint)
        {
            const std::string name =
                std::string(kLegNames[leg]) + "_" +
                kJointNames[joint] + "_q_state";
            columns.q_state[leg][joint] = find(name);
        }
    }

    if (columns.phase == kMissing ||
        columns.plan_valid == kMissing ||
        columns.actual_contact_count == kMissing ||
        columns.cmd_time_s == kMissing)
    {
        return false;
    }
    for (const auto &leg_columns : columns.q_state)
    {
        for (std::size_t index : leg_columns)
        {
            if (index == kMissing)
                return false;
        }
    }
    return true;
}

bool ReadField(
    const std::vector<std::string> &fields,
    std::size_t index,
    double &value)
{
    return index < fields.size() && ParseDouble(fields[index], value);
}

bool BuildGroundTruthColumns(
    const std::vector<std::string> &header,
    GroundTruthColumns &columns)
{
    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < header.size(); ++index)
        indices.emplace(header[index], index);
    const auto find = [&](const std::string &name) {
        const auto it = indices.find(name);
        return it == indices.end() ? kMissing : it->second;
    };

    columns.time_s = find("time_s");
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const char *axis_name = axis == 0 ? "x" : axis == 1 ? "y" : "z";
        columns.qacc_world[axis] =
            find(std::string("base_qacc_world_") + axis_name + "_mps2");
    }
    for (std::size_t axis = 0; axis < 4; ++axis)
    {
        const char *name =
            axis == 0 ? "w" : axis == 1 ? "x" : axis == 2 ? "y" : "z";
        columns.quaternion[axis] =
            find(std::string("base_quat_") + name);
    }

    const std::array<const char *, 6> suffixes = {
        "trans_x_N",
        "trans_y_N",
        "trans_z_N",
        "rot_x_Nm",
        "rot_y_Nm",
        "rot_z_Nm"};
    for (std::size_t index = 0; index < suffixes.size(); ++index)
    {
        columns.mass_qacc[index] = find(
            std::string("base_mass_qacc_qfrc_qcoord_") + suffixes[index]);
        columns.smooth[index] = find(
            std::string("base_qfrc_smooth_qcoord_") + suffixes[index]);
    }
    for (std::size_t row = 0; row < 6; ++row)
    {
        for (std::size_t column = 0; column < 6; ++column)
        {
            columns.mass_matrix[row * 6 + column] = find(
                "base_mass_matrix_qcoord_r" +
                std::to_string(row) + "c" + std::to_string(column));
        }
    }

    if (columns.time_s == kMissing)
        return false;
    for (std::size_t index : columns.quaternion)
        if (index == kMissing)
            return false;
    for (std::size_t index : columns.qacc_world)
        if (index == kMissing)
            return false;
    for (std::size_t index : columns.mass_qacc)
        if (index == kMissing)
            return false;
    for (std::size_t index : columns.mass_matrix)
        if (index == kMissing)
            return false;
    for (std::size_t index : columns.smooth)
        if (index == kMissing)
            return false;
    return true;
}

bool LoadGroundTruth(
    const std::string &path,
    std::vector<GroundTruthRow> &rows)
{
    std::ifstream input(path);
    if (!input)
        return false;
    std::string line;
    if (!std::getline(input, line))
        return false;
    GroundTruthColumns columns;
    if (!BuildGroundTruthColumns(SplitCsvLine(line), columns))
        return false;

    double previous_time = -std::numeric_limits<double>::infinity();
    while (std::getline(input, line))
    {
        const std::vector<std::string> fields = SplitCsvLine(line);
        GroundTruthRow row;
        if (!ReadField(fields, columns.time_s, row.time_s) ||
            row.time_s <= previous_time)
            return false;
        previous_time = row.time_s;
        for (std::size_t index = 0; index < row.quaternion.size(); ++index)
        {
            if (!ReadField(
                    fields, columns.quaternion[index],
                    row.quaternion[index]))
                return false;
        }
        for (std::size_t index = 0; index < row.qacc_world.size(); ++index)
        {
            if (!ReadField(
                    fields, columns.qacc_world[index],
                    row.qacc_world[index]))
                return false;
        }
        for (std::size_t index = 0; index < row.mass_qacc.size(); ++index)
        {
            if (!ReadField(
                    fields, columns.mass_qacc[index],
                    row.mass_qacc[index]))
                return false;
            if (!ReadField(
                    fields, columns.smooth[index],
                    row.smooth[index]))
                return false;
        }
        for (std::size_t index = 0; index < row.mass_matrix.size(); ++index)
        {
            if (!ReadField(
                    fields, columns.mass_matrix[index],
                    row.mass_matrix[index]))
                return false;
        }
        rows.push_back(row);
    }
    return !rows.empty();
}

const GroundTruthRow *MatchGroundTruth(
    const std::vector<GroundTruthRow> &rows,
    double time_s,
    double tolerance_s)
{
    const auto iterator = std::lower_bound(
        rows.begin(),
        rows.end(),
        time_s,
        [](const GroundTruthRow &row, double value) {
            return row.time_s < value;
        });
    const GroundTruthRow *best = nullptr;
    double best_error = std::numeric_limits<double>::infinity();
    if (iterator != rows.end())
    {
        best = &*iterator;
        best_error = std::abs(iterator->time_s - time_s);
    }
    if (iterator != rows.begin())
    {
        const auto previous = iterator - 1;
        const double error = std::abs(previous->time_s - time_s);
        if (error < best_error)
        {
            best = &*previous;
            best_error = error;
        }
    }
    return best != nullptr && best_error <= tolerance_s ? best : nullptr;
}

std::array<double, 3> RotateBodyToWorld(
    const std::array<double, 4> &quaternion,
    const std::array<double, 3> &vector)
{
    const double norm = std::sqrt(
        quaternion[0] * quaternion[0] +
        quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] +
        quaternion[3] * quaternion[3]);
    if (!std::isfinite(norm) || norm <= 1e-12)
        return {0.0, 0.0, 0.0};
    const double w = quaternion[0] / norm;
    const double x = quaternion[1] / norm;
    const double y = quaternion[2] / norm;
    const double z = quaternion[3] / norm;
    const double vx = vector[0];
    const double vy = vector[1];
    const double vz = vector[2];
    return {
        (1.0 - 2.0 * (y * y + z * z)) * vx +
            2.0 * (x * y - z * w) * vy +
            2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx +
            (1.0 - 2.0 * (x * x + z * z)) * vy +
            2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx +
            2.0 * (y * z + x * w) * vy +
            (1.0 - 2.0 * (x * x + y * y)) * vz};
}

std::array<double, 3> RotateWorldToBody(
    const std::array<double, 4> &quaternion,
    const std::array<double, 3> &vector)
{
    const std::array<double, 4> conjugate = {
        quaternion[0],
        -quaternion[1],
        -quaternion[2],
        -quaternion[3]};
    return RotateBodyToWorld(conjugate, vector);
}

std::array<double, 6> BuildDynamicAccelerationWrench(
    const GroundTruthRow &ground_truth,
    double desired_force_x_n,
    double mass_kg)
{
    const std::array<double, 3> desired_acceleration_body = {
        desired_force_x_n / mass_kg, 0.0, 0.0};
    const std::array<double, 3> desired_acceleration_world =
        RotateBodyToWorld(
            ground_truth.quaternion,
            desired_acceleration_body);
    std::array<double, 3> delta_acceleration{};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        delta_acceleration[axis] =
            desired_acceleration_world[axis] -
            ground_truth.qacc_world[axis];
    }

    std::array<double, 6> target_qforce{};
    for (std::size_t row = 0; row < 6; ++row)
    {
        target_qforce[row] =
            ground_truth.mass_qacc[row] - ground_truth.smooth[row];
        for (std::size_t column = 0; column < 3; ++column)
        {
            target_qforce[row] +=
                ground_truth.mass_matrix[row * 6 + column] *
                delta_acceleration[column];
        }
    }

    const std::array<double, 3> target_force_world = {
        target_qforce[0], target_qforce[1], target_qforce[2]};
    const std::array<double, 3> target_moment_world = {
        target_qforce[3], target_qforce[4], target_qforce[5]};
    const std::array<double, 3> target_force_body =
        RotateWorldToBody(
            ground_truth.quaternion,
            target_force_world);
    const std::array<double, 3> target_moment_body =
        RotateWorldToBody(
            ground_truth.quaternion,
            target_moment_world);
    return {
        target_force_body[0],
        target_force_body[1],
        target_force_body[2],
        target_moment_body[0],
        target_moment_body[1],
        target_moment_body[2]};
}

double LegPhase(double phase, std::size_t leg)
{
    const bool diagonal_pair_b =
        leg == static_cast<std::size_t>(go2::Leg::FL) ||
        leg == static_cast<std::size_t>(go2::Leg::RR);
    double leg_phase = phase + (diagonal_pair_b ? 0.5 : 0.0);
    leg_phase -= std::floor(leg_phase);
    return leg_phase;
}

void UpdateSummary(
    const go2_control::ProjectedContactWrenchSolution &wrench_solution,
    const go2_control::ContactTorqueMapSolution &torque_solution,
    int inferred_contacts,
    int actual_contacts,
    bool has_actual_contact_mask,
    bool contact_mask_match,
    bool reduced_task_requested,
    Summary &summary)
{
    ++summary.analyzed_rows;
    if (inferred_contacts == 2)
        ++summary.selected_two_contact_rows;
    if (inferred_contacts == 4)
        ++summary.selected_four_contact_rows;
    if (inferred_contacts != actual_contacts)
        ++summary.contact_count_mismatch_rows;
    if (has_actual_contact_mask)
    {
        ++summary.actual_contact_mask_rows;
        if (!contact_mask_match)
            ++summary.contact_mask_mismatch_rows;
    }
    if (!wrench_solution.wrench_satisfied)
        ++summary.wrench_unsatisfied_rows;
    if (!wrench_solution.task_satisfied)
        ++summary.task_unsatisfied_rows;
    if (reduced_task_requested)
    {
        ++summary.reduced_task_rows;
        if (!wrench_solution.task_satisfied)
            ++summary.reduced_task_unsatisfied_rows;
    }
    if (!wrench_solution.constraint_report.feasible)
        ++summary.constraint_failures;

    summary.max_residual = std::max(
        summary.max_residual, wrench_solution.residual_norm);
    summary.max_constraint_violation = std::max(
        summary.max_constraint_violation,
        wrench_solution.constraint_report.max_violation);
    summary.max_task_residual = std::max(
        summary.max_task_residual, wrench_solution.task_residual_norm);
    summary.max_abs_torque = std::max(
        summary.max_abs_torque, torque_solution.max_abs_torque);

    for (const go2::Vec3 &force : wrench_solution.forces)
    {
        summary.max_force_component = std::max(
            summary.max_force_component,
            std::max(
                std::abs(force.x),
                std::max(std::abs(force.y), std::abs(force.z))));
    }
}

std::array<double, 6> ComputeContactWrench(
    const go2_control::ProjectedContactWrenchRequest &request,
    const go2_control::ProjectedContactWrenchSolution &solution)
{
    std::array<double, 6> wrench{};
    for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
    {
        if (!request.wrench.contact[leg])
            continue;
        const go2::Vec3 &force = solution.forces[leg];
        const go2::Vec3 &position = request.wrench.contact_positions_body[leg];
        wrench[0] += force.x;
        wrench[1] += force.y;
        wrench[2] += force.z;
        wrench[3] += position.y * force.z - position.z * force.y;
        wrench[4] += position.z * force.x - position.x * force.z;
        wrench[5] += position.x * force.y - position.y * force.x;
    }
    return wrench;
}
void WriteRow(
    std::ofstream &output,
    std::size_t row_number,
    double cmd_time_s,
    double phase,
    const std::array<double, 6> &desired_wrench,
    const std::array<double, 6> &achieved_wrench,
    const std::array<double, 6> &wrench_residual,
    int inferred_contacts,
    bool reduced_task,
    int actual_contacts,
    int contact_mask_match,
    bool has_measured_wrench,
    double measured_fz,
    double measured_mx,
    double measured_my,
    double measured_wrench_residual,
    const ShadowRow &shadow,
    const go2_control::DynamicAccelerationTargetResult &dynamic_accel_target,
    const go2_control::ProjectedContactWrenchSolution &wrench_solution,
    const go2_control::ContactTorqueMapSolution &torque_solution)
{
    output << row_number << ","
           << std::setprecision(9) << cmd_time_s << ","
           << phase << ","
           << desired_wrench[0] << ","
           << desired_wrench[1] << ","
           << desired_wrench[2] << ","
           << desired_wrench[3] << ","
           << desired_wrench[4] << ","
           << desired_wrench[5] << ","
           << achieved_wrench[0] << ","
           << achieved_wrench[1] << ","
           << achieved_wrench[2] << ","
           << achieved_wrench[3] << ","
           << achieved_wrench[4] << ","
           << achieved_wrench[5] << ","
           << wrench_residual[0] << ","
           << wrench_residual[1] << ","
           << wrench_residual[2] << ","
           << wrench_residual[3] << ","
           << wrench_residual[4] << ","
           << wrench_residual[5] << ","
           << wrench_solution.max_radial_friction_ratio << ","
           << wrench_solution.min_contact_normal_force << ","
           << inferred_contacts << ","
           << (reduced_task ? 1 : 0) << ","
           << actual_contacts << ","
           << (inferred_contacts == actual_contacts ? 1 : 0) << ","
           << contact_mask_match << ","
           << (has_measured_wrench ? measured_fz : -1.0) << ","
           << (has_measured_wrench ? measured_mx : -1.0) << ","
           << (has_measured_wrench ? measured_my : -1.0) << ","
           << (has_measured_wrench ? measured_wrench_residual : -1.0)
           << ","
           << wrench_solution.iterations << ","
           << (wrench_solution.converged ? 1 : 0) << ","
           << (wrench_solution.wrench_satisfied ? 1 : 0) << ","
           << wrench_solution.residual_norm << ","
           << (wrench_solution.task_satisfied ? 1 : 0) << ","
           << wrench_solution.task_residual_norm << ","
           << (wrench_solution.constraint_report.feasible ? 1 : 0) << ","
           << wrench_solution.constraint_report.max_violation << ","
           << torque_solution.max_abs_torque;

    for (const auto &leg_torques : torque_solution.torques)
    {
        for (double torque : leg_torques)
            output << "," << torque;
    }
    output << "," << (shadow.active ? 1 : 0)
           << "," << (shadow.policy_satisfied ? 1 : 0)
           << "," << (shadow.moment_task_active ? 1 : 0)
           << "," << (shadow.fallback_to_force_solution ? 1 : 0)
           << "," << shadow.max_force_excess
           << "," << shadow.max_moment_excess
           << "," << shadow.force_slack[0]
           << "," << shadow.force_slack[1]
           << "," << shadow.force_slack[2]
           << "," << shadow.moment_slack[0]
           << "," << shadow.moment_slack[1]
           << "," << shadow.moment_slack[2]
           << "," << (shadow.torque_rate_task_active ? 1 : 0)
           << "," << (shadow.torque_rate_satisfied ? 1 : 0)
           << "," << shadow.max_torque_rate_excess
           << "," << (dynamic_accel_target.valid ? 1 : 0)
           << "," << (dynamic_accel_target.reference_held_for_duplicate_time ? 1 : 0)
           << "," << (dynamic_accel_target.slew_limited ? 1 : 0)
           << "," << dynamic_accel_target.raw_correction_world[0]
           << "," << dynamic_accel_target.raw_correction_world[1]
           << "," << dynamic_accel_target.raw_correction_world[2]
           << "," << dynamic_accel_target.applied_correction_world[0]
           << "," << dynamic_accel_target.applied_correction_world[1]
           << "," << dynamic_accel_target.applied_correction_world[2]
           << "," << dynamic_accel_target.correction_slack_world[0]
           << "," << dynamic_accel_target.correction_slack_world[1]
           << "," << dynamic_accel_target.correction_slack_world[2];
    output << "\n";
}

void PrintUsage()
{
    std::cerr
        << "usage: analyze_contact_torque_replay"
        << " --csv INPUT --out OUTPUT"
        << " [--ground-truth-csv INPUT]"
        << " [--period S] [--duty F] [--mass-kg KG]"
        << " [--gravity MPS2] [--mu VALUE]"
        << " [--max-normal-force N] [--contact-source gait|actual]"
        << " [--wrench-source gravity|imu|state|dynamic-accel|dynamic-accel-bounded]"
        << " [--dynamic-accel-correction-limit-mps2 N]"
        << " [--dynamic-accel-correction-slew-mps3 N]"
        << " [--reduced-contact-task] [--force-only-task]"
        << " [--contact-conditioned-slack]"
        << " [--force-tolerance-n N] [--moment-slack-nm N]"
        << " [--torque-rate-shadow] [--torque-rate-limit-nm-s N]"
        << " [--torque-rate-weight W]\n";
}

} // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!ParseOptions(argc, argv, options))
    {
        PrintUsage();
        return 2;
    }

    std::vector<GroundTruthRow> ground_truth_rows;
    if ((options.wrench_source == "dynamic-accel" ||
         options.wrench_source == "dynamic-accel-bounded") &&
        !LoadGroundTruth(options.ground_truth_csv, ground_truth_rows))
    {
        std::cerr << "cannot load dynamic-accel ground-truth CSV: "
                  << options.ground_truth_csv << "\n";
        return 2;
    }
    std::ifstream input(options.csv_path);
    if (!input)
    {
        std::cerr << "cannot open input CSV: " << options.csv_path << "\n";
        return 2;
    }
    std::ofstream output(options.output_path);
    if (!output)
    {
        std::cerr << "cannot open output CSV: " << options.output_path << "\n";
        return 2;
    }

    std::string line;
    if (!std::getline(input, line))
    {
        std::cerr << "input CSV has no header.\n";
        return 2;
    }
    const std::vector<std::string> header = SplitCsvLine(line);
    Columns columns;
    if (!BuildColumns(header, columns))
    {
        std::cerr << "input CSV is missing required state columns.\n";
        return 2;
    }
    if (options.contact_source == "actual" &&
        !columns.has_actual_contact_mask)
    {
        std::cerr << "actual contact source requires per-leg contact columns.\n";
        return 2;
    }

    if ((options.wrench_source == "state" ||
         options.wrench_source == "dynamic-accel" ||
         options.wrench_source == "dynamic-accel-bounded") &&
        columns.wbc_shadow_desired_force_x_n == kMissing)
    {
        std::cerr << "state/dynamic wrench source requires "
                  << "wbc_shadow_desired_force_x_n.\n";
        return 2;
    }

    output << "row_number,cmd_time_s,phase,desired_force_x_n,"
           << "desired_force_y_n,desired_force_z_n,desired_moment_x_nm,"
           << "desired_moment_y_nm,desired_moment_z_nm,"
           << "achieved_force_x_n,achieved_force_y_n,achieved_force_z_n,"
           << "achieved_moment_x_nm,achieved_moment_y_nm,achieved_moment_z_nm,"
           << "wrench_residual_force_x_n,wrench_residual_force_y_n,wrench_residual_force_z_n,"
           << "wrench_residual_moment_x_nm,wrench_residual_moment_y_nm,wrench_residual_moment_z_nm,"
           << "max_radial_friction_ratio,min_contact_normal_force,selected_contact_count,"
           << "reduced_task,"
           << "actual_contact_count,contact_count_match,contact_mask_match,"
           << "measured_fz_total,measured_mx,measured_my,"
           << "measured_wrench_residual,iterations,"
           << "converged,wrench_satisfied,residual_norm,"
           << "task_satisfied,task_residual_norm,"
           << "constraint_feasible,max_constraint_violation,max_abs_torque";
    for (const char *leg : kLegNames)
    {
        for (const char *joint : kJointNames)
            output << "," << leg << "_" << joint << "_tau_ff_candidate";
    }
    output << ",shadow_active,shadow_policy_satisfied,shadow_moment_task_active,"
           << "shadow_fallback_to_force_solution,shadow_max_force_excess_n,shadow_max_moment_excess_nm,"
           << "shadow_force_slack_x_n,shadow_force_slack_y_n,shadow_force_slack_z_n,"
           << "shadow_moment_slack_x_nm,shadow_moment_slack_y_nm,shadow_moment_slack_z_nm,"
           << "shadow_torque_rate_task_active,shadow_torque_rate_satisfied,"
           << "shadow_max_torque_rate_excess_nm"
           << ",dynamic_accel_target_active,dynamic_accel_reference_held_for_duplicate_time,"
           << "dynamic_accel_slew_limited,"
           << "dynamic_accel_correction_raw_x_mps2,dynamic_accel_correction_raw_y_mps2,dynamic_accel_correction_raw_z_mps2,"
           << "dynamic_accel_correction_applied_x_mps2,dynamic_accel_correction_applied_y_mps2,dynamic_accel_correction_applied_z_mps2,"
           << "dynamic_accel_correction_slack_x_mps2,dynamic_accel_correction_slack_y_mps2,dynamic_accel_correction_slack_z_mps2";
    output << "\n";

    Summary summary;
    std::size_t row_number = 0;
    bool has_previous_shadow_torque = false;
    double previous_shadow_time_s = 0.0;
    std::array<double, go2::kJointCount> previous_shadow_torque{};
    bool has_previous_dynamic_acceleration = false;
    double previous_dynamic_time_s = 0.0;
    std::array<double, 3> previous_dynamic_correction{};
    while (std::getline(input, line))
    {
        ++row_number;
        ++summary.total_rows;
        const std::vector<std::string> fields = SplitCsvLine(line);

        double plan_valid = 0.0;
        double phase = 0.0;
        double actual_contact_count = 0.0;
        double cmd_time_s = 0.0;
        double desired_force_x_n = 0.0;
        if (!ReadField(fields, columns.plan_valid, plan_valid) ||
            !ReadField(fields, columns.phase, phase) ||
            !ReadField(
                fields, columns.actual_contact_count, actual_contact_count) ||
            !ReadField(fields, columns.cmd_time_s, cmd_time_s) ||
            plan_valid < 0.5 ||
            phase < 0.0 ||
            phase >= 1.0)
        {
            ++summary.skipped_rows;
            continue;
        }

        std::array<int, go2::kLegCount> actual_contact_flags{};
        std::array<double, go2::kLegCount> measured_foot_forces{};
        std::array<double, 3> imu_acceleration{};
        bool row_valid = true;
        if ((options.wrench_source == "state" ||
             options.wrench_source == "dynamic-accel" ||
             options.wrench_source == "dynamic-accel-bounded") &&
            !ReadField(
                fields, columns.wbc_shadow_desired_force_x_n,
                desired_force_x_n))
            row_valid = false;
        if (columns.has_actual_contact_mask)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                double actual_contact = 0.0;
                if (!ReadField(
                        fields, columns.actual_contact[leg],
                        actual_contact))
                {
                    row_valid = false;
                }
                actual_contact_flags[leg] =
                    actual_contact >= 0.5 ? 1 : 0;
            }
        }
        if (columns.has_foot_force)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (!ReadField(
                        fields, columns.foot_force[leg],
                        measured_foot_forces[leg]))
                {
                    row_valid = false;
                }
            }
        }
        if (columns.has_imu_acceleration)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (!ReadField(
                        fields, columns.imu_acceleration[axis],
                        imu_acceleration[axis]))
                {
                    row_valid = false;
                }
            }
        }

        go2_control::ProjectedContactWrenchRequest request;
        request.wrench.contact.fill(false);
        request.force_constraints.friction_coefficient =
            options.friction_coefficient;
        request.force_constraints.max_normal_force =
            options.max_normal_force;
        request.residual_tolerance = 1e-5;

        std::array<std::array<double, 3>, go2::kLegCount> angles{};
        for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
        {
            for (std::size_t joint = 0;
                 joint < go2::kJointsPerLeg;
                 ++joint)
            {
                if (!ReadField(
                        fields, columns.q_state[leg][joint],
                        angles[leg][joint]))
                {
                    row_valid = false;
                }
            }
            const bool gait_contact =
                LegPhase(phase, leg) < options.duty_factor;
            request.wrench.contact[leg] =
                options.contact_source == "actual"
                    ? actual_contact_flags[leg] != 0
                    : gait_contact;
            request.wrench.contact_positions_body[leg] =
                go2::FootPosition(
                    static_cast<go2::Leg>(leg),
                    angles[leg][0],
                    angles[leg][1],
                    angles[leg][2]);
        }
        if (!row_valid)
        {
            ++summary.skipped_rows;
            continue;
        }

        int active_contacts = 0;
        for (bool active : request.wrench.contact)
            active_contacts += active ? 1 : 0;
        const bool shadow_moment_task_active =
            options.contact_conditioned_slack &&
            !options.force_only_task &&
            active_contacts == static_cast<int>(go2::kLegCount);
        const bool task_is_reduced =
            options.force_only_task ||
            (options.reduced_contact_task &&
             active_contacts < go2::kLegCount) ||
            (options.contact_conditioned_slack &&
             !shadow_moment_task_active);
        if (task_is_reduced)
            request.wrench.task_weights = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};

        go2_control::DynamicAccelerationTargetResult dynamic_accel_target;
        std::array<double, 6> desired_wrench = {
            0.0,
            0.0,
            options.mass_kg * options.gravity_mps2,
            0.0,
            0.0,
            0.0};
        if (options.wrench_source == "imu")
        {
            desired_force_x_n = options.mass_kg * imu_acceleration[0];
            desired_wrench[0] = options.mass_kg * imu_acceleration[0];
            desired_wrench[1] = options.mass_kg * imu_acceleration[1];
            desired_wrench[2] = options.mass_kg * imu_acceleration[2];
        }
        else if (options.wrench_source == "state")
        {
            desired_wrench[0] = desired_force_x_n;
        }
        else if (options.wrench_source == "dynamic-accel")
        {
            const GroundTruthRow *ground_truth = MatchGroundTruth(
                ground_truth_rows,
                cmd_time_s,
                kGroundTruthMatchToleranceS);
            if (ground_truth == nullptr)
            {
                ++summary.ground_truth_match_failures;
                ++summary.skipped_rows;
                continue;
            }
            desired_wrench = BuildDynamicAccelerationWrench(
                *ground_truth,
                desired_force_x_n,
                options.mass_kg);
        }
        else if (options.wrench_source == "dynamic-accel-bounded")
        {
            const GroundTruthRow *ground_truth = MatchGroundTruth(
                ground_truth_rows,
                cmd_time_s,
                kGroundTruthMatchToleranceS);
            if (ground_truth == nullptr)
            {
                ++summary.ground_truth_match_failures;
                ++summary.skipped_rows;
                continue;
            }
            go2_control::DynamicAccelerationTargetInput target_input;
            target_input.quaternion = ground_truth->quaternion;
            target_input.measured_acceleration_world =
                ground_truth->qacc_world;
            target_input.mass_qacc_qcoord = ground_truth->mass_qacc;
            target_input.base_mass_matrix_qcoord =
                ground_truth->mass_matrix;
            target_input.smooth_qcoord = ground_truth->smooth;
            target_input.desired_force_x_n = desired_force_x_n;
            target_input.mass_kg = options.mass_kg;
            const double dynamic_dt_s =
                cmd_time_s - previous_dynamic_time_s;
            const std::array<double, 3> *previous_correction =
                has_previous_dynamic_acceleration
                    ? &previous_dynamic_correction
                    : nullptr;
            go2_control::DynamicAccelerationTargetOptions target_options;
            target_options.correction_limit_mps2 =
                options.dynamic_accel_correction_limit_mps2;
            target_options.correction_slew_limit_mps3 =
                options.dynamic_accel_correction_slew_limit_mps3;
            dynamic_accel_target =
                go2_control::BuildDynamicAccelerationTarget(
                    target_input,
                    target_options,
                    previous_correction,
                    dynamic_dt_s);
            if (!dynamic_accel_target.valid)
            {
                ++summary.dynamic_target_failures;
                ++summary.skipped_rows;
                continue;
            }
            desired_wrench = dynamic_accel_target.desired_wrench_body;
            previous_dynamic_correction =
                dynamic_accel_target.applied_correction_world;
            previous_dynamic_time_s = cmd_time_s;
            has_previous_dynamic_acceleration = true;
        }
        request.wrench.desired_wrench = desired_wrench;
        double measured_fz = 0.0;
        double measured_mx = 0.0;
        double measured_my = 0.0;
        double measured_wrench_residual = -1.0;
        if (columns.has_foot_force)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                const auto &position =
                    request.wrench.contact_positions_body[leg];
                const double force_z = measured_foot_forces[leg];
                measured_fz += force_z;
                measured_mx += position.y * force_z;
                measured_my -= position.x * force_z;
            }
            measured_wrench_residual = std::sqrt(
                (measured_fz - request.wrench.desired_wrench[2]) *
                    (measured_fz - request.wrench.desired_wrench[2]) +
                measured_mx * measured_mx +
                measured_my * measured_my);
        }
        const double shadow_dt_s =
            cmd_time_s - previous_shadow_time_s;
        const bool shadow_torque_rate_task_active =
            options.torque_rate_shadow &&
            has_previous_shadow_torque &&
            std::isfinite(shadow_dt_s) && shadow_dt_s > 1e-12;
        ShadowRow shadow_row;
        go2_control::ProjectedContactWrenchSolution wrench_solution;
        if (options.contact_conditioned_slack)
        {
            go2_control::LexicographicContactWrenchRequest shadow_request;
            shadow_request.wrench = request.wrench;
            shadow_request.force_constraints = request.force_constraints;
            shadow_request.moment_task_active =
                shadow_moment_task_active;
            shadow_request.force_tolerance = options.force_tolerance_n;
            shadow_request.moment_slack_limit = options.moment_slack_nm;
            shadow_request.residual_tolerance =
                request.residual_tolerance;
            shadow_request.torque_rate_task_active =
                shadow_torque_rate_task_active;
            shadow_request.joint_angles = angles;
            shadow_request.previous_torque = previous_shadow_torque;
            shadow_request.dt_s = shadow_dt_s;
            shadow_request.torque_rate_limit_nm_s =
                options.torque_rate_limit_nm_s;
            shadow_request.torque_rate_weight =
                options.torque_rate_weight;
            go2_control::ContactWrenchLexicographicSlackAllocator
                shadow_allocator;
            go2_control::LexicographicContactWrenchSolution
                shadow_solution;
            if (!shadow_allocator.Solve(
                    shadow_request, shadow_solution))
            {
                ++summary.solver_failures;
                continue;
            }
            wrench_solution.forces = shadow_solution.forces;
            wrench_solution.active_contacts =
                shadow_solution.active_contacts;
            wrench_solution.iterations = shadow_solution.iterations;
            wrench_solution.converged = shadow_solution.converged;
            wrench_solution.wrench_satisfied =
                shadow_solution.wrench_satisfied;
            wrench_solution.residual_norm =
                shadow_solution.residual_norm;
            wrench_solution.task_satisfied =
                shadow_solution.policy_satisfied;
            wrench_solution.task_residual_norm = std::max(
                std::max(
                    shadow_solution.max_force_excess,
                    shadow_solution.max_moment_excess),
                shadow_solution.max_torque_rate_excess);
            wrench_solution.constraint_report =
                shadow_solution.constraint_report;
            wrench_solution.max_axis_friction_ratio =
                shadow_solution.max_axis_friction_ratio;
            wrench_solution.max_radial_friction_ratio =
                shadow_solution.max_radial_friction_ratio;
            wrench_solution.min_contact_normal_force =
                shadow_solution.min_contact_normal_force;
            shadow_row.active = true;
            shadow_row.policy_satisfied =
                shadow_solution.policy_satisfied;
            shadow_row.moment_task_active =
                shadow_solution.moment_task_active;
            shadow_row.fallback_to_force_solution =
                shadow_solution.fallback_to_force_solution;
            shadow_row.max_force_excess =
                shadow_solution.max_force_excess;
            shadow_row.max_moment_excess =
                shadow_solution.max_moment_excess;
            shadow_row.force_slack = shadow_solution.force_slack;
            shadow_row.moment_slack = shadow_solution.moment_slack;
            shadow_row.torque_rate_task_active =
                shadow_solution.torque_rate_task_active;
            shadow_row.torque_rate_satisfied =
                shadow_solution.torque_rate_satisfied;
            shadow_row.max_torque_rate_excess =
                shadow_solution.max_torque_rate_excess;
        }
        else
        {
            go2_control::ContactWrenchProjectedAllocator allocator;
            if (!allocator.Solve(request, wrench_solution))
            {
                ++summary.solver_failures;
                continue;
            }
        }

        go2_control::ContactTorqueMapRequest torque_request;
        torque_request.joint_angles = angles;
        torque_request.contact_forces = wrench_solution.forces;
        torque_request.contact = request.wrench.contact;
        go2_control::ContactTorqueMapSolution torque_solution;
        if (!go2_control::MapContactForcesToJointTorques(
                torque_request, torque_solution))
        {
            ++summary.mapping_failures;
            continue;
        }

        if (options.torque_rate_shadow)
        {
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                for (std::size_t joint = 0;
                     joint < go2::kJointsPerLeg;
                     ++joint)
                {
                    previous_shadow_torque[
                        3 * leg + joint] =
                        torque_solution.torques[leg][joint];
                }
            }
            previous_shadow_time_s = cmd_time_s;
            has_previous_shadow_torque = true;
        }
        const int inferred_contacts = wrench_solution.active_contacts;
        const std::array<double, 6> achieved_wrench =
            ComputeContactWrench(request, wrench_solution);
        std::array<double, 6> wrench_residual{};
        for (std::size_t index = 0; index < wrench_residual.size(); ++index)
        {
            wrench_residual[index] = achieved_wrench[index] - desired_wrench[index];
        }
        const int actual_contacts = columns.has_actual_contact_mask
            ? [&]() {
                  int count = 0;
                  for (int contact : actual_contact_flags)
                      count += contact;
                  return count;
              }()
            : static_cast<int>(std::llround(actual_contact_count));
        bool contact_mask_match = false;
        int contact_mask_match_field = -1;
        if (columns.has_actual_contact_mask)
        {
            contact_mask_match = true;
            for (std::size_t leg = 0; leg < go2::kLegCount; ++leg)
            {
                if (static_cast<int>(request.wrench.contact[leg]) !=
                    actual_contact_flags[leg])
                {
                    contact_mask_match = false;
                }
            }
            contact_mask_match_field = contact_mask_match ? 1 : 0;
        }
        if (shadow_row.active)
        {
            if (!shadow_row.policy_satisfied)
                ++summary.shadow_policy_unsatisfied_rows;
            if (shadow_row.fallback_to_force_solution)
                ++summary.shadow_fallback_rows;
            summary.max_shadow_force_excess = std::max(
                summary.max_shadow_force_excess,
                shadow_row.max_force_excess);
            summary.max_shadow_moment_excess = std::max(
                summary.max_shadow_moment_excess,
                shadow_row.max_moment_excess);
            if (shadow_row.torque_rate_task_active &&
                !shadow_row.torque_rate_satisfied)
                ++summary.shadow_torque_rate_unsatisfied_rows;
            summary.max_shadow_torque_rate_excess = std::max(
                summary.max_shadow_torque_rate_excess,
                shadow_row.max_torque_rate_excess);
        }
        UpdateSummary(
            wrench_solution,
            torque_solution,
            inferred_contacts,
            actual_contacts,
            columns.has_actual_contact_mask,
            contact_mask_match,
            task_is_reduced,
            summary);
        if (columns.has_foot_force)
        {
            ++summary.measured_wrench_rows;
            summary.max_measured_fz_error = std::max(
                summary.max_measured_fz_error,
                std::abs(
                    measured_fz -
                    request.wrench.desired_wrench[2]));
            summary.max_measured_wrench_residual = std::max(
                summary.max_measured_wrench_residual,
                measured_wrench_residual);
        }
        WriteRow(
            output,
            row_number,
            cmd_time_s,
            phase,
            desired_wrench,
            achieved_wrench,
            wrench_residual,
            inferred_contacts,
            task_is_reduced,
            actual_contacts,
            contact_mask_match_field,
            columns.has_foot_force,
            measured_fz,
            measured_mx,
            measured_my,
            measured_wrench_residual,
            shadow_row,
            dynamic_accel_target,
            wrench_solution,
            torque_solution);
    }

    std::cout << "contact torque replay completed\n"
              << "input=" << options.csv_path << "\n"
              << "output=" << options.output_path << "\n"
              << "contact_source=" << options.contact_source << "\n"
              << "wrench_source=" << options.wrench_source << "\n"
              << "total_rows=" << summary.total_rows << "\n"
              << "analyzed_rows=" << summary.analyzed_rows << "\n"
              << "skipped_rows=" << summary.skipped_rows << "\n"
              << "solver_failures=" << summary.solver_failures << "\n"
              << "mapping_failures=" << summary.mapping_failures << "\n"
              << "wrench_unsatisfied_rows="
              << summary.wrench_unsatisfied_rows << "\n"
              << "task_unsatisfied_rows="
              << summary.task_unsatisfied_rows << "\n"
              << "reduced_task_rows="
              << summary.reduced_task_rows << "\n"
              << "reduced_task_unsatisfied_rows="
              << summary.reduced_task_unsatisfied_rows << "\n"
              << "constraint_failures=" << summary.constraint_failures << "\n"
              << "contact_count_mismatch_rows="
              << summary.contact_count_mismatch_rows << "\n"
              << "actual_contact_mask_rows="
              << summary.actual_contact_mask_rows << "\n"
              << "contact_mask_mismatch_rows="
              << summary.contact_mask_mismatch_rows << "\n"
              << "selected_two_contact_rows="
              << summary.selected_two_contact_rows << "\n"
              << "selected_four_contact_rows="
              << summary.selected_four_contact_rows << "\n"
              << "max_residual=" << summary.max_residual << "\n"
              << "max_constraint_violation="
              << summary.max_constraint_violation << "\n"
              << "max_force_component=" << summary.max_force_component << "\n"
              << "max_abs_torque=" << summary.max_abs_torque << "\n"
              << "max_task_residual=" << summary.max_task_residual << "\n"
              << "measured_wrench_rows="
              << summary.measured_wrench_rows << "\n"
              << "max_measured_fz_error="
              << summary.max_measured_fz_error << "\n"
              << "max_measured_wrench_residual="
              << summary.max_measured_wrench_residual << "\n";
    std::cout << "ground_truth_match_failures="
              << summary.ground_truth_match_failures << "\n";
    std::cout << "dynamic_target_failures="
              << summary.dynamic_target_failures << "\n";
    std::cout << "shadow_policy_unsatisfied_rows="
              << summary.shadow_policy_unsatisfied_rows << "\n"
              << "shadow_fallback_rows="
              << summary.shadow_fallback_rows << "\n"
              << "max_shadow_force_excess="
              << summary.max_shadow_force_excess << "\n"
              << "max_shadow_moment_excess="
              << summary.max_shadow_moment_excess << "\n"
              << "shadow_torque_rate_unsatisfied_rows="
              << summary.shadow_torque_rate_unsatisfied_rows << "\n"
              << "max_shadow_torque_rate_excess="
              << summary.max_shadow_torque_rate_excess << "\n";
    return summary.analyzed_rows > 0 &&
                   summary.ground_truth_match_failures == 0 &&
                   summary.dynamic_target_failures == 0
               ? 0
               : 1;
}
