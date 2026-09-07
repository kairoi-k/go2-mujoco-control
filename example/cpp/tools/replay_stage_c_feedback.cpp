// Offline Stage-C feedback sample replay.
// This calls the controller-side model evaluator and feedback_step only; it
// never calls mj_step and cannot certify contact evolution or final actuators.
#include "stage_c/feedback_step.h"

#include <cerrno>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {
using go2_control::RigidBodyState;
using go2_terrain::stage_c::TimeNs;
constexpr int kLegs = 4;
constexpr int kJoints = 12;

struct CsvTable {
    std::vector<std::string> header;
    std::map<std::string, std::size_t> index;
    std::vector<std::vector<std::string>> rows;
};

std::string Trim(std::string value) {
    while (!value.empty() && (value.back() == '\r' || value.back() == ' ' || value.back() == '\t')) value.pop_back();
    std::size_t first = 0;
    while (first < value.size() && (value[first] == ' ' || value[first] == '\t')) ++first;
    return value.substr(first);
}

std::vector<std::string> SplitCsv(const std::string &line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"'); ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            fields.push_back(Trim(field)); field.clear();
        } else {
            field.push_back(c);
        }
    }
    if (quoted) return {};
    fields.push_back(Trim(field));
    return fields;
}

bool LoadCsv(const std::string &path, CsvTable &table, std::string &error) {
    std::ifstream stream(path);
    if (!stream) { error = "cannot open samples CSV: " + path; return false; }
    std::string line;
    if (!std::getline(stream, line)) { error = "empty samples CSV"; return false; }
    table.header = SplitCsv(line);
    if (table.header.empty()) { error = "invalid samples header"; return false; }
    for (std::size_t i = 0; i < table.header.size(); ++i) {
        if (table.header[i].empty() || !table.index.emplace(table.header[i], i).second) {
            error = "duplicate or empty samples header"; return false;
        }
    }
    std::size_t row_number = 1;
    while (std::getline(stream, line)) {
        ++row_number;
        if (line.empty()) continue;
        auto row = SplitCsv(line);
        if (row.size() != table.header.size()) {
            error = "malformed samples row " + std::to_string(row_number); return false;
        }
        table.rows.push_back(std::move(row));
    }
    if (table.rows.empty()) { error = "samples CSV has no rows"; return false; }
    return true;
}

bool Get(const CsvTable &table, const std::vector<std::string> &row,
         const std::string &name, std::string &value) {
    const auto found = table.index.find(name);
    if (found == table.index.end() || found->second >= row.size()) return false;
    value = row[found->second];
    return true;
}

bool Finite(const CsvTable &table, const std::vector<std::string> &row,
            const std::string &name, double &value) {
    std::string text;
    if (!Get(table, row, name, text) || text.empty()) return false;
    char *end = nullptr;
    errno = 0;
    value = std::strtod(text.c_str(), &end);
    return errno != ERANGE && end != text.c_str() && *end == '\0' && std::isfinite(value);
}

bool StrictInt(const CsvTable &table, const std::vector<std::string> &row,
               const std::string &name, int minimum, int maximum, int &value) {
    double number = 0.0;
    if (!Finite(table, row, name, number)) return false;
    const double rounded = std::round(number);
    if (std::abs(number - rounded) > 1.0e-9 || rounded < minimum || rounded > maximum) return false;
    value = static_cast<int>(rounded);
    return true;
}

bool ParseList(const std::string &text, std::size_t expected, std::vector<double> &values) {
    values.clear();
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        const std::string trimmed = Trim(token);
        char *end = nullptr;
        errno = 0;
        const double value = std::strtod(trimmed.c_str(), &end);
        if (errno == ERANGE || end == trimmed.c_str() || *end != '\0' || !std::isfinite(value)) return false;
        values.push_back(value);
    }
    return values.size() == expected;
}
bool ParseScalar(const std::string &text, double &value) {
    std::vector<double> values;
    if (!ParseList(text, 1, values)) return false;
    value = values[0];
    return true;
}

struct Options {
    std::string samples;
    std::string model;
    std::string output;
    std::vector<double> centroidal_target;
    std::vector<double> centroidal_weights;
    std::vector<double> foot_acceleration;
    std::vector<double> force_reference;
    bool use_scene_surface = false;
    bool have_force_reference = false;
    double surface_friction_mu = 0.8;
    double surface_min_normal_n = 0.0;
    double surface_max_normal_n = 180.0;
    double tau_limit_nm = 35.0;
    double joint_velocity_limit_radps = 30.0;
};

void Usage() {
    std::cerr << "usage: replay_stage_c_feedback --samples FILE --model FILE --out NEW.json "
                 "--centroidal-target a,b,c,d,e,f --foot-acceleration 12 values "
                 "[--centroidal-weights 6 values] [--force-reference 12 values] "
                 "[--use-scene-surface --surface-friction-mu MU --surface-min-normal-n N "
                 "--surface-max-normal-n N] [--tau-limit-nm N] [--joint-velocity-limit-radps N]\n";
}

bool ParseOptions(int argc, char **argv, Options &options) {
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto next = [&](std::string &value) {
            if (i + 1 >= argc) return false;
            value = argv[++i]; return true;
        };
        std::string value;
        if (argument == "--samples") { if (!next(options.samples)) return false; }
        else if (argument == "--model") { if (!next(options.model)) return false; }
        else if (argument == "--out") { if (!next(options.output)) return false; }
        else if (argument == "--centroidal-target") {
            if (!next(value) || !ParseList(value, 6, options.centroidal_target)) return false;
        } else if (argument == "--centroidal-weights") {
            if (!next(value) || !ParseList(value, 6, options.centroidal_weights)) return false;
        } else if (argument == "--foot-acceleration") {
            if (!next(value) || !ParseList(value, 12, options.foot_acceleration)) return false;
        } else if (argument == "--force-reference") {
            if (!next(value) || !ParseList(value, 12, options.force_reference)) return false;
            options.have_force_reference = true;
        } else if (argument == "--use-scene-surface") options.use_scene_surface = true;
        else if (argument == "--surface-friction-mu") {
            if (!next(value) || !ParseScalar(value, options.surface_friction_mu)) return false;
        } else if (argument == "--surface-min-normal-n") {
            if (!next(value) || !ParseScalar(value, options.surface_min_normal_n)) return false;
        } else if (argument == "--surface-max-normal-n") {
            if (!next(value) || !ParseScalar(value, options.surface_max_normal_n)) return false;
        } else if (argument == "--tau-limit-nm") {
            if (!next(value) || !ParseScalar(value, options.tau_limit_nm)) return false;
        } else if (argument == "--joint-velocity-limit-radps") {
            if (!next(value) || !ParseScalar(value, options.joint_velocity_limit_radps)) return false;
        } else { return false; }
    }
    if (options.samples.empty() || options.model.empty() || options.output.empty() ||
        options.centroidal_target.size() != 6 || options.foot_acceleration.size() != 12) return false;
    if (options.centroidal_weights.empty()) options.centroidal_weights.assign(6, 1.0);
    if (options.centroidal_weights.size() != 6 || !std::isfinite(options.tau_limit_nm) ||
        options.tau_limit_nm <= 0.0 || !std::isfinite(options.joint_velocity_limit_radps) ||
        options.joint_velocity_limit_radps <= 0.0) return false;
    if (options.use_scene_surface &&
        (!std::isfinite(options.surface_friction_mu) || options.surface_friction_mu < 0.0 ||
         !std::isfinite(options.surface_min_normal_n) || options.surface_min_normal_n < 0.0 ||
         !std::isfinite(options.surface_max_normal_n) || options.surface_max_normal_n < options.surface_min_normal_n)) return false;
    return true;
}

std::string JsonEscape(const std::string &value) {
    std::string escaped;
    for (const char c : value) {
        if (c == '"') escaped += "\\\"";
        else if (c == '\\') escaped += "\\\\";
        else if (c == '\n') escaped += "\\n";
        else if (c == '\r') escaped += "\\r";
        else if (c == '\t') escaped += "\\t";
        else escaped.push_back(c);
    }
    return escaped;
}

void JsonString(std::ostream &out, const std::string &value) {
    out << '"' << JsonEscape(value) << '"';
}
void JsonOptionalString(std::ostream &out, const CsvTable &table,
                        const std::vector<std::string> &row, const std::string &name) {
    std::string value;
    if (Get(table, row, name, value)) JsonString(out, value);
    else out << "null";
}

void JsonDouble(std::ostream &out, double value) {
    if (!std::isfinite(value)) out << "null";
    else out << std::setprecision(17) << value;
}

void JsonBool(std::ostream &out, bool value) { out << (value ? "true" : "false"); }

void JsonVector(std::ostream &out, const std::vector<double> &values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) { if (i) out << ','; JsonDouble(out, values[i]); }
    out << ']';
}

bool BuildState(const CsvTable &table, const std::vector<std::string> &row, RigidBodyState &state) {
    double value = 0.0;
    if (!Finite(table, row, "base_x_m", state.position_world.x()) ||
        !Finite(table, row, "base_y_m", state.position_world.y()) ||
        !Finite(table, row, "base_z_m", state.position_world.z()) ||
        !Finite(table, row, "base_quat_w", value)) return false;
    const double qw = value;
    double qx = 0.0, qy = 0.0, qz = 0.0;
    if (!Finite(table, row, "base_quat_x", qx) || !Finite(table, row, "base_quat_y", qy) ||
        !Finite(table, row, "base_quat_z", qz) || !Finite(table, row, "base_vx_mps", state.linear_vel_world.x()) ||
        !Finite(table, row, "base_vy_mps", state.linear_vel_world.y()) || !Finite(table, row, "base_vz_mps", state.linear_vel_world.z()) ||
        !Finite(table, row, "imu_gyro_body_x_radps", state.angular_vel_body.x()) ||
        !Finite(table, row, "imu_gyro_body_y_radps", state.angular_vel_body.y()) ||
        !Finite(table, row, "imu_gyro_body_z_radps", state.angular_vel_body.z())) return false;
    state.quat_world_from_body = Eigen::Quaterniond(qw, qx, qy, qz);
    const double quaternion_norm = state.quat_world_from_body.norm();
    if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1.0e-6) return false;
    for (int joint = 0; joint < kJoints; ++joint) {
        std::string q_name = "q_" + std::to_string(joint);
        std::string dq_name = "dq_" + std::to_string(joint);
        if (!Finite(table, row, q_name, state.q[joint]) || !Finite(table, row, dq_name, state.dq[joint])) return false;
    }
    return true;
}

bool BuildContactSurface(const CsvTable &table, const std::vector<std::string> &row,
                         int leg, TimeNs start, TimeNs end, std::uint64_t map_epoch,
                         double friction, double min_normal, double max_normal,
                         go2_terrain::stage_c::TimedPoint &point,
                         go2_terrain::stage_c::ContactSurface &surface) {
    const char *names[kLegs] = {"FR", "FL", "RR", "RL"};
    const std::string prefix = names[leg] + std::string("_gt_foot_");
    double x = 0.0, y = 0.0;
    if (!Finite(table, row, prefix + "x_m", x) || !Finite(table, row, prefix + "y_m", y)) return false;
    double z = 0.0, nx = 0.0, ny = 0.0, nz = 0.0;
    if (!Finite(table, row, "surface_plane_z_m", z) || !Finite(table, row, "surface_normal_x", nx) ||
        !Finite(table, row, "surface_normal_y", ny) || !Finite(table, row, "surface_normal_z", nz) ||
        std::abs(nx) > 1.0e-12 || std::abs(ny) > 1.0e-12 || std::abs(nz - 1.0) > 1.0e-12) return false;
    std::string source;
    if (!Get(table, row, "surface_provenance", source) || source.rfind("scene:", 0) != 0) return false;
    point.value = {x, y, z}; point.frame = go2_terrain::stage_c::Frame::kWorld;
    point.source_time = start; point.valid = true;
    point.role = go2_terrain::stage_c::PointRole::kSurfaceContactPoint;
    surface.basis_world = Eigen::Matrix3d::Identity();
    surface.frame = go2_terrain::stage_c::Frame::kWorld;
    surface.coverage = go2_terrain::stage_c::MapCoverageState::kKnown;
    surface.map_epoch = map_epoch; surface.valid_until = end;
    surface.friction_mu = friction; surface.min_normal_n = min_normal; surface.max_normal_n = max_normal;
    return map_epoch != 0;
}

void EmitNullableArray(std::ostream &out, const std::array<Eigen::Vector3d, 4> &values,
                       const std::array<bool, 4> &valid) {
    out << '[';
    for (int leg = 0; leg < kLegs; ++leg) {
        if (leg) out << ',';
        if (!valid[leg]) { out << "null"; continue; }
        out << '['; for (int axis = 0; axis < 3; ++axis) { if (axis) out << ','; JsonDouble(out, values[leg][axis]); } out << ']';
    }
    out << ']';
}

void EmitGapArray(std::ostream &out, const std::array<double, 4> &values,
                  const std::array<bool, 4> &valid) {
    out << '[';
    for (int leg = 0; leg < kLegs; ++leg) { if (leg) out << ','; if (!valid[leg]) out << "null"; else JsonDouble(out, values[leg]); }
    out << ']';
}

}  // namespace

int main(int argc, char **argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) { Usage(); return 2; }
    if (std::filesystem::exists(options.output)) {
        std::cerr << "ERROR: refusing to overwrite existing output: " << options.output << "\n"; return 2;
    }
    CsvTable samples;
    std::string error;
    if (!LoadCsv(options.samples, samples, error)) { std::cerr << "ERROR: " << error << "\n"; return 2; }
    go2_control::Go2RigidBody robot;
    if (!robot.Load(options.model)) { std::cerr << "ERROR: cannot load model: " << options.model << "\n"; return 2; }
    std::ofstream output(options.output, std::ios::out | std::ios::trunc);
    if (!output) { std::cerr << "ERROR: cannot create output: " << options.output << "\n"; return 2; }
    output << "{\n  \"schema\":\"stage_c_feedback_replay_v1\",\n"
           << "  \"claim_boundary\":\"offline same-model sample evaluation only; no mj_step, contact evolution, final PD actuator, or B1 claim\",\n"
           << "  \"model\":"; JsonString(output, options.model);
    output << ",\n  \"reference_provenance\":\"explicit evaluator CLI; absent from raw packet\",\n"
           << "  \"centroidal_target\":"; JsonVector(output, options.centroidal_target);
    output << ",\n  \"centroidal_weights\":"; JsonVector(output, options.centroidal_weights);
    output << ",\n  \"foot_acceleration\":"; JsonVector(output, options.foot_acceleration);
    output << ",\n  \"force_reference_provided\":"; JsonBool(output, options.have_force_reference);
    output << ",\n  \"use_scene_surface\":"; JsonBool(output, options.use_scene_surface);
    output << ",\n  \"surface_friction_mu\":"; JsonDouble(output, options.surface_friction_mu);
    output << ",\n  \"surface_min_normal_n\":"; JsonDouble(output, options.surface_min_normal_n);
    output << ",\n  \"surface_max_normal_n\":"; JsonDouble(output, options.surface_max_normal_n);
    output << ",\n  \"tau_limit_nm\":"; JsonDouble(output, options.tau_limit_nm);
    output << ",\n  \"joint_velocity_limit_radps\":"; JsonDouble(output, options.joint_velocity_limit_radps);
    output << ",\n  \"rows\":[\n";
    std::size_t verified = 0, unknown = 0, failed = 0;
    for (std::size_t row_number = 0; row_number < samples.rows.size(); ++row_number) {
        const auto &row = samples.rows[row_number];
        if (row_number) output << ",\n";
        double time_s = 0.0, dt_s = 0.0;
        int stage = -1, active = -1, contact_mask = -1;
        const bool common = Finite(samples, row, "state_tick_s", time_s) && Finite(samples, row, "motion_dt_s", dt_s) &&
            StrictInt(samples, row, "motion_stage", 2, 3, stage) && StrictInt(samples, row, "velocity_command_active", 1, 1, active) &&
            StrictInt(samples, row, "wbc_measured_contact_mask", 0, 15, contact_mask);
        output << "    {\"sample_index\":" << row_number << ",\"state_tick_s\":"; JsonDouble(output, time_s);
        output << ",\"motion_dt_s\":"; JsonDouble(output, dt_s);
        output << ",\"motion_stage\":" << stage << ",\"velocity_command_active\":" << active;
        std::string source;
        output << ",\"terrain_map_source\":"; JsonOptionalString(output, samples, row, "terrain_map_source");
        output << ",\"contact_provenance\":"; JsonOptionalString(output, samples, row, "contact_provenance");
        output << ",\"wbc_measured_contact_mask\":" << contact_mask;
        output << ",\"source_data_row\":"; std::string source_row; Get(samples, row, "data_row_index", source_row) ? output << source_row : output << "null";
        output << ",\"source_ground_truth_row\":"; Get(samples, row, "ground_truth_row_index", source_row) ? output << source_row : output << "null";
        output << ",\"base_position_source\":"; JsonOptionalString(output, samples, row, "base_position_source");
        output << ",\"orientation_source\":"; JsonOptionalString(output, samples, row, "orientation_source");
        output << ",\"base_velocity_source\":"; JsonOptionalString(output, samples, row, "base_linear_velocity_source");
        output << ",\"angular_velocity_source\":"; JsonOptionalString(output, samples, row, "angular_velocity_source");
        output << ",\"sample_certificate\":";
        std::string status = "unknown_bad_row";
        go2_terrain::stage_c::ArticulatedFeedbackStep step;
        std::array<bool, 4> gap_valid{};
        std::array<Eigen::Vector3d, 4> material_velocity{};
        std::array<bool, 4> material_valid{};
        if (!common || !std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 0.002000001) {
            ++unknown;
            output << "{\"status\":\"unknown_bad_row\",\"model_sample_verified\":false}";
        } else {
            RigidBodyState state;
            const bool state_valid = BuildState(samples, row, state);
            go2_control::RigidBodyPlanningKinematics model_state;
            const bool model_valid = state_valid && robot.EvaluatePlanningKinematics(state, model_state);
            const bool has_contact = contact_mask != 0;
            if (!state_valid) status = "unknown_nonfinite_or_bad_state";
            else if (!model_valid) status = "unknown_model_state_rejected";
            else if (!options.use_scene_surface && has_contact) status = "unknown_surface_not_authorized";
            else if (has_contact && !options.have_force_reference) status = "unknown_force_reference_missing";
            if (status != "unknown_bad_row" && status.rfind("unknown_", 0) == 0) {
                ++unknown;
                output << "{\"status\":"; JsonString(output, status); output << ",\"model_sample_verified\":false}";
            } else {
                status = "inputs_ready";
                go2_terrain::stage_c::ArticulatedFeedbackReference reference;
                reference.start = TimeNs::FromSeconds(time_s);
                reference.end = TimeNs::FromSeconds(time_s + dt_s);
                for (int i = 0; i < 6; ++i) {
                    reference.centroidal_derivative[i] = options.centroidal_target[i];
                    reference.centroidal_weights[i] = options.centroidal_weights[i];
                }
                for (int i = 0; i < 4; ++i) {
                    reference.foot_acceleration[i] = Eigen::Vector3d(
                        options.foot_acceleration[3 * i], options.foot_acceleration[3 * i + 1], options.foot_acceleration[3 * i + 2]);
                    reference.foot_acceleration_valid[i] = true;
                    reference.nominal_force.contact[i] = (contact_mask & (1 << i)) != 0;
                    const int offset = 3 * i;
                    reference.nominal_force.force_world[i] = options.have_force_reference
                        ? go2::Vec3{options.force_reference[offset], options.force_reference[offset + 1], options.force_reference[offset + 2]}
                        : go2::Vec3{};
                }
                reference.nominal_force.start = reference.start;
                reference.nominal_force.end = reference.end;
                int map_epoch = 0;
                if (!StrictInt(samples, row, "terrain_map_epoch", 1, 2147483647, map_epoch)) {
                    status = "unknown_map_epoch";
                } else {
                    for (int leg = 0; leg < kLegs; ++leg) {
                        if (!reference.nominal_force.contact[leg]) continue;
                        if (!BuildContactSurface(samples, row, leg, reference.start, reference.end,
                                                 static_cast<std::uint64_t>(map_epoch), options.surface_friction_mu,
                                                 options.surface_min_normal_n, options.surface_max_normal_n,
                                                 reference.surface_plane_points[leg], reference.surfaces[leg])) {
                            status = "unknown_surface_fields"; break;
                        }
                    }
                }
                if (status.rfind("unknown_", 0) == 0) {
                    ++unknown;
                    output << "{\"status\":"; JsonString(output, status); output << ",\"model_sample_verified\":false}";
                } else {
                    step = go2_terrain::stage_c::SolveArticulatedFeedbackStep(
                        robot, state, reference, [&] { go2_control::IdWbcParams p; p.tau_limit_nm = options.tau_limit_nm; return p; }(),
                        options.joint_velocity_limit_radps);
                    status = step.model_sample_verified ? "verified_model_sample" : go2_terrain::stage_c::JointPlannerFailureName(step.failure);
                    if (step.model_sample_verified) ++verified; else ++failed;
                    for (int leg = 0; leg < kLegs; ++leg) {
                        gap_valid[leg] = reference.nominal_force.contact[leg] && std::isfinite(step.signed_normal_gap_m[leg]);
                        material_valid[leg] = reference.nominal_force.contact[leg] && step.surface_material_velocity_world[leg].allFinite();
                        material_velocity[leg] = step.surface_material_velocity_world[leg];
                    }
                    output << "{\"status\":"; JsonString(output, status);
                    output << ",\"model_sample_verified\":"; JsonBool(output, step.model_sample_verified);
                    output << ",\"contact_evolution_verified\":false,\"execution_ready\":false";
                    output << ",\"failure\":"; JsonString(output, go2_terrain::stage_c::JointPlannerFailureName(step.failure));
                    output << ",\"qp_ok\":"; JsonBool(output, step.solution.ok);
                    output << ",\"qp_converged\":"; JsonBool(output, step.solution.qp_converged);
                    output << ",\"eq_residual\":"; JsonDouble(output, step.solution.eq_residual);
                    output << ",\"max_tau_violation_nm\":"; JsonDouble(output, step.solution.max_tau_violation_nm);
                    output << ",\"certificate_checked\":"; JsonBool(output, step.certificate.dynamics.checked);
                    output << ",\"certificate_input_valid\":"; JsonBool(output, step.certificate.dynamics.input_valid);
                    output << ",\"certificate_valid\":"; JsonBool(output, step.certificate.dynamics.valid);
                    output << ",\"certificate_feasible\":"; JsonBool(output, step.certificate.sample_feasible);
                    output << ",\"certificate_failure_bitmask\":" << step.certificate.dynamics.failure_bitmask;
                    output << ",\"force_residual_n\":"; JsonDouble(output, step.certificate.dynamics.max_dynamics_force_residual_N);
                    output << ",\"moment_residual_nm\":"; JsonDouble(output, step.certificate.dynamics.max_dynamics_moment_residual_Nm);
                    output << ",\"joint_residual_nm\":"; JsonDouble(output, step.certificate.dynamics.max_joint_dynamics_residual_Nm);
                    output << ",\"gap_m\":"; EmitGapArray(output, step.signed_normal_gap_m, gap_valid);
                    output << ",\"material_velocity_mps\":"; EmitNullableArray(output, material_velocity, material_valid);
                    output << '}';
                }
            }
        }
        output << '}';
    }
    output << "\n  ],\n  \"summary\":{\"verified_model_samples\":" << verified
           << ",\"unknown_rows\":" << unknown << ",\"failed_model_samples\":" << failed
           << ",\"contact_evolution_verified\":false,\"execution_ready\":false}\n}\n";
    if (!output) { std::cerr << "ERROR: failed writing output\n"; return 2; }
    std::cout << "wrote " << options.output << " verified_model_samples=" << verified
              << " unknown_rows=" << unknown << " failed_model_samples=" << failed << "\n";
    return 0;
}