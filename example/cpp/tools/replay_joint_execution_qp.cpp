// Offline replay of a serialized inverse-dynamics WBC QP.
// This tool never reads a plant, writes a motor command, or runs simulation.
// Currently scoped to a two-contact 24-variable, 18-equality, 36-inequality
// captured secondary HQP; other layouts reject explicitly.
// Input is a strict whitespace fixture containing named matrices:
// H g Aineq bineq Aeq beq seed iterate w_posture contact J0..J3
// bias0..bias3 task0..task3.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include "../wbc/dense_qp_active_set.h"
namespace {
struct Field { int rows = 0; int cols = 0; Eigen::MatrixXd value; };
const std::set<std::string> kNames = {
    "H", "g", "Aineq", "bineq", "Aeq", "beq", "seed", "iterate",
    "w_posture", "contact", "J0", "J1", "J2", "J3",
    "bias0", "bias1", "bias2", "bias3", "task0", "task1", "task2", "task3"};
std::string Quote(const std::string &s) {
    std::ostringstream o;
    o << '"';
    for (const char c : s) {
        if (c == '\\' || c == '"') o << '\\' << c;
        else if (c == '\n') o << "\\n";
        else if (c == '\r') o << "\\r";
        else o << c;
    }
    o << '"';
    return o.str();
}
std::string Number(double x) {
    if (!std::isfinite(x)) return "null";
    std::ostringstream o;
    o << std::setprecision(17) << x;
    return o.str();
}
void PrintVec(const Eigen::VectorXd &v) {
    std::cout << '[';
    for (int i = 0; i < v.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << Number(v[i]);
    }
    std::cout << ']';
}
void PrintArray(const std::vector<double> &v) {
    std::cout << '[';
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << Number(v[i]);
    }
    std::cout << ']';
}
void Fail(const std::string &why) {
    std::cout << "{\"ok\":false,\"error\":" << Quote(why)
              << ",\"feasibility_conclusion\":\"unknown_not_infeasible\"}\n";
}
bool ReadFixture(const std::string &path, std::map<std::string, Field> &out,
                 std::string &error) {
    std::ifstream in(path);
    if (!in) { error = "cannot_open_input"; return false; }
    std::string name;
    while (in >> name) {
        if (!kNames.count(name)) { error = "unexpected_field_" + name; return false; }
        if (out.count(name)) { error = "duplicate_field_" + name; return false; }
        Field f;
        if (!(in >> f.rows >> f.cols)) { error = "missing_dimensions_" + name; return false; }
        if (f.rows < 0 || f.cols < 0 || f.rows > 10000 || f.cols > 10000) {
            error = "invalid_dimensions_" + name; return false;
        }
        const long long count = static_cast<long long>(f.rows) * f.cols;
        if (count > 2000000) { error = "field_too_large_" + name; return false; }
        f.value.resize(f.rows, f.cols);
        for (long long i = 0; i < count; ++i) {
            double x;
            if (!(in >> x)) { error = "missing_number_" + name; return false; }
            if (!std::isfinite(x)) { error = "nonfinite_number_" + name; return false; }
            f.value(static_cast<int>(i / f.cols), static_cast<int>(i % f.cols)) = x;
        }
        out.emplace(name, std::move(f));
    }
    if (!in.eof()) { error = "malformed_or_trailing_input"; return false; }
    if (out.size() != kNames.size()) { error = "missing_required_field"; return false; }
    for (const auto &n : kNames)
        if (!out.count(n)) { error = "missing_field_" + n; return false; }
    return true;
}
bool Shape(const std::map<std::string, Field> &f, const std::string &n,
           int rows, int cols, std::string &error) {
    const auto &x = f.at(n);
    if (x.rows != rows || x.cols != cols) {
        std::ostringstream o; o << "bad_shape_" << n << "_" << x.rows << "x" << x.cols;
        error = o.str(); return false;
    }
    if (!x.value.allFinite()) { error = "nonfinite_matrix_" + n; return false; }
    return true;
}
double EqInf(const Eigen::MatrixXd &A, const Eigen::VectorXd &x,
             const Eigen::VectorXd &b) {
    if (A.rows() == 0) return 0.0;
    return (A * x - b).lpNorm<Eigen::Infinity>();
}
double IneqViolation(const Eigen::MatrixXd &A, const Eigen::VectorXd &x,
                     const Eigen::VectorXd &b) {
    if (A.rows() == 0) return 0.0;
    return std::max(0.0, (A * x - b).maxCoeff());
}
double Objective(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                 const Eigen::VectorXd &x) {
    return 0.5 * x.dot(H * x) + g.dot(x);
}
struct RunResult {
    bool solved = false;
    int iterations = 0;
    std::string failure;
    go2_control::DenseQpActiveSetDiagnostics diagnostic;
    Eigen::VectorXd x;
};
RunResult Solve(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                const Eigen::MatrixXd &Ai, const Eigen::VectorXd &bi,
                const Eigen::MatrixXd &Ae, const Eigen::VectorXd &be,
                const Eigen::VectorXd &seed) {
    RunResult r;
    r.solved = go2_control::SolveDenseQpPrimalActiveSet(
        H, g, Ai, bi, Ae, be, seed, r.x, r.iterations, &r.diagnostic);
    r.failure = r.diagnostic.failure;
    return r;
}
void PrintTaskResiduals(const RunResult &run,
                        const std::map<std::string, Field> &f) {
    std::cout << "[";
    for (int leg = 0; leg < 4; ++leg) {
        if (leg) std::cout << ',';
        std::cout << "{\"leg\":" << leg << ",\"available\":"
                  << (run.solved ? "true" : "false") << ",\"vector_mps2\":";
        if (!run.solved) { std::cout << "null}"; continue; }
        const Eigen::VectorXd r = f.at("J" + std::to_string(leg)).value *
            run.x.head(18) + f.at("bias" + std::to_string(leg)).value.col(0) -
            f.at("task" + std::to_string(leg)).value.col(0);
        std::cout << '[' << Number(r[0]) << ',' << Number(r[1]) << ',' << Number(r[2])
                  << "],\"norm_mps2\":" << Number(r.norm()) << '}';
    }
    std::cout << ']';
}
bool RecoverForce(const RunResult &run, const Eigen::VectorXd &contact,
                  std::vector<std::vector<double>> &force, std::string &error) {
    if (!run.solved || run.x.size() != 24) { error = "unavailable_solver_result"; return false; }
    int ncontact = 0;
    for (int leg = 0; leg < 4; ++leg) ncontact += contact[leg] > 0.5 ? 1 : 0;
    if (run.x.size() - 18 != 3 * ncontact) { error = "force_layout_mismatch"; return false; }
    force.assign(4, std::vector<double>(3, 0.0));
    int offset = 18;
    for (int leg = 0; leg < 4; ++leg) if (contact[leg] > 0.5) {
        for (int j = 0; j < 3; ++j) force[leg][j] = run.x[offset++];
    }
    return true;
}
bool RecoverTorque(const RunResult &run, const Eigen::MatrixXd &Ai,
                   const Eigen::VectorXd &bi, std::vector<double> &tau,
                   std::vector<double> &limits, std::string &error) {
    // WBC emits 2*nf force-cone rows followed by 24 paired torque rows.
    if (!run.solved || run.x.size() != 24 || Ai.rows() < 24) {
        error = "torque_unavailable_missing_solver_result"; return false;
    }
    const int start = Ai.rows() - 24;
    tau.resize(12); limits.resize(12);
    for (int i = 0; i < 12; ++i) {
        const int p = start + 2 * i, m = p + 1;
        const double scale = std::max({1.0, Ai.row(p).norm(), Ai.row(m).norm(),
                                       std::abs(bi[p]), std::abs(bi[m])});
        if ((Ai.row(p) + Ai.row(m)).norm() > 1e-8 * scale) {
            error = "torque_pair_structure_missing"; return false;
        }
        limits[i] = 0.5 * (bi[p] + bi[m]);
        const double h = 0.5 * (bi[m] - bi[p]);
        tau[i] = Ai.row(p).dot(run.x) + h;
    }
    return true;
}
void PrintRun(const char *label, const RunResult &run,
              const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
              const Eigen::MatrixXd &Ai, const Eigen::VectorXd &bi,
              const Eigen::MatrixXd &Ae, const Eigen::VectorXd &be,
              const Eigen::VectorXd &seed, const Eigen::VectorXd &contact,
              const std::map<std::string, Field> &f) {
    std::cout << "{\"label\":" << Quote(label)
              << ",\"solved\":" << (run.solved ? "true" : "false")
              << ",\"numerical_failure_is_not_infeasibility\":true"
              << ",\"failure\":" << Quote(run.failure)
              << ",\"iterations\":" << run.iterations
              << ",\"diagnostic\":{\"equality_rank\":" << run.diagnostic.equality_rank
              << ",\"equality_residual\":" << Number(run.diagnostic.equality_residual)
              << ",\"inequality_violation\":" << Number(run.diagnostic.inequality_violation)
              << ",\"stationarity_residual\":" << Number(run.diagnostic.stationarity_residual)
              << ",\"active_count\":" << run.diagnostic.active_count << '}'
              << ",\"objective\":";
    if (!run.solved) std::cout << "null";
    else std::cout << Number(Objective(H, g, run.x));
    std::cout << ",\"qdd_norm\":";
    if (!run.solved) std::cout << "null"; else std::cout << Number(run.x.head(18).norm());
    std::cout << ",\"original_constraint_certificate\":{\"seed_eq_inf\":"
              << Number(EqInf(Ae, seed, be)) << ",\"seed_ineq_max\":"
              << Number(IneqViolation(Ai, seed, bi)) << ",\"solution_eq_inf\":";
    if (!run.solved) std::cout << "null";
    else std::cout << Number(EqInf(Ae, run.x, be));
    std::cout << ",\"solution_ineq_max\":";
    if (!run.solved) std::cout << "null";
    else std::cout << Number(IneqViolation(Ai, run.x, bi));
    const bool cert = run.solved && EqInf(Ae, run.x, be) <= 5e-7 &&
        IneqViolation(Ai, run.x, bi) <= 5e-7;
    std::cout << ",\"seed_feasible\":" << (EqInf(Ae, seed, be) <= 2e-7 &&
        IneqViolation(Ai, seed, bi) <= 2e-7 ? "true" : "false")
              << ",\"solution_constraints_satisfied\":" << (cert ? "true" : "false") << '}'
              << ",\"task_residuals\":";
    PrintTaskResiduals(run, f);
    std::vector<std::vector<double>> force; std::string force_error;
    std::cout << ",\"force_by_leg_N\":";
    if (!RecoverForce(run, contact, force, force_error)) std::cout << "null";
    else {
        std::cout << '[';
        for (int leg = 0; leg < 4; ++leg) {
            if (leg) std::cout << ',';
            PrintArray(force[leg]);
        }
        std::cout << ']';
    }
    std::vector<double> tau, limits; std::string tau_error;
    std::cout << ",\"torque_nm\":";
    if (!RecoverTorque(run, Ai, bi, tau, limits, tau_error)) std::cout << "null";
    else PrintArray(tau);
    std::cout << ",\"torque_limit_nm\":";
    if (limits.empty()) std::cout << "null"; else PrintArray(limits);
    std::cout << ",\"unavailable_reasons\":{\"force\":"
              << Quote(force_error) << ",\"torque\":" << Quote(tau_error) << "}}";
}
}  // namespace
int main(int argc, char **argv) {
    if (argc != 2) { Fail("usage: replay_joint_execution_qp FIXTURE"); return 2; }
    std::map<std::string, Field> f; std::string error;
    if (!ReadFixture(argv[1], f, error)) { Fail(error); return 2; }
    const auto require = [&](const std::string &n, int r, int c) {
        return Shape(f, n, r, c, error);
    };
    if (!require("H",24,24) || !require("g",24,1) ||
        !require("Aineq",36,24) || !require("bineq",36,1) ||
        !require("Aeq",18,24) || !require("beq",18,1) ||
        !require("seed",24,1) || !require("iterate",24,1) ||
        !require("w_posture",1,1) || !require("contact",4,1)) {
        Fail(error); return 2;
    }
    for (int leg = 0; leg < 4; ++leg) {
        if (!require("J"+std::to_string(leg),3,18) ||
            !require("bias"+std::to_string(leg),3,1) ||
            !require("task"+std::to_string(leg),3,1)) { Fail(error); return 2; }
    }
    const double w_posture = f.at("w_posture").value(0,0);
    if (!std::isfinite(w_posture) || w_posture < 0.0) { Fail("invalid_w_posture"); return 2; }
    const Eigen::VectorXd contact = f.at("contact").value.col(0);
    for (int leg = 0; leg < 4; ++leg)
        if (!(contact[leg] == 0.0 || contact[leg] == 1.0)) { Fail("contact_not_strict_binary"); return 2; }
    const Eigen::MatrixXd H = f.at("H").value;
    const Eigen::VectorXd g = f.at("g").value.col(0);
    const Eigen::MatrixXd Ai = f.at("Aineq").value;
    const Eigen::VectorXd bi = f.at("bineq").value.col(0);
    const Eigen::MatrixXd Ae = f.at("Aeq").value;
    const Eigen::VectorXd be = f.at("beq").value.col(0);
    const Eigen::VectorXd seed = f.at("seed").value.col(0);
    const Eigen::VectorXd iterate = f.at("iterate").value.col(0);
    if ((H-H.transpose()).norm() > 1e-9*std::max(1.0,H.norm())) { Fail("H_not_symmetric"); return 2; }
    const RunResult baseline = Solve(H,g,Ai,bi,Ae,be,seed);
    Eigen::MatrixXd Hab = H;
    for (int i = 6; i < 18; ++i) Hab(i,i) -= 2.0*w_posture;
    if (!Hab.allFinite()) { Fail("ablation_H_nonfinite"); return 2; }
    const RunResult ablation = Solve(Hab,g,Ai,bi,Ae,be,seed);
    std::cout << "{\"ok\":true,\"input\":" << Quote(argv[1])
              << ",\"dimensions\":{\"variables\":24,\"equalities\":18,\"inequalities\":36}"
              << ",\"w_posture\":" << Number(w_posture)
              << ",\"contact_mask\":"; PrintVec(contact);
    std::cout << ",\"ablation\":{\"changed\":\"H[6:18,6:18] diagonal minus 2*w_posture\",\"equalities_reused\":true,\"other_problem_data_reused\":true}"
              << ",\"baseline\":";
    PrintRun("baseline",baseline,H,g,Ai,bi,Ae,be,seed,contact,f);
    std::cout << ",\"runtime_iterate\":{";
    std::cout << "\"objective\":" << Number(Objective(H,g,iterate))
              << ",\"eq_inf\":" << Number(EqInf(Ae,iterate,be))
              << ",\"ineq_max\":" << Number(IneqViolation(Ai,iterate,bi))
              << ",\"finite\":true";
    if (baseline.solved) {
        std::cout << ",\"replayed_minus_iterate_inf\":" << Number((baseline.x-iterate).lpNorm<Eigen::Infinity>());
        std::cout << ",\"objective_delta\":" << Number(Objective(H,g,baseline.x)-Objective(H,g,iterate));
    }
    std::cout << "},\"ablation_result\":";
    PrintRun("ablation_w_posture_zero",ablation,Hab,g,Ai,bi,Ae,be,seed,contact,f);
    std::cout << "}\n";
    // A successful solve alone is not a replay match.
    const bool matched = baseline.solved &&
        (baseline.x-iterate).lpNorm<Eigen::Infinity>() <= 1e-8;
    return (matched && ablation.solved) ? 0 : 3;
}
