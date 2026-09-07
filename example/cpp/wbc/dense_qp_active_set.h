#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
namespace go2_control {
// Feasible-seed primal active set for
//   min .5 x'Hx + g'x, Aineq*x <= bineq, Aeq*x = beq.
// The supplied seed is mandatory: this helper never treats a failed numerical
// solve as proof of infeasibility and never relaxes the caller's constraints.
struct DenseQpActiveSetDiagnostics {
  int equality_rank = 0;
  int iterations = 0;
  int active_count = 0;
  double equality_residual = std::numeric_limits<double>::infinity();
  double inequality_violation = std::numeric_limits<double>::infinity();
  double stationarity_residual = std::numeric_limits<double>::infinity();
  std::vector<int> active_rows;
  std::string failure;
};
inline bool SolveDenseQpPrimalActiveSet(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
    const Eigen::MatrixXd& Aineq, const Eigen::VectorXd& bineq,
    const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq,
    const Eigen::VectorXd& feasible_seed, Eigen::VectorXd& x,
    int& iterations, DenseQpActiveSetDiagnostics* diagnostics = nullptr) {
  constexpr double kRankTol = 1e-11;
  constexpr double kSeedTol = 2e-7;
  constexpr double kAcceptTol = 5e-7;
  constexpr double kDirectionTol = 1e-11;
  constexpr double kKktTol = 2e-8;
  constexpr double kMultiplierTol = 1e-9;
  constexpr int kMaxIterations = 1024;
  iterations = 0;
  x.resize(0);
  if (diagnostics != nullptr) *diagnostics = DenseQpActiveSetDiagnostics{};
  auto fail = [&](const char* why) {
    if (diagnostics != nullptr) diagnostics->failure = why;
    x.resize(0);
    return false;
  };
  auto inf_norm = [](const Eigen::VectorXd& v) {
    return v.size() == 0 ? 0.0 : v.lpNorm<Eigen::Infinity>();
  };
  auto max_violation = [&](const Eigen::VectorXd& v) {
    if (v.size() == 0) return 0.0;
    return std::max(0.0, v.maxCoeff());
  };
  auto finite = [](const Eigen::MatrixXd& m) { return m.allFinite(); };
  const int n = H.rows();
  if (n <= 0 || H.cols() != n || g.size() != n ||
      Aineq.cols() != n || bineq.size() != Aineq.rows() ||
      Aeq.cols() != n || beq.size() != Aeq.rows() ||
      feasible_seed.size() != n || !finite(H) || !g.allFinite() ||
      !finite(Aineq) || !bineq.allFinite() || !finite(Aeq) ||
      !beq.allFinite() || !feasible_seed.allFinite()) {
    return fail("invalid_dimensions_or_nonfinite_input");
  }
  const double hscale = std::max(1.0, H.norm());
  if ((H - H.transpose()).norm() > 1e-9 * hscale) {
    return fail("H_not_symmetric");
  }
  const Eigen::MatrixXd Hsym = (H + H.transpose()) * 0.5;
  // Find x0 and a rank-revealing equality nullspace x=x0+N*y.
  Eigen::VectorXd x0;
  Eigen::MatrixXd N;
  int equality_rank = 0;
  if (Aeq.rows() == 0) {
    x0 = Eigen::VectorXd::Zero(n);
    N = Eigen::MatrixXd::Identity(n, n);
  } else {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        Aeq, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::VectorXd s = svd.singularValues();
    const double s0 = s.size() == 0 ? 0.0 : s[0];
    const double cutoff = kRankTol * std::max(1.0, s0);
    Eigen::VectorXd sinv = Eigen::VectorXd::Zero(s.size());
    while (equality_rank < s.size() && s[equality_rank] > cutoff) {
      sinv[equality_rank] = 1.0 / s[equality_rank];
      ++equality_rank;
    }
    const int k = s.size();
    x0 = svd.matrixV().leftCols(k) * sinv.asDiagonal() *
         svd.matrixU().leftCols(k).transpose() * beq;
    N = svd.matrixV().rightCols(n - equality_rank);
  }
  const double x0_eq = inf_norm(Aeq * x0 - beq);
  if (!std::isfinite(x0_eq) || x0_eq > kSeedTol) {
    return fail("inconsistent_equalities");
  }
  if (diagnostics != nullptr) diagnostics->equality_rank = equality_rank;
  const double seed_eq = inf_norm(Aeq * feasible_seed - beq);
  const double seed_ineq = max_violation(Aineq * feasible_seed - bineq);
  if (!std::isfinite(seed_eq) || !std::isfinite(seed_ineq) ||
      seed_eq > kSeedTol || seed_ineq > kSeedTol) {
    return fail("feasible_seed_rejected");
  }
  const int d = N.cols();
  if (d == 0) {
    x = x0;
    const double eq = inf_norm(Aeq * x - beq);
    const double iv = max_violation(Aineq * x - bineq);
    if (eq > kAcceptTol || iv > kAcceptTol) return fail("fixed_point_not_feasible");
    iterations = 1;
    if (diagnostics != nullptr) {
      diagnostics->iterations = iterations;
      diagnostics->equality_residual = eq;
      diagnostics->inequality_violation = iv;
    }
    return true;
  }
  const Eigen::MatrixXd Hred = N.transpose() * Hsym * N;
  Eigen::LLT<Eigen::MatrixXd> llt(Hred);
  if (llt.info() != Eigen::Success) return fail("reduced_H_not_SPD");
  const Eigen::MatrixXd L = llt.matrixL();
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(d, d);
  const Eigen::MatrixXd T =
      L.transpose().triangularView<Eigen::Upper>().solve(I);  // y=T*z
  const Eigen::VectorXd yseed = N.transpose() * (feasible_seed - x0);
  const Eigen::VectorXd zseed = L.transpose() * yseed;
  const double coordinate_scale = std::max(1.0, zseed.norm());
  Eigen::VectorXd w = zseed / coordinate_scale;  // z=scale*w, H is identity.
  const Eigen::VectorXd gr = N.transpose() * (Hsym * x0 + g);
  const Eigen::VectorXd gz =
      L.triangularView<Eigen::Lower>().solve(gr);
  const Eigen::VectorXd grad = gz / coordinate_scale;
  // Whitened and positively row-scaled constraints are used only for the
  // search. Every acceptance check below is in the caller's original rows.
  Eigen::MatrixXd C = Aineq * N * T * coordinate_scale;
  Eigen::VectorXd rhs = bineq - Aineq * x0;
  for (int r = 0; r < C.rows(); ++r) {
    const double row_scale = std::max(1.0, C.row(r).norm());
    C.row(r) /= row_scale;
    rhs[r] /= row_scale;
  }
  if (!C.allFinite() || !rhs.allFinite() || !w.allFinite() || !grad.allFinite()) {
    return fail("nonfinite_reduced_problem");
  }
  if (max_violation(C * w - rhs) > kSeedTol) {
    return fail("seed_reconstruction_not_feasible");
  }
  const int m = C.rows();
  std::vector<int> working;
  std::vector<char> active(static_cast<size_t>(m), 0);
  std::vector<char> ignored(static_cast<size_t>(m), 0);
  auto rank_of = [&](const Eigen::MatrixXd& M) {
    if (M.rows() == 0 || M.cols() == 0) return 0;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
    lu.setThreshold(kRankTol);
    return static_cast<int>(lu.rank());
  };
  auto add_if_independent = [&](int row) {
    if (active[static_cast<size_t>(row)] ||
        static_cast<int>(working.size()) >= d) return false;
    Eigen::MatrixXd W(static_cast<int>(working.size()), d);
    for (int i = 0; i < W.rows(); ++i) W.row(i) = C.row(working[i]);
    Eigen::MatrixXd Wnew(W.rows() + 1, d);
    if (W.rows() != 0) Wnew.topRows(W.rows()) = W;
    Wnew.row(W.rows()) = C.row(row);
    if (rank_of(Wnew) <= rank_of(W)) return false;
    working.push_back(row);
    active[static_cast<size_t>(row)] = 1;
    return true;
  };
  auto x_from_w = [&]() -> Eigen::VectorXd {
    const Eigen::VectorXd y = T * (coordinate_scale * w);
    return x0 + N * y;
  };
  auto original_ok = [&](const Eigen::VectorXd& candidate) {
    const double eq = inf_norm(Aeq * candidate - beq);
    const double iv = max_violation(Aineq * candidate - bineq);
    return std::isfinite(eq) && std::isfinite(iv) && eq <= kAcceptTol && iv <= kAcceptTol;
  };
  auto finalize = [&](const Eigen::VectorXd& lambda) {
    x = x_from_w();
    const double eq = inf_norm(Aeq * x - beq);
    const double iv = max_violation(Aineq * x - bineq);
    Eigen::VectorXd stationarity = w + grad;
    if (!working.empty()) {
      Eigen::MatrixXd CW(static_cast<int>(working.size()), d);
      for (int i = 0; i < CW.rows(); ++i) CW.row(i) = C.row(working[i]);
      stationarity += CW.transpose() * lambda;
    }
    const double st = inf_norm(stationarity);
    if (!std::isfinite(eq) || !std::isfinite(iv) || !std::isfinite(st) ||
        eq > kAcceptTol || iv > kAcceptTol || st > 5e-6) {
      return fail("final_original_or_KKT_check_failed");
    }
    if (diagnostics != nullptr) {
      diagnostics->iterations = iterations;
      diagnostics->active_count = static_cast<int>(working.size());
      diagnostics->active_rows = working;
      diagnostics->equality_residual = eq;
      diagnostics->inequality_violation = iv;
      diagnostics->stationarity_residual = st;
    }
    return true;
  };
  for (int it = 0; it < kMaxIterations; ++it) {
    iterations = it + 1;
    const Eigen::VectorXd grad_now = w + grad;
    Eigen::VectorXd direction;
    Eigen::VectorXd multipliers;
    if (working.empty()) {
      direction = -grad_now;
      multipliers.resize(0);
    } else {
      const int k = static_cast<int>(working.size());
      Eigen::MatrixXd CW(k, d);
      for (int i = 0; i < k; ++i) CW.row(i) = C.row(working[i]);
      Eigen::MatrixXd K = Eigen::MatrixXd::Zero(d + k, d + k);
      K.topLeftCorner(d, d).setIdentity();
      K.topRightCorner(d, k) = CW.transpose();
      K.bottomLeftCorner(k, d) = CW;
      Eigen::VectorXd rhs_kkt = Eigen::VectorXd::Zero(d + k);
      rhs_kkt.head(d) = -grad_now;
      Eigen::FullPivLU<Eigen::MatrixXd> lu(K);
      if (!lu.isInvertible()) return fail("active_KKT_singular");
      const Eigen::VectorXd sol = lu.solve(rhs_kkt);
      const double kres = inf_norm(K * sol - rhs_kkt);
      if (!sol.allFinite() || !std::isfinite(kres) ||
          kres > kKktTol * std::max(1.0, rhs_kkt.norm())) {
        return fail("active_KKT_numerical_failure");
      }
      direction = sol.head(d);
      multipliers = sol.tail(k);
    }
    const double direction_limit =
        kDirectionTol * std::max({1.0, w.norm(), grad.norm()});
    if (direction.norm() <= direction_limit) {
      if (!working.empty()) {
        double min_multiplier = std::numeric_limits<double>::infinity();
        int drop = -1;
        for (int i = 0; i < multipliers.size(); ++i) {
          if (multipliers[i] < min_multiplier) {
            min_multiplier = multipliers[i];
            drop = i;
          }
        }
        const double multiplier_limit =
            kMultiplierTol * std::max({1.0, grad_now.norm(), multipliers.norm()});
        if (drop >= 0 && min_multiplier < -multiplier_limit) {
          active[static_cast<size_t>(working[drop])] = 0;
          working.erase(working.begin() + drop);
          std::fill(ignored.begin(), ignored.end(), 0);
          continue;
        }
      }
      return finalize(multipliers);
    }
    double alpha = 1.0;
    double best_ratio = std::numeric_limits<double>::infinity();
    int blocker = -1;
    for (int r = 0; r < m; ++r) {
      if (active[static_cast<size_t>(r)] || ignored[static_cast<size_t>(r)]) continue;
      const double ap = C.row(r).dot(direction);
      if (ap <= direction_limit) continue;
      const double slack = rhs[r] - C.row(r).dot(w);
      if (slack < -kSeedTol) return fail("current_search_point_infeasible");
      const double ratio = std::max(0.0, slack / ap);
      if (ratio < best_ratio - 1e-12 ||
          (std::abs(ratio - best_ratio) <= 1e-12 && (blocker < 0 || r < blocker))) {
        best_ratio = ratio;
        blocker = r;
      }
    }
    const bool hit = blocker >= 0 && best_ratio <= 1.0 + 1e-10;
    if (hit) alpha = std::min(1.0, best_ratio);
    w += alpha * direction;
    if (!w.allFinite()) return fail("nonfinite_search_point");
    const Eigen::VectorXd candidate = x_from_w();
    if (!original_ok(candidate)) return fail("search_step_left_original_feasible_set");
    if (hit) {
      if (!add_if_independent(blocker)) {
        // A dependent tight row is implied by the independent working set.
        // Skip it for this direction; never add a dependent KKT row.
        ignored[static_cast<size_t>(blocker)] = 1;
      } else {
        std::fill(ignored.begin(), ignored.end(), 0);
      }
    } else {
      std::fill(ignored.begin(), ignored.end(), 0);
    }
  }
  return fail("active_set_iteration_limit");
}
}  // namespace go2_control
