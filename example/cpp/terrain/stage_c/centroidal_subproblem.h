#pragma once

#include <Eigen/Dense>
#include "joint_planner.h"
#include "srbd_mpc.h"

namespace go2_terrain { namespace stage_c {

using CentroidalState = Eigen::Matrix<double, 9, 1>; // world c, v, L about COM
using ForceArray = std::array<Eigen::Vector3d, 4>;

struct ContactSurface {
    // Right-handed orthonormal columns (t1,t2,n), all in world frame.
    Eigen::Matrix3d basis_world = Eigen::Matrix3d::Identity();
    Frame frame = Frame::kUnknown;
    MapCoverageState coverage = MapCoverageState::kMetadataUnavailable;
    std::uint64_t map_epoch = 0;
    TimeNs valid_until{};
    double friction_mu = 0.0;
    double min_normal_n = 0.0;
    double max_normal_n = 0.0;
};

struct FixedScheduleInterval {
    TimeNs start{}, end{};
    std::array<bool, 4> contact{};
    // -1 is an explicitly observed initial anchor; >=0 is an event index.
    std::array<int, 4> event_index{{-1, -1, -1, -1}};
};

struct StateBox {
    CentroidalState lower = CentroidalState::Constant(-100.0);
    CentroidalState upper = CentroidalState::Constant(100.0);
};

struct CentroidalProblem {
    JointPlanningRequest request{};
    std::vector<std::size_t> combination;
    // This is a complete preview from the schedule authority, not synthesized
    // from gaps in an event list. No hidden initial stance lifetime.
    std::uint64_t schedule_epoch = 0;
    std::vector<FixedScheduleInterval> schedule;
    std::array<ContactSurface, 4> initial_surfaces{};
    std::vector<ContactSurface> event_surfaces;
    std::vector<TimeNs> grid; // strict absolute state nodes, event-aligned
    TimeNs required_start{}, required_end{};
    Eigen::Vector3d initial_momentum_world = Eigen::Vector3d::Zero();
    bool initial_momentum_valid = false;
    // Same SRBD physical parameters and default reference weights. No second
    // gravity/mass/friction convention. Surface constraints override flat mu.
    go2_control::SrbdMpcParams model{};
    // Bounds apply at nodes and linearly between nodes; finite, mandatory.
    std::vector<StateBox> bounds;
    // Already accepted prefix on this exact grid, including its original times.
    std::vector<TimeNs> committed_state_times;
    std::vector<CentroidalState> committed_states;
    std::vector<ContactForceInterval> committed_forces;
    double w_momentum = 4.0; // explicit (N*m*s)^-2, not an angular-rate weight
    int max_scp_iterations = 6;
    int max_qp_iterations = 1200;
    double force_trust_n = 80.0;
};

struct DynamicsResidual {
    double position_m = 0.0;
    double velocity_mps = 0.0;
    double momentum_nms = 0.0;
    double force_n = 0.0;
    double state_bound = 0.0;
    double initial = 0.0;
};

struct SeparationWitness {
    bool valid = false;
    int interval = -1;
    Eigen::Vector3d linear_direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_direction = Eigen::Vector3d::Zero();
    double required_lower = 0.0;
    double attainable_upper = 0.0;
};

struct DynamicsCertificate {
    // A certificate for this reduced continuous centroidal model ONLY.
    // No orientation, IK, swept collision, torque or execution certification.
    bool feasible = false;
    bool input_checked = false;
    bool coverage_checked = false;
    bool commitment_checked = false;
    bool original_dynamics_checked = false;
    bool full_geometry_checked = false;
    bool geometric_15mm_checked = false;
    bool geometric_15mm_pass = false;
    FrozenContractConflict frozen_conflict = FrozenContractConflict::kNone;
    DynamicsResidual residual{};
    SeparationWitness separation{};
};

struct CentroidalResult {
    JointPlannerFailure failure = JointPlannerFailure::kInvalidInput;
    std::string detail;
    std::vector<CentroidalState> states;
    std::vector<ContactForceInterval> forces;
    DynamicsCertificate certificate{};
    double cost = std::numeric_limits<double>::infinity();
    int scp_iterations = 0;
    int qp_iterations = 0;
};

// Fixed tolerances for development model certificates, not B0/B1 thresholds.
constexpr double kDynamicsTolerance = 2e-7;
constexpr double kForceTolerance = 2e-5;
constexpr double kStateTolerance = 2e-6;

struct CentroidalSample {
    bool state_valid = false;
    bool force_valid = false; // false at the terminal state, never clamped
    CentroidalState state = CentroidalState::Zero();
    ContactForceInterval force{};
};
CentroidalSample SampleCentroidalTrajectory(const CentroidalProblem &problem,
    const CentroidalResult &result, TimeNs time);

CentroidalResult SolveCentroidalSubproblem(const CentroidalProblem &problem);
// Independent of QP matrices and solver success, checks the original equations
// with Simpson torque quadrature and exact polynomial extrema between nodes.
DynamicsCertificate VerifyCentroidalTrajectory(
    const CentroidalProblem &problem, const CentroidalResult &result);
// The existing exhaustive search consumes this explicitly reduced certificate;
// it must never be promoted to an accepted execution bundle by this helper.
JointEvaluation AsJointEvaluation(const CentroidalProblem &problem,
                                  const CentroidalResult &result);

}} // namespace
