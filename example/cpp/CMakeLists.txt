cmake_minimum_required(VERSION 3.16)
project(stand_go2)
enable_testing()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Eigen3 REQUIRED)

set(GO2_CPP_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/apps
    ${CMAKE_CURRENT_SOURCE_DIR}/trot
    ${CMAKE_CURRENT_SOURCE_DIR}/gait
    ${CMAKE_CURRENT_SOURCE_DIR}/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/contact
    ${CMAKE_CURRENT_SOURCE_DIR}/wbc
    ${CMAKE_CURRENT_SOURCE_DIR}/util
    ${CMAKE_CURRENT_SOURCE_DIR}/leg_lift
    ${CMAKE_CURRENT_SOURCE_DIR}/terrain
)
include_directories(${GO2_CPP_INCLUDE_DIRS})

add_executable(stand_go2 apps/stand_go2.cpp)
target_link_libraries(stand_go2 unitree_sdk2)

add_executable(hold_pose_go2 apps/hold_pose_go2.cpp)
target_link_libraries(hold_pose_go2 unitree_sdk2)

add_executable(track_lowcmd_lowstate_go2 apps/track_lowcmd_lowstate_go2.cpp)
target_link_libraries(track_lowcmd_lowstate_go2 unitree_sdk2)

add_executable(single_leg_lift_go2 apps/single_leg_lift_go2.cpp)
target_link_libraries(single_leg_lift_go2 unitree_sdk2)

add_executable(real_leg_lift_go2
    leg_lift/real_leg_lift_go2.cpp
    leg_lift/leg_lift_cli.cpp
    leg_lift/leg_lift_lifecycle.cpp
    leg_lift/leg_lift_world.cpp
    leg_lift/leg_lift_diagnostics.cpp
    leg_lift/leg_lift_control.cpp
)
target_link_libraries(real_leg_lift_go2 unitree_sdk2)

add_executable(real_trot_go2
    trot/real_trot_go2.cpp
    trot/trot_cli.cpp
    trot/trot_task.cpp
    trot/trot_experiment_lifecycle.cpp
    trot/trot_experiment_gait.cpp
    trot/trot_experiment_wbc.cpp
    trot/trot_experiment_diagnostics.cpp
    trot/trot_experiment_control.cpp
)
target_link_libraries(real_trot_go2 unitree_sdk2 Eigen3::Eigen)

set(_go2_mujoco_lib
    "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib/libmujoco.so")
set(_go2_mujoco_inc
    "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/include")
set(_go2_model_path
    "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_robots/go2/go2.xml")
if(EXISTS "${_go2_mujoco_lib}")
    target_include_directories(real_trot_go2 PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(real_trot_go2 "${_go2_mujoco_lib}")
    target_compile_definitions(
        real_trot_go2 PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(
        real_trot_go2 PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
endif()

function(go2_add_ctest name)
    # A Release test must execute assertions, including calls inside assert().
    if(MSVC)
        target_compile_options(${name} PRIVATE /UNDEBUG)
    else()
        target_compile_options(${name} PRIVATE -UNDEBUG)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

if(EXISTS "${_go2_mujoco_lib}")
    add_executable(test_go2_forward_kinematics tests/test_go2_forward_kinematics.cpp)
    target_include_directories(
        test_go2_forward_kinematics
        PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/include")
    target_link_libraries(
        test_go2_forward_kinematics
        "${_go2_mujoco_lib}")
    target_compile_definitions(
        test_go2_forward_kinematics
        PRIVATE GO2_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_robots/go2/go2.xml")
    go2_add_ctest(test_go2_forward_kinematics)
endif()

add_executable(test_go2_inverse_kinematics tests/test_go2_inverse_kinematics.cpp)

add_executable(test_raibert_footstep_planner tests/test_raibert_footstep_planner.cpp)

add_executable(test_raibert_trot_kernel tests/test_raibert_trot_kernel.cpp)
target_link_libraries(test_raibert_trot_kernel Eigen3::Eigen)

add_executable(test_cartesian_world_trot tests/test_cartesian_world_trot.cpp)
go2_add_ctest(test_cartesian_world_trot)

add_executable(test_motion_frame_utils tests/test_motion_frame_utils.cpp)

add_executable(test_velocity_filter tests/test_velocity_filter.cpp)
add_executable(test_velocity_command tests/test_velocity_command.cpp)
go2_add_ctest(test_velocity_command)
add_executable(test_velocity_running_research tests/test_velocity_running_research.cpp)
target_link_libraries(test_velocity_running_research unitree_sdk2)
go2_add_ctest(test_velocity_running_research)

add_executable(test_terrain_swing_lift tests/test_terrain_swing_lift.cpp)
target_link_libraries(test_terrain_swing_lift Eigen3::Eigen)

add_executable(test_terrain_swing_contract tests/test_terrain_swing_contract.cpp)
target_link_libraries(test_terrain_swing_contract unitree_sdk2)

add_executable(test_terrain_commitment_lifecycle
    tests/test_terrain_commitment_lifecycle.cpp)

add_executable(test_trot_task tests/test_trot_task.cpp trot/trot_task.cpp)

add_executable(test_lean_route_guard
    tests/test_lean_route_guard.cpp
    trot/trot_cli.cpp
    trot/trot_task.cpp)
target_link_libraries(test_lean_route_guard unitree_sdk2 Eigen3::Eigen)
go2_add_ctest(test_lean_route_guard)

add_executable(test_contact_state_filter tests/test_contact_state_filter.cpp)
add_executable(test_contact_fusion_fault_harness tests/test_contact_fusion_fault_harness.cpp)
go2_add_ctest(test_contact_fusion_fault_harness)

set(CMAKE_BUILD_TYPE Release)

add_executable(test_contact_wrench_allocator tests/test_contact_wrench_allocator.cpp)
target_link_libraries(test_contact_wrench_allocator Eigen3::Eigen)

add_executable(test_go2_leg_jacobian tests/test_go2_leg_jacobian.cpp)
target_link_libraries(test_go2_leg_jacobian Eigen3::Eigen)

add_executable(test_contact_wrench_constraints tests/test_contact_wrench_constraints.cpp)
target_link_libraries(test_contact_wrench_constraints Eigen3::Eigen)

add_executable(test_contact_torque_mapping tests/test_contact_torque_mapping.cpp)
target_link_libraries(test_contact_torque_mapping Eigen3::Eigen)

add_executable(test_projected_contact_wrench_allocator tests/test_projected_contact_wrench_allocator.cpp)
target_link_libraries(test_projected_contact_wrench_allocator Eigen3::Eigen)

add_executable(test_contact_wrench_lexicographic_allocator tests/test_contact_wrench_lexicographic_allocator.cpp)
target_link_libraries(test_contact_wrench_lexicographic_allocator Eigen3::Eigen)

add_executable(test_dynamic_acceleration_target tests/test_dynamic_acceleration_target.cpp)

add_executable(test_motion_event_response tests/test_motion_event_response.cpp)

add_executable(test_wbc_runtime_gate tests/test_wbc_runtime_gate.cpp)

# Order-108 verification-only controller-side lockstep writer tick gate
# (DDS-free; header-only in trot/lockstep_writer_gate.h).
add_executable(test_lockstep_writer_gate tests/test_lockstep_writer_gate.cpp)
go2_add_ctest(test_lockstep_writer_gate)

# Order-109 verification-only state-synchronous motion clock (DDS-free).
add_executable(test_lockstep_motion_clock tests/test_lockstep_motion_clock.cpp)
go2_add_ctest(test_lockstep_motion_clock)

# Order-109b production call-chain integration test. The controller sources
# are compiled unchanged except for a test-only publish suppression and
# observability seam; the test drives TrotExperiment::LowCmdWrite itself.
add_executable(test_lockstep_motion_clock_integration
    tests/test_lockstep_motion_clock_integration.cpp
    trot/trot_task.cpp
    trot/trot_experiment_lifecycle.cpp
    trot/trot_experiment_gait.cpp
    trot/trot_experiment_wbc.cpp
    trot/trot_experiment_diagnostics.cpp
    trot/trot_experiment_control.cpp
)
target_compile_definitions(test_lockstep_motion_clock_integration
    PRIVATE GO2_TROT_TESTING=1)
target_link_libraries(test_lockstep_motion_clock_integration
    unitree_sdk2 Eigen3::Eigen)
if(EXISTS "${_go2_mujoco_lib}")
    target_include_directories(test_lockstep_motion_clock_integration
        PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(test_lockstep_motion_clock_integration
        "${_go2_mujoco_lib}")
    target_compile_definitions(test_lockstep_motion_clock_integration
        PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(test_lockstep_motion_clock_integration PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
endif()
go2_add_ctest(test_lockstep_motion_clock_integration)

add_executable(test_centroidal_wbc tests/test_centroidal_wbc.cpp)

add_executable(test_preview_footstep_horizon tests/test_preview_footstep_horizon.cpp)
target_link_libraries(test_preview_footstep_horizon Eigen3::Eigen)

add_executable(test_dense_qp tests/test_dense_qp.cpp)
target_link_libraries(test_dense_qp Eigen3::Eigen)

add_executable(test_contact_wrench_qp tests/test_contact_wrench_qp.cpp)
target_link_libraries(test_contact_wrench_qp Eigen3::Eigen)

if(EXISTS "${_go2_mujoco_lib}")
    add_executable(test_go2_rigid_body tests/test_go2_rigid_body.cpp)
    target_include_directories(test_go2_rigid_body PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(test_go2_rigid_body "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2)
    target_compile_definitions(
        test_go2_rigid_body PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(
        test_go2_rigid_body PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(test_go2_rigid_body)

    add_executable(test_inverse_dynamics_wbc tests/test_inverse_dynamics_wbc.cpp)
    target_include_directories(
        test_inverse_dynamics_wbc PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(
        test_inverse_dynamics_wbc "${_go2_mujoco_lib}" Eigen3::Eigen)
    target_compile_definitions(
        test_inverse_dynamics_wbc PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(
        test_inverse_dynamics_wbc PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(test_inverse_dynamics_wbc)
    add_executable(test_id_wbc_certificate tests/test_id_wbc_certificate.cpp)
    target_include_directories(test_id_wbc_certificate PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(test_id_wbc_certificate "${_go2_mujoco_lib}" Eigen3::Eigen)
    set_target_properties(test_id_wbc_certificate PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(test_id_wbc_certificate)
endif()

add_executable(test_srbd_mpc tests/test_srbd_mpc.cpp)
target_link_libraries(test_srbd_mpc Eigen3::Eigen)
go2_add_ctest(test_srbd_mpc)

add_executable(test_terrain_interfaces tests/test_terrain_interfaces.cpp)
target_link_libraries(test_terrain_interfaces unitree_sdk2)
go2_add_ctest(test_terrain_interfaces)

add_executable(test_terrain_execution_consistency
    tests/test_terrain_execution_consistency.cpp)
go2_add_ctest(test_terrain_execution_consistency)

# Stage C C0-01 contract witnesses and C0-02 deterministic discrete core.
# This target is pure/offline and does not enter the terrain control path.
add_executable(test_stage_c_foundations tests/test_stage_c_foundations.cpp)
target_link_libraries(test_stage_c_foundations unitree_sdk2)
go2_add_ctest(test_stage_c_foundations)

add_executable(analyze_contact_torque_replay tools/analysis/analyze_contact_torque_replay.cpp)
target_include_directories(analyze_contact_torque_replay PRIVATE ${GO2_CPP_INCLUDE_DIRS})
target_link_libraries(analyze_contact_torque_replay Eigen3::Eigen)

go2_add_ctest(test_go2_inverse_kinematics)
go2_add_ctest(test_raibert_footstep_planner)
go2_add_ctest(test_raibert_trot_kernel)
go2_add_ctest(test_motion_frame_utils)
go2_add_ctest(test_trot_task)
go2_add_ctest(test_velocity_filter)
go2_add_ctest(test_terrain_swing_lift)
go2_add_ctest(test_terrain_swing_contract)
add_executable(test_terrain_swing_world_audit tests/test_terrain_swing_world_audit.cpp)
target_link_libraries(test_terrain_swing_world_audit unitree_sdk2)
go2_add_ctest(test_terrain_swing_world_audit)
go2_add_ctest(test_terrain_commitment_lifecycle)
go2_add_ctest(test_contact_state_filter)
go2_add_ctest(test_contact_wrench_allocator)
go2_add_ctest(test_go2_leg_jacobian)
go2_add_ctest(test_contact_wrench_constraints)
go2_add_ctest(test_contact_torque_mapping)
go2_add_ctest(test_projected_contact_wrench_allocator)
go2_add_ctest(test_contact_wrench_lexicographic_allocator)
go2_add_ctest(test_dynamic_acceleration_target)
go2_add_ctest(test_motion_event_response)
go2_add_ctest(test_wbc_runtime_gate)
go2_add_ctest(test_centroidal_wbc)
go2_add_ctest(test_preview_footstep_horizon)
go2_add_ctest(test_dense_qp)
go2_add_ctest(test_contact_wrench_qp)

# Offline Stage C core only; no controller dependency or actuation path.
add_library(stage_c_centroidal STATIC terrain/stage_c/centroidal_subproblem.cpp terrain/stage_c/verify_centroidal.cpp)
target_link_libraries(stage_c_centroidal PUBLIC Eigen3::Eigen unitree_sdk2)
add_executable(test_stage_c_centroidal tests/test_stage_c_centroidal.cpp)
target_link_libraries(test_stage_c_centroidal stage_c_centroidal unitree_sdk2)
go2_add_ctest(test_stage_c_centroidal)

add_executable(test_terrain_nominal_com_height tests/test_terrain_nominal_com_height.cpp)
go2_add_ctest(test_terrain_nominal_com_height)

add_executable(test_terrain_map_envelope tests/test_terrain_map_envelope.cpp)
target_link_libraries(test_terrain_map_envelope unitree_sdk2)
go2_add_ctest(test_terrain_map_envelope)
add_executable(test_terrain_map_transport tests/test_terrain_map_transport.cpp)
target_link_libraries(test_terrain_map_transport unitree_sdk2)
go2_add_ctest(test_terrain_map_transport)

add_executable(test_terrain_map_intervals_v2 tests/test_terrain_map_intervals_v2.cpp)
target_link_libraries(test_terrain_map_intervals_v2 unitree_sdk2)
go2_add_ctest(test_terrain_map_intervals_v2)
add_executable(test_stage_c_event_schedule tests/test_stage_c_event_schedule.cpp)
target_link_libraries(test_stage_c_event_schedule stage_c_centroidal unitree_sdk2)
go2_add_ctest(test_stage_c_event_schedule)
if(EXISTS "${_go2_mujoco_lib}")
    add_executable(test_stage_c_body_reconstruction tests/test_stage_c_body_reconstruction.cpp)
    target_include_directories(test_stage_c_body_reconstruction PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(test_stage_c_body_reconstruction "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2)
    target_compile_definitions(test_stage_c_body_reconstruction PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(test_stage_c_body_reconstruction PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(test_stage_c_body_reconstruction)
endif()

if(EXISTS "${_go2_mujoco_lib}")
    add_executable(test_go2_contact_point tests/test_go2_contact_point.cpp)
    target_include_directories(test_go2_contact_point PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(test_go2_contact_point "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2)
    target_compile_definitions(test_go2_contact_point PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(test_go2_contact_point PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(test_go2_contact_point)
endif()

if(EXISTS "${_go2_mujoco_lib}")
    add_executable(test_stage_c_body_acceleration tests/test_stage_c_body_acceleration.cpp)
    target_include_directories(test_stage_c_body_acceleration PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(test_stage_c_body_acceleration "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2)
    target_compile_definitions(test_stage_c_body_acceleration PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(test_stage_c_body_acceleration PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(test_stage_c_body_acceleration)
endif()

if(EXISTS "${_go2_mujoco_lib}")
  foreach(test_name IN ITEMS test_joint_closed_loop_reference test_motor_command_certificate test_stage_c_prepare_joint_problem test_stage_c_feedback_step test_stage_c_centroidal_wbc test_stage_c_model_observation test_stage_c_joint_trajectory test_id_wbc_force_application test_stage_c_articulated_certificate)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE "${_go2_mujoco_inc}")
    target_link_libraries(${test_name} "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2 stage_c_centroidal)
    target_compile_definitions(${test_name} PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
    set_target_properties(${test_name} PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
    go2_add_ctest(${test_name})
  endforeach()
endif()

add_executable(test_stage_c_foot_trajectory tests/test_stage_c_foot_trajectory.cpp)
target_link_libraries(test_stage_c_foot_trajectory stage_c_centroidal unitree_sdk2)
go2_add_ctest(test_stage_c_foot_trajectory)

add_executable(test_stage_c_terrain_candidates tests/test_stage_c_terrain_candidates.cpp)
target_link_libraries(test_stage_c_terrain_candidates stage_c_centroidal unitree_sdk2)
go2_add_ctest(test_stage_c_terrain_candidates)
add_executable(test_stage_c_centroidal_joint_proposal tests/test_stage_c_centroidal_joint_proposal.cpp)
target_link_libraries(test_stage_c_centroidal_joint_proposal stage_c_centroidal unitree_sdk2)
go2_add_ctest(test_stage_c_centroidal_joint_proposal)
add_executable(test_stage_c_anytime_joint_search tests/test_stage_c_anytime_joint_search.cpp)
target_link_libraries(test_stage_c_anytime_joint_search stage_c_centroidal unitree_sdk2)
go2_add_ctest(test_stage_c_anytime_joint_search)

target_link_libraries(real_trot_go2 stage_c_centroidal)
target_link_libraries(test_lockstep_motion_clock_integration stage_c_centroidal)
add_executable(test_stage_c_phase_clock tests/test_stage_c_phase_clock.cpp)
target_link_libraries(test_stage_c_phase_clock unitree_sdk2 Eigen3::Eigen)
go2_add_ctest(test_stage_c_phase_clock)

add_executable(test_stage_c_world_terrain_view tests/test_stage_c_world_terrain_view.cpp)
target_link_libraries(test_stage_c_world_terrain_view unitree_sdk2 Eigen3::Eigen)
go2_add_ctest(test_stage_c_world_terrain_view)
add_executable(test_stage_c_capture_terrain_view tests/test_stage_c_capture_terrain_view.cpp)
target_link_libraries(test_stage_c_capture_terrain_view unitree_sdk2 Eigen3::Eigen)
go2_add_ctest(test_stage_c_capture_terrain_view)
if(EXISTS "${_go2_mujoco_lib}")
  add_executable(replay_stage_c_feedback tools/replay_stage_c_feedback.cpp)
  target_include_directories(replay_stage_c_feedback PRIVATE "${_go2_mujoco_inc}")
  target_link_libraries(replay_stage_c_feedback "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2 stage_c_centroidal)
  target_compile_definitions(replay_stage_c_feedback PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
  set_target_properties(replay_stage_c_feedback PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
endif()

if(EXISTS "${_go2_mujoco_lib}")
  add_executable(replay_joint_shadow_snapshot tools/replay_joint_shadow_snapshot.cpp)
  target_include_directories(replay_joint_shadow_snapshot PRIVATE "${_go2_mujoco_inc}")
  target_link_libraries(replay_joint_shadow_snapshot "${_go2_mujoco_lib}" Eigen3::Eigen unitree_sdk2 stage_c_centroidal)
  target_compile_definitions(replay_joint_shadow_snapshot PRIVATE GO2_MODEL_PATH="${_go2_model_path}")
  set_target_properties(replay_joint_shadow_snapshot PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib")
  add_executable(joint_shadow_snapshot_fixture tests/joint_shadow_snapshot_fixture.cpp)
  target_include_directories(joint_shadow_snapshot_fixture PRIVATE "${_go2_mujoco_inc}")
  target_link_libraries(joint_shadow_snapshot_fixture Eigen3::Eigen unitree_sdk2)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME test_joint_shadow_snapshot_roundtrip COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_joint_shadow_snapshot_roundtrip.py"
    $<TARGET_FILE:joint_shadow_snapshot_fixture> $<TARGET_FILE:replay_joint_shadow_snapshot>
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/scripts/extract_joint_shadow_snapshot.py")
endif()
add_executable(test_stage_c_world_terrain_snapshot tests/test_stage_c_world_terrain_snapshot.cpp)
target_link_libraries(test_stage_c_world_terrain_snapshot unitree_sdk2 Eigen3::Eigen)
go2_add_ctest(test_stage_c_world_terrain_snapshot)

add_executable(test_dense_qp_active_set tests/test_dense_qp_active_set.cpp)
target_link_libraries(test_dense_qp_active_set Eigen3::Eigen)
add_test(NAME test_dense_qp_active_set COMMAND test_dense_qp_active_set "${CMAKE_CURRENT_SOURCE_DIR}/../../docs/research/evidence/joint_feedback_replay_20260907/attempt_0006/qp.txt")
