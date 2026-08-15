cmake_minimum_required(VERSION 3.16)
project(stand_go2)
enable_testing()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

add_executable(hold_pose_go2 hold_pose_go2.cpp)
target_link_libraries(hold_pose_go2  unitree_sdk2)

add_executable(track_lowcmd_lowstate_go2 track_lowcmd_lowstate_go2.cpp)
target_link_libraries(track_lowcmd_lowstate_go2  unitree_sdk2)

add_executable(single_leg_lift_go2 single_leg_lift_go2.cpp)
target_link_libraries(single_leg_lift_go2 unitree_sdk2)

add_executable(real_leg_lift_go2
    real_leg_lift_go2.cpp
    leg_lift_cli.cpp
    leg_lift_lifecycle.cpp
    leg_lift_world.cpp
    leg_lift_diagnostics.cpp
    leg_lift_control.cpp
    leg_lift_types.h
    leg_lift_experiment.h
)
target_link_libraries(real_leg_lift_go2 unitree_sdk2)

add_executable(real_trot_go2
    real_trot_go2.cpp
    trot_cli.cpp
    trot_experiment_lifecycle.cpp
    trot_experiment_gait.cpp
    trot_experiment_wbc.cpp
    trot_experiment_diagnostics.cpp
    trot_experiment_control.cpp
    trot_types.h
    trot_experiment.h
    trot_true_dynamics.h
    locomotion_kernel.h
    contact_state_filter.h
)
target_link_libraries(real_trot_go2 unitree_sdk2 Eigen3::Eigen)

function(go2_add_ctest name)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

set(_go2_mujoco_lib
    "${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/mujoco/lib/libmujoco.so")
if(EXISTS "${_go2_mujoco_lib}")
    add_executable(test_go2_forward_kinematics test_go2_forward_kinematics.cpp)
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

add_executable(test_go2_inverse_kinematics test_go2_inverse_kinematics.cpp)

add_executable(test_raibert_footstep_planner test_raibert_footstep_planner.cpp)

add_executable(test_raibert_trot_kernel test_raibert_trot_kernel.cpp)
target_link_libraries(test_raibert_trot_kernel Eigen3::Eigen)

add_executable(test_motion_frame_utils test_motion_frame_utils.cpp)

add_executable(test_velocity_filter test_velocity_filter.cpp)

add_executable(test_contact_state_filter test_contact_state_filter.cpp)

SET(CMAKE_BUILD_TYPE Release)

add_executable(test_contact_wrench_allocator test_contact_wrench_allocator.cpp)
target_link_libraries(test_contact_wrench_allocator Eigen3::Eigen)

add_executable(test_go2_leg_jacobian test_go2_leg_jacobian.cpp)
target_link_libraries(test_go2_leg_jacobian Eigen3::Eigen)

add_executable(test_contact_wrench_constraints test_contact_wrench_constraints.cpp)
target_link_libraries(test_contact_wrench_constraints Eigen3::Eigen)
add_executable(test_contact_torque_mapping test_contact_torque_mapping.cpp)
target_link_libraries(test_contact_torque_mapping Eigen3::Eigen)

add_executable(test_projected_contact_wrench_allocator test_projected_contact_wrench_allocator.cpp)
target_link_libraries(test_projected_contact_wrench_allocator Eigen3::Eigen)

add_executable(test_contact_wrench_lexicographic_allocator test_contact_wrench_lexicographic_allocator.cpp)
target_link_libraries(test_contact_wrench_lexicographic_allocator Eigen3::Eigen)

add_executable(test_dynamic_acceleration_target test_dynamic_acceleration_target.cpp)

add_executable(test_wbc_runtime_gate test_wbc_runtime_gate.cpp)

add_executable(test_centroidal_wbc test_centroidal_wbc.cpp)

add_executable(test_preview_footstep_horizon test_preview_footstep_horizon.cpp)
target_link_libraries(test_preview_footstep_horizon Eigen3::Eigen)

add_executable(test_dense_qp test_dense_qp.cpp)
target_link_libraries(test_dense_qp Eigen3::Eigen)

add_executable(test_contact_wrench_qp test_contact_wrench_qp.cpp)
target_link_libraries(test_contact_wrench_qp Eigen3::Eigen)

add_executable(analyze_contact_torque_replay tools/analysis/analyze_contact_torque_replay.cpp)
target_include_directories(analyze_contact_torque_replay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(analyze_contact_torque_replay Eigen3::Eigen)

go2_add_ctest(test_go2_inverse_kinematics)
go2_add_ctest(test_raibert_footstep_planner)
go2_add_ctest(test_raibert_trot_kernel)
go2_add_ctest(test_motion_frame_utils)
go2_add_ctest(test_velocity_filter)
go2_add_ctest(test_contact_state_filter)
go2_add_ctest(test_contact_wrench_allocator)
go2_add_ctest(test_go2_leg_jacobian)
go2_add_ctest(test_contact_wrench_constraints)
go2_add_ctest(test_contact_torque_mapping)
go2_add_ctest(test_projected_contact_wrench_allocator)
go2_add_ctest(test_contact_wrench_lexicographic_allocator)
go2_add_ctest(test_dynamic_acceleration_target)
go2_add_ctest(test_wbc_runtime_gate)
go2_add_ctest(test_centroidal_wbc)
go2_add_ctest(test_preview_footstep_horizon)
go2_add_ctest(test_dense_qp)
go2_add_ctest(test_contact_wrench_qp)
