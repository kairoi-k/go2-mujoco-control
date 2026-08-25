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

add_executable(test_trot_task tests/test_trot_task.cpp trot/trot_task.cpp)

add_executable(test_contact_state_filter tests/test_contact_state_filter.cpp)

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
    target_link_libraries(test_go2_rigid_body "${_go2_mujoco_lib}" Eigen3::Eigen)
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
endif()

add_executable(test_srbd_mpc tests/test_srbd_mpc.cpp)
target_link_libraries(test_srbd_mpc Eigen3::Eigen)
go2_add_ctest(test_srbd_mpc)

add_executable(test_terrain_interfaces tests/test_terrain_interfaces.cpp)
target_link_libraries(test_terrain_interfaces unitree_sdk2)
go2_add_ctest(test_terrain_interfaces)

add_executable(analyze_contact_torque_replay tools/analysis/analyze_contact_torque_replay.cpp)
target_include_directories(analyze_contact_torque_replay PRIVATE ${GO2_CPP_INCLUDE_DIRS})
target_link_libraries(analyze_contact_torque_replay Eigen3::Eigen)

go2_add_ctest(test_go2_inverse_kinematics)
go2_add_ctest(test_raibert_footstep_planner)
go2_add_ctest(test_raibert_trot_kernel)
go2_add_ctest(test_motion_frame_utils)
go2_add_ctest(test_trot_task)
go2_add_ctest(test_velocity_filter)
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
