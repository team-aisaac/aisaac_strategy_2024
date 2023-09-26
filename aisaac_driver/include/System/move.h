#pragma once
#include "System/accel_limited_control.h"

/// @brief 目標位置へ向かう
/// @param robot_goal_pose 目標姿勢
/// @param current_pose 現在姿勢
/// @param pid_x
/// @param pid_y
/// @param pid_theta
/// @param inout_velocity input: 1ループ前の目標vx,vy,omega. output: 目標vx,vy,omega
void goTargetPose(const State2D *target_pose, const State2D *current_pose, const State2D *current_velocity,
                  RobotController *robot_controller, State2D *output_velocity)
{
    State2D error = {0}; // 偏差
    diffState2D(target_pose, current_pose, &error);

    //// xy速度を計算
    State2D velocity_from_pid = {0};
    calcVelocityFromPID(ROBOT_MAX_ACCEL, POSITION_CONTROL_DEAD_BAND, VEROSITY_CONTROL_THRESHOLD,
                        &error, &(robot_controller->pid_x), &(robot_controller->pid_y), &velocity_from_pid);
    // 加速度制約
    velocityConstraint(robot_controller->max_robot_velocity, ROBOT_MAX_ACCEL, CONTROL_DT, current_velocity, &velocity_from_pid, output_velocity);

    //// 角速度計算 x=theta, y=0とすることで、XY用の関数を流用可能にしている
    State2D error_for_omega = {0};
    State2D omega_from_pid = {0};
    _pid_t dummy = {0};
    error_for_omega.x = angle_range_corrector_deg2(error.theta); //[-180, 180]に収める
    calcVelocityFromPID(ROBOT_MAX__ROTATE_ACCELE, 0, 0,
                        &error_for_omega, &(robot_controller->pid_theta), &dummy, &omega_from_pid);
    // 加速度制約 x=theta, y=0とすることで、XY用の関数を流用可能にしている
    State2D current_omega = {0};
    State2D output_omega = {0};
    current_omega.x = current_velocity->theta;
    velocityConstraint(SAFETY_MACHINE_OMEGA, ROBOT_MAX__ROTATE_ACCELE, CONTROL_DT, &current_omega, &omega_from_pid, &output_omega);
    output_velocity->theta = output_omega.x;
}

/// @brief StrategyPcCommandに従って移動. actuatorは動かさない
/// @param strategy_pc_command
/// @param world
/// @param robot_controller
/// @param output
void move(const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    output->actuator_type = ACTION_MOVE;
    output->actuator_value = 0;
    goTargetPose(&(strategy_pc_command->robot_goal_pose), &(world->robot_pose), &(world->robot_velocity), robot_controller, &(output->velocity));
}
