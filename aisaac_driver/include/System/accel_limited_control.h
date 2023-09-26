#pragma once
#include "System/struct.h"
#include "System/struct_tool.h"
#include "System/pid.h"
#include "System/tools.h"
#include "System/define.h"

/// @brief PIDで速度を計算。制動速度以下であることも確認する
/// @param max_accel 最大加速度
/// @param dead_band 制動をかける最小誤差。この値以下のときはPIDを優先
/// @param velocity_threshold 制動をかける最小速度. この値以下のときはPIDを優先
/// @param error_vector 偏差
/// @param pid_x 
/// @param pid_y 
/// @param output_velocity 目標xy速度 
void calcVelocityFromPID(float max_accel, float dead_band, float velocity_threshold, 
                    const State2D *error_vector, _pid_t *pid_x, _pid_t *pid_y, State2D *output_velocity)
{
    // Calculate the target vector in the world coordinate system
    pid_x->error = error_vector->x;
    pid_y->error = error_vector->y;
    float error = xyNorm(error_vector);
    State2D target_velocity = {0};

    target_velocity.x = pidExecute(&pid_x);
    target_velocity.y = pidExecute(&pid_y);
    float target_velocity_norm = xyNorm(&target_velocity);

    // 現在の速度と目標速度が乖離していないか確認
    float braking_distance = target_velocity_norm * target_velocity_norm / (2 * max_accel);
    if (error < braking_distance && dead_band < error && velocity_threshold < target_velocity_norm)
    {
        // Decelerates the robot when the distance between the current location and the target value is smaller than the braking distance.
        float v_limit = sqrt(2 * error * max_accel);
        // ロボットの指令速度が最大速度を超えていないか
        xyMultiple(&target_velocity, v_limit / target_velocity_norm, &target_velocity);
        target_velocity_norm = v_limit;
    }
    *output_velocity = target_velocity;
}

/// @brief 最大速度制約、加速度制約を加えた速度を算出する
/// @param max_robot_vel 最大速度
/// @param max_accel 最大加速度
/// @param dt 制御周期
/// @param current_velocity 現在xy速度 
/// @param nominal_velocity 制約付与前の目標xy速度 
/// @param output_velocity 
void velocityConstraint(float max_robot_vel,float max_accel, float dt,  const State2D *current_velocity, const State2D *nominal_velocity, State2D *output_velocity)
{

    // 最大速度制約
    State2D result = *target_velocity;
    float target_velocity_norm = xyNorm(target_velocity);
        // robot velocity check
    if (target_velocity_norm > max_robot_vel)
    {
        // the command speed of the robot is exceeding the maximum speed
        xyMultiple(target_velocity, max_robot_vel / target_velocity_norm, &result);
        target_velocity_norm = max_robot_vel;
    }
    
    State2D accel_vector = {0};
    diffState2D(&result, current_velocity, &accel_vector);
    float accel = xyNorm(&accel_vector);
    float max_dv = max_accel * dt;
    if (accel > max_dv)
    {
        result.x = current_velocity->x + accel_vector.x * max_dv / accel;
        result.y = current_velocity->y + accel_vector.y * max_dv / accel;
    }
    *output_velocity = result;
}



// /// @brief PIDで速度を計算。制動速度以下であることも確認する
// /// @param max_accel 最大加速度
// /// @param dead_band 制動をかける最小誤差。この値以下のときはPIDを優先
// /// @param velocity_threshold 制動をかける最小速度. この値以下のときはPIDを優先
// /// @param max_robot_vel 最大速度
// /// @param target_position
// /// @param current_position
// /// @param current_velocity
// /// @param pid
// /// @return
// float calcVelocityFromPID(float max_accel, float dead_band, float velocity_threshold, float target_position, float current_position, float current_velocity, _pid_t *pid)
// {
//     float error = target_position - current_position;
//     float error_abs = fabs(error);
//     float target_velocity = pidExecute(pid);
//     float target_velocity_abs = fabs(target_velocity);

//     // 制動可能速度以下か
//     float v_limit = sqrt(2 * error_abs * max_accel);
//     if (target_velocity_abs > v_limit && dead_band < error_abs && velocity_threshold < target_velocity_abs)
//     {
//         // Decelerates the robot when the distance between the current location and the target value is smaller than the braking distance.
//         target_velocity_abs = v_limit;
//     }
//     return getSign(target_velocity) * target_velocity_abs;
// }

// float velocityConstraint(float max_accel, float dt, float max_velocity, float target_velocity, float current_velocity)
// {
//     // 加速度制約
//     float result = target_velocity;
//     float target_accel = target_velocity - current_velocity;
//     float max_dv = max_accel * dt;
//     if (fabs(target_accel) > max_dv)
//     {
//         result = current_velocity + getSign(target_accel) * max_dv;
//     }

//     // 最大速度制約
//     if (fabs(result) > max_velocity)
//     {
//         result = max_velocity * getSign(result);
//     }
//     return result;
// }