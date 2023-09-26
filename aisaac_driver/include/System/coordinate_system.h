#pragma once
#include "System/struct.h"
#include "System/define.h"
#include <math.h>

/// @brief world座標系での各objectの位置更新
/// @param msg SSL visionの情報を主に利用する
/// @param inout_world input:前回の座標, output: 現在の座標
void updateWorld(Raspberry2MainboardMsg *msg, World *inout_world)
{
    // simとrealで異なるため、後回し
    if (!msg->updated_vision_data)
    {
        // 値の更新がなければearly return
        return
    }
    if (msg->vision_data.robot.pose.reliability > 0.5)
    {
        inout_world->robot_pose = msg->vision_data.robot.pose;
    }
    if (msg->vision_data.ball.pose.reliability > 0.5)
    {
        inout_world->ball_pose = msg->vision_data.ball.pose;
    }

#ifdef REAL
    /*オドメトリの更新*/
    odom_update();
    /*オドメトリの更新終了*/

    /*ボールの位置情報の更新*/
    // get ball sensor data
    ball_position_update();
    /*ボールの位置情報の更新終了*/

    // カメラとオドメトリの合成
    weighted_position_calculate(&coordinate_system.wp, &odom_adjuster, wheel_odom, camera_odom);
    robot_state_update(&coordinate_system.wv, coordinate_system.wp);
#endif
};

/// @brief 座標変換
/// @param world 現在位置. ロボットの角度を利用する
/// @param wov ワールド座標系での速度ベクトル
/// @param rov ロボット座標系での速度ベクトル
void vectorChangeWorldRobot(const World *world, const State2D *wov, State2D *rov)
{
    // Convert the objective vector of the world coordinate system to the robot coordinate system
    float world_rad = world->robot_pose.theta * DEG2RAD;
    float cos_val = cosf(world_rad);
    float sin_val = sinf(world_rad);
    rov->x = wov->x * cos_val + wov->y * sin_val;
    rov->y = -wov->x * sin_val + wov->y * cos_val;
    rov->theta = wov->theta; // 角速度はそのまま
}

void world_ob_vector_calculate(const Raspberry2MainboardMsg *msg, const World *world, RobotOutput *output)
{
    // Calculate the target vector in the world coordinate system

    wovPID_x.error = msg->strategy_pc_command.goal_pose.x - world->robot_pose.x;
    world->robot_pose.x - wp.x;
    wovPID_y.error = wop.y - wp.y;
    float error = norm(0.0f, 0.0f, wovPID_x.error, wovPID_y.error);
    float wvx = pidExecute(&(wovPID_x));
    float wvy = pidExecute(&(wovPID_y));
    float v = norm(0.0f, 0.0f, wvx, wvy);
    // 現在の速度と目標速度が乖離していないか確認
    float braking_distance = v * v / (2 * ROBOT_MAX_ACCELE);
    if (error < braking_distance && POSITION_CONTROL_DEAD_BAND < error && VEROSITY_CONTROL_THRESHOLD < v)
    {
        // Decelerates the robot when the distance between the current location and the target value is smaller than the braking distance.
        float v_limit = sqrt(2 * error * ROBOT_MAX_ACCELE);
        // ロボットの指令速度が最大速度を超えていないか
        wvx = v_limit * wvx / v;
        wvy = v_limit * wvy / v;
    }
    // robot velocity check
    v = norm(0.0f, 0.0f, wvx, wvy);
    if (v > max_robot_vel)
    {
        // the command speed of the robot is exceeding the maximum speed
        wvx = max_robot_vel * wvx / v;
        wvy = max_robot_vel * wvy / v;
    }
    // robot accele check
    float accel_x = (wvx - wov->pre_vx) * CONTROL_FREQUENCY;
    float accel_y = (wvy - wov->pre_vy) * CONTROL_FREQUENCY;
    float accel = norm(0.0f, 0.0f, accel_x, accel_y);
    if (accel > ROBOT_MAX_ACCELE)
    {
        // 加速度がロボットが出せる最大加速度より大きい時
        wvx = wov->pre_vx + accel_x * ROBOT_MAX_ACCELE / (accel * CONTROL_FREQUENCY);
        wvy = wov->pre_vy + accel_y * ROBOT_MAX_ACCELE / (accel * CONTROL_FREQUENCY);
    }
    wov->vx = wvx;
    wov->vy = wvy;
    wov->pre_vx = wvx;
    wov->pre_vy = wvy;

    // ロボットの角速度と角加速度
    wovPID_theta.error = angle_range_corrector((theta_trap_control(wov, wop.theta) - wp.theta) * DEG2RAD); // rad
    wov->omega = pidExecute(&(wovPID_theta));                                                              // rad
    wov->pre_omega = wov->omega;
}
