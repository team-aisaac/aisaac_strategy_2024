#pragma once
#include "System/move.h"
/// @brief ボールを取りに行く
/// @param target_theta ボールを取る際の機体の角度
/// @param world 現在位置
/// @param robot_controller
/// @param output_velocity 速度
void goBallGetPose(float target_theta, const World *world, RobotController *robot_controller, State2D *output_velocity)
{

    //// ロボット中心からドリブラー中心のベクトルを計算
    float robot_ball_con_distance = ROBOT_KICK_MIN_X;
    State2D robot2dribbler_vector = {0};
    robot2dribbler_vector.x = cosf(target_theta) * robot_ball_con_distance;
    robot2dribbler_vector.y = sinf(target_theta) * robot_ball_con_distance;

    // ボールを受け取る目標pose = ball_pose - robot2dribbler_vector
    State2D target_robot_pose = {0};
    diffState2D(&(world->ball_pose), &robot2dribbler_vector, &target_robot_pose);
    target_robot_pose.theta = target_theta;
    goTargetPose(&target_robot_pose, &(world->robot_pose), &(world->robot_velocity), robot_controller, output_velocity);
}

void dribble(const World *world, const StrategyPcCommand *strategy_pc_command,
                       RobotController *robot_controller, RobotOutput *output)
{
    output->actuator_type = ACTION_DRIBLE;
    output->actuator_value = DRIBBLE_POWER;
    if (world->is_ball_detecting)
    {
        //// ボールを持っているときは、ドリブルで進む
        robot_controller->max_robot_velocity = DRIBBLE_VELOCITY;
        goTargetPose(&(strategy_pc_command->ball_goal_pose), &(world->robot_pose), &(world->robot_velocity), robot_controller, &(output->velocity));
    }
    else
    {
        //// まずボールを取りに行く
        //// どの角度でボールを取るか
        //// [TODO] フィールドの隅にいるときは後ろ向きで取る
        State2D ball_goal_vector = {0};
        diffState2D(&(strategy_pc_command->ball_goal_pose), &(world->ball_pose), &ball_goal_vector);
        float target_theta = atan2(ball_goal_vector.y, ball_goal_vector.x);
        goBallGetPose(target_theta, world, robot_controller, &(output->velocity));
    }
}
