#pragma once
#include "System/struct.h"
#include "System/define.h"
#include "System/temp.h"
#include "System/move.h"
#include "System/dribble.h"
#include "System/kick.h"
/// @brief halt中は出力0
/// @param output
void halt(RobotOutput *output)
{
    RobotOutput zero = {0};
    *output = zero;
}
/// @brief stop中は最大速度減
/// @param strategy_pc_command
/// @param world
/// @param output
void stopGame(const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    robot_controller->max_robot_vel = MAX_ROBOT_SPEED_GAME_STOP;
    // 目標値に向けて走行
    move(strategy_pc_command, world, robot_controller, output);
    goTargetPose(&(strategy_pc_command->robot_goal_pose), &(world->robot_pose), robot_controller, &(output->velocity));
}

void inGame(const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    robot_controller->max_robot_vel = MAX_ROBOT_SPEED;
    switch (strategy_pc_command->ball_action)
    {
    case ACTION_MOVE: // ただ目標値に移動
        move(strategy_pc_command, world, robot_controller, output);
        break;
    case ACTION_DRIBLE:
        dribble(strategy_pc_command, world, robot_controller, output);
        break;
    case ACTION_KICK:
        kick(strategy_pc_command, world, robot_controller, output);
        break;
    default:
        move(strategy_pc_command, world, robot_controller, output);
        break;
    }
}

/// @brief 目標出力計算
/// @param game_state
/// @param strategy_pc_command
/// @param world 座標
/// @param output 目標出力
void calcOutput(const int *game_state, const StrategyPcCommand *strategy_pc_command, const World *world, RobotController *robot_controller, RobotOutput *output)
{
    int game_state_ = *game_state;
    if (checkRobotError())
    {
        game_state_ = true;
    }
    switch (game_state_)
    {
    case GAME_STATE_STOP:
        stopGame(strategy_pc_command, world, robot_controller, output);
        break;
    case GAME_STATE_INGAME:
    case GAME_STATE_OUR_BALL_PLACEMENT:
        inGame(strategy_pc_command, world, robot_controller, output);
        break;
    case GAME_STATE_THEIR_BALL_PLACEMENT:
        stopGame(strategy_pc_command, world, robot_controller, output);
        break;
    case GAME_STATE_HALT:
        halt(output);
        break;
    default:
        halt(output);
        // printf("[ERROR] game state is invalid")
        break;
    }
};
