#pragma once
#include "stdbool.h"
#include "System/pid.h"
/// @brief game state
enum
{
    GAME_STATE_HALT = 0,
    GAME_STATE_STOP,
    GAME_STATE_INGAME,
    GAME_STATE_OUR_BALL_PLACEMENT,
    GAME_STATE_THEIR_BALL_PLACEMENT,
};

/// @brief actuator type
enum
{
    ACTION_MOVE = 0,
    ACTION_KICK,
    ACTION_DRIBLE,
};
typedef struct
{
    float x;
    float y;
    float theta;
} State2D; // consai_ros2と構造体を一致させた

typedef struct
{
    State2D pose;
    float reliability;
} VisionPose;

typedef struct
{
    VisionPose robot;
    VisionPose ball;
} VisionData;

typedef struct
{
    State2D robot_goal_pose; // 機体をどこに移動させるか
    State2D ball_goal_pose;  // ボールをどこに運ぶか
    State2D target_velocity; // 速度制御指示
    bool is_velocity_control;    // 速度制御を行う[TODO]
    int ball_action;
} StrategyPcCommand;

typedef struct
{
    State2D velocity;
    int actuator_type;
    float actuator_value;
} RobotOutput;

typedef struct
{
    bool updated_vision_data;
    VisionData vision_data;
    StrategyPcCommand strategy_pc_command;
    int game_state;
} Raspberry2MainboardMsg;

typedef struct
{
    State2D robot_pose;
    State2D robot_velocity;
    State2D ball_pose;
    bool is_ball_detecting;
} World;

typedef struct
{
    _pid_t pid_x;
    _pid_t pid_y;
    _pid_t pid_theta;
    float max_robot_velocity;
} RobotController;
