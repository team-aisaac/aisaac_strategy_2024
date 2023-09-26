#include "System/struct.h"
#include "System/aisaac_uart_msg.h" //readmsg()
#include "System/temp.h"
#include "System/coordinate_system.h" // odomUpdate()
#include "System/game_processing.h"   // calcOutput()
//// 1ループ前の値が必要なので、globalで保存
Raspberry2MainboardMsg msg = {0};
World world = {0};
State2D output_velocity_world = {0};

void timerTask()
{
    // データ受信
    readmsg(&msg);

    // 自己位置推定 カルマンフィルタとか
    updateWorld(&msg, &world);

    // 目標出力計算
    calcOutput(&msg, &world, &output_velocity_world);

    // world座標系 -> robot座標系に変換
    State2D output_velocity_robot = {0};
    vectorChangeWorldRobot(&output_velocity_world, &output_velocity_robot);

    // 出力
    omniDrive(&output_velocity_robot);
    dribblerDrive();
}
