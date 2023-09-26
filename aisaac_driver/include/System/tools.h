#ifndef _TOOLS_H_
#define _TOOLS_H_

/// @brief 符号を返す
/// @param val 値
/// @return 1 or -1
int getSign(float val)
{
    return (val > 0) - (val < 0);
}

// 最大値を算出
float max_value(float data[], uint16_t size);
// 最小値を算出
float min_value(float data[], uint16_t size);
// 正規化
void min_max_normalize(float *data, uint16_t size);
// 角度範囲内に再計算
float angle_range_corrector(float angle);
// 角度範囲内に再計算(deg)
float angle_range_corrector_deg(float angle);
// 角度範囲内に再計算(-180-180deg)
float angle_range_corrector_deg2(float angle);
// 角度が範囲内かを判定する
bool angele_check(float check_angle, float angle_max, float angle_min);
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.acosfの引数などに使用)
float modifid_acosf(float value);
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.asinfの引数などに使用)
float modifid_asinf(float value);
// ワールド座標系をロボット座標系に変更する関数
void w_to_r_coordinate_chang(float w_x, float w_y, float *r_x, float *r_y, float w_robot_x, float w_robot_y, float w_robot_theta);
// ロボット座標系をワールド座標系に変換する
void r_to_w_coordinate_chang(float *w_x, float *w_y, float r_x, float r_y, float w_robot_x, float w_robot_y, float w_robot_theta);
//(x1,y1)と(x2,y2)を通る直線上の店の中で(x3,y3)に最も近い点を求める
bool nearest_point_to_straight_line(float x1, float y1, float x2, float y2, float x3, float y3, float *x, float *y);
//(x1,y1)と(x2,y2)のノルムを計算する関数
float norm(float x1, float y1, float x2, float y2);
// 絶対値を返す(absの引数が0の時エラーになることがあるためエラーを発生させない関数)
float modifid_abs(float x);

#endif // _TOOLS_H_
