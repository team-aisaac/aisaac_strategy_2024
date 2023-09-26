#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<math.h>
#include "System/tools.h"
//#include "robot_controller/dwa_include/dwa.h"

//基本関数
//最大値を算出
float max_value(float data[], uint16_t size){
    float max = data[0];
    for(int i = 1; i < size; i++){
        if(max < data[i]){
            max = data[i];
        }
    }
    return max;
}
//最小値を算出
float min_value(float data[], uint16_t size){
    float min = data[0];
    for(int i = 1; i < size; i++){
        if(data[i] < min){
            min = data[i];
        }
    }
    return min;
}
//正規化
void min_max_normalize(float *data, uint16_t size){
    float max = max_value(data, size);
    float min = min_value(data, size);
    if(max - min == 0){
        for(int i = 0; i<size; i++){
            data[i] = 0;
        }
    }
    else{
        for(int i = 0; i<size; i++){
            data[i] = (data[i]-min)/(max - min);
        }
    }
}
//角度範囲内に再計算(-pi-pi rad)
float angle_range_corrector(float angle){
    if(angle > M_PI){
        while(angle > M_PI){
            angle -=  2 * M_PI;
        }
    }
    else if(angle < -M_PI){
        while(angle < -M_PI){
            angle += 2 * M_PI;
        }
    }
    return angle;
}
//角度範囲内に再計算(0-360deg)
float angle_range_corrector_deg(float angle){
    if(angle < 0){
        while(angle < 0){
            angle = angle + 360;
        }
    }
    else if(360 < angle){
        while(360 < angle){
            angle = angle - 360;
        }
    }
    return angle;
}
//角度範囲内に再計算(-180-180deg)
float angle_range_corrector_deg2(float angle){
    if(angle <= -180){
        while(angle < -180){
            angle = angle + 360;
        }
    }
    else if(180 < angle){
        while(180 < angle){
            angle = angle - 360;
        }
    }
    return angle;
}
//角度が範囲内かを判定する(範囲内:1, 範囲外:0, 単位:rad)
bool angele_check(float check_angle, float angle_max, float angle_min){
    check_angle = angle_range_corrector(check_angle);
    angle_min = angle_range_corrector(angle_min);
    angle_max = angle_range_corrector(angle_max);
    if(angle_min <= angle_max){
        if(angle_min <= check_angle && check_angle <= angle_max){
            return 1;
        }
    }
    else{
        if(angle_max < 0){
            angle_max += 2 * M_PI;
        }
        if(angle_min < 0){
            angle_min += 2 * M_PI;
        }
        if(check_angle < 0){
            check_angle += 2 * M_PI;
        }
        if(angle_min <= check_angle && check_angle <= angle_max){
            return 1;
        }
    }
    return 0;
}
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.acosfの引数などに使用)
float modifid_acosf(float value){
    if(1.0 < value){
        return acosf(1.0);
    }
    if(value < -1.0){
        return acosf(-1.0);
    }
    return acosf(value);
}
//-1~1で数字をラップする(float等で計算した場合わずかに超えたりする.asinfの引数などに使用)
float modifid_asinf(float value){
        if(1.0 < value){
        return asinf(1.0);
    }
    if(value < -1.0){
        return asinf(-1.0);
    }
    return asinf(value);
}
//ワールド座標系をロボット座標系に変更する関数
void w_to_r_coordinate_chang(float w_x, float w_y, float *r_x, float *r_y, float w_robot_x, float w_robot_y, float w_robot_theta){
    *r_x = (w_x - w_robot_x)*cosf(-w_robot_theta) - (w_y - w_robot_y)*sinf(-w_robot_theta);
    *r_y = (w_x - w_robot_x)*sinf(-w_robot_theta) + (w_y - w_robot_y)*cosf(-w_robot_theta);
}
//ロボット座標系をワールド座標系に変更する関数
void r_to_w_coordinate_chang(float *w_x, float *w_y, float r_x, float r_y, float w_robot_x, float w_robot_y, float w_robot_theta){
    *w_x = r_x*cosf(w_robot_theta) - r_y*sinf(w_robot_theta) + w_robot_x;
    *w_y = r_x*sinf(w_robot_theta) + r_y*cosf(w_robot_theta) + w_robot_y;
}
//(x1,y1)と(x2,y2)を通る直線上の店の中で(x3,y3)に最も近い点(x,y)を求める
bool nearest_point_to_straight_line(float x1, float y1, float x2, float y2 ,float x3, float y3, float *x, float *y){
    float v_x = x2 - x1;
    float v_y = y2 - y1;
    float square_norm = v_x*v_x + v_y*v_y;
    if(square_norm <= 0){  //(x1,y1)と(x2,y2)が近すぎる、あるいは一致しているときは計算しない
        return false;
    }
    float t = (v_x*(x3-x1) + v_y*(y3-y1))/square_norm;
    *x = x1 + t*v_x;
    *y = y1 + t*v_y;
    return true;
}
//(x1,y1)と(x2,y2)のノルムを計算する関数
float norm(float x1, float y1, float x2, float y2){
    float square_norm = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
    if (square_norm <= 0)
    {
        return 0.0f;
    }
    else{
        return sqrt(square_norm);
    }
    
}

float modifid_abs(float x){
	if(x == 0.0f){
		return 0.0f;
	}
	else{
		float abs_x = abs(x);
		return abs_x;
	}
}
