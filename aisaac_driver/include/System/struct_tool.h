#pragma once
#include "System/struct.h"
#include <math.h>
/// @brief
/// @param a
/// @param b
/// @param output a+b
void addState2D(const State2D *a, const State2D *b, State2D *output)
{
    output->x = a->x + b->x;
    output->y = a->y + b->y;
    output->theta = a->theta + b->theta;
}
/// @brief
/// @param a
/// @param b
/// @param output a-b
void diffState2D(const State2D *a, const State2D *b, State2D *output)
{
    output->x = a->x - b->x;
    output->y = a->y - b->y;
    output->theta = a->theta - b->theta;
}
/// @brief
/// @param a
/// @return sqrt(x*x+y*y)
float xyNorm(const State2D *a)
{
    return sqrt(a->x * a->x + a->y * a->y)
}
/// @brief
/// @param a
/// @param b
/// @param output a.x*b, a.y*b
void xyMultiple(const State2D *a, float b, State2D *output)
{
    output->x = a->x * b;
    output->y = a->y * b;
}
