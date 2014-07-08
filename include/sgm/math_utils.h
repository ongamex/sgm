#pragma once

#ifndef SGM_MATH_UTILS_H_07072014
#define SGM_MATH_UTILS_H_07072014

#include <cmath>

#include "sgm_base.h"

SGE_BEGIN_MATH_NAMESPACE

//----------------------------------------------------------
//min/max
//----------------------------------------------------------
template <class T>
T pick_min(const T& a, const T& b)
{
	return a<b ? a : b;
}

template <class T>
T pick_max(const T& a, const T& b)
{
	return a>b ? a : b;
}

//----------------------------------------------------------
//clamp and saturate
//----------------------------------------------------------
template <class T>
T clamp(const T& x, const T& min, const T& max)
{
	if(x < min) return min;
	if(x > max)	return max;
	return x;
}

float saturate(const float x)
{
	return clamp(x, 0.f, 1.f);
}

double saturate(const double x)
{
	return clamp(x, 0.0, 1.0);
}

//----------------------------------------------------------
//lerp
//----------------------------------------------------------
float lerp(const float a, const float b, const float t)
{
	return a + (b-a)*t;
}

double lerp(const double a, const double b, const double t)
{
	return a + (b-a)*t;
}

//----------------------------------------------------------
//abs
//----------------------------------------------------------
float abs(const float f)
{
	return ::fabsf(f);
}

double abs(const double f)
{
	return ::abs(f);
}

int abs(const int f)
{
	return f < 0 ? -f : f;
}

short abs(const short f)
{
	return f < 0 ? -f : f;
}

long abs(const long f)
{
	return f < 0L ? -f : f;
}

long long abs(const long long f)
{
	return f < 0LL ? -f : f;
}

signed char abs(const signed char f)
{
	return f < 0 ? -f : f;
}

//----------------------------------------------------------
//sqr
//----------------------------------------------------------
template <class T>
T sqr(const T& f)
{
	return f*f;
}

//----------------------------------------------------------
//sqrt
//----------------------------------------------------------
float sqrt(const float f)
{
	return sqrtf(f);
}

double sqrt(const double f)
{
	return ::sqrt(f);
}

//----------------------------------------------------------
//sign
//----------------------------------------------------------
template <class T>
T sign(const T& f)
{
	return (f < (T)(0)) ? -(T)(1) : T(1);
}

SGE_END_MATH_NAMESPACE

#endif