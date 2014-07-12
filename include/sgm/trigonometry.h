#pragma once

#ifndef SGM_TRIGONOMETRY_H_07072014
#define SGM_TRIGONOMETRY_H_07072014

#include "sgm_base.h"
#include <cmath>

SGE_BEGIN_MATH_NAMESPACE

//----------------------------------------------------------
//common constants
//----------------------------------------------------------
template <typename T>
T pi(void) { return (T)(3.141592653589793); }

template <typename T>
T half_pi(void)
{ return pi<T>() * (T)(0.5); }

template <typename T>
T two_pi(void)
{ return pi<T>() * (T)(2.0); }

//template <typename T>
//T inv_pi(void)
//{ return (T)(1.0) / pi<T>(); }
//
//template <typename T>
//T inv_two_pi(void)
//{ return (T)(1.0) / TwoPi<T>(); }

//some shortcut macros for pi, usage example SGM_PI(float)
#define SGM_PI(T)				sgm::pi< T >()
#define SGM_HALF_PI(T)			sgm::half_pi< T >()
#define SGM_2PI(T)				sgm::two_pi< T >()

//-----------------------------------------------------------------
//angle convert
//-----------------------------------------------------------------
template<class T>
T radians(const T& degree)
{
	return degree * (T)(0.01745329251994);
}

template<class T>
T degrees(const T& radian)
{
	return radian * (T)(57.2957795130785);
}

//-----------------------------------------------------------------
//sin
//-----------------------------------------------------------------
float sin(const float a)
{
	return std::sin(a);
}

double sin(const double a)
{
	return std::sin(a);
}

//-----------------------------------------------------------------
//arcsin
//-----------------------------------------------------------------
float asin(const float a)
{
	return std::asin(a);
}

double asin(const double a)
{
	return std::asin(a);
}

//-----------------------------------------------------------------
//cos
//-----------------------------------------------------------------
float cos(const float a)
{
	return std::cos(a);
}

double cos(const double a)
{
	return std::cos(a);
}

//-----------------------------------------------------------------
//arccos
//-----------------------------------------------------------------
float acos(const float a)
{
	return std::acos(a);
}

double acos(const double a)
{
	return std::acos(a);
}

//-----------------------------------------------------------------
//sincos
//-----------------------------------------------------------------
void sincos(const float& angle, float& rsin, float& rcos)
{
	rsin = ::sin(angle);
	rcos = ::cos(angle);
}

void sincos(const double& angle, double& rsin, double& rcos)
{
	rsin = ::sin(angle);
	rcos = ::cos(angle);
}

//-----------------------------------------------------------------
//tangent
//-----------------------------------------------------------------
float tan(const float a)
{
	return ::tanf(a);
}

double tan(const double a)
{
	return ::tan(a);
}

//-----------------------------------------------------------------
//arctangent
//-----------------------------------------------------------------
float atan(const float x)
{
	return ::atanf(x);
}

float atan2(const float y, const float x)
{
	return ::atan2f(y, x);
}

double atan(const double x)
{
	return ::atan(x);
}

double atan2(const double y, const double x)
{
	return ::atan2(y, x);
}

SGE_END_MATH_NAMESPACE

#endif