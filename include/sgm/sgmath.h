#pragma once

#include <assert.h>
#include <cmath>
#include <float.h>
#include <memory>

#define SGE_BEGIN_MATH_NAMESPACE namespace sgm {
#define SGE_END_MATH_NAMESPACE }

#define SGE_MATH_FORCE_INLINE __forceinline
#define SGE_MATH_STATIC_ASSERT(condition, strLiteralMessage) static_assert(condition, strLiteralMessage)

#if defined _MSC_VER
	#define SGM_ATTRIBUTE_ALIGNED16						__declspec(align(16)) 
#else
	#define SGM_ATTRIBUTE_ALIGNED16						__attribute__(align(16))
#endif

SGE_BEGIN_MATH_NAMESPACE

//----------------------------------------------------------
//Common Pi constants
//----------------------------------------------------------
template <typename T>
T pi(void)
{
	return (T)(3.141592653589793);
}

template <typename T>
T half_pi(void)
{
	return pi<T>() * (T)(0.5);
}

template <typename T>
T two_pi(void)
{
	return pi<T>() * (T)(2.0);
}

template <typename T>
T inv_pi(void)
{
	return (T)(1.0) / pi<T>();
}

template <typename T>
T inv_2pi(void)
{
	return (T)(1.0) / TwoPi<T>();
}

template<class T>
T to_radian(const T& degree)
{
	return degree * (T)(0.01745329251994);
}

template<class T>
T to_degree(const T& radian)
{
	return radian * (T)(57.2957795130785);
}

//----------------------------------------------------------
//Basic math operations
//----------------------------------------------------------
template <class T>
T abs(T a)					{ return ::fabsf((float)a); }

template <>
double abs(double a)		{ return ::abs(a); }

//
template <class T>
T sign(T a)					{ return (a >= (T)0.f) ? (T)1.f : (T)-1.f; }

//
template <class T>
T sqrt(T a)					{ return ::sqrtf((float)a); }

template <>
double sqrt(double a)		{ return ::sqrt(a); }

template <class T>
T inv_sqrt(T a)				{ return 1.f/sqrt(a); }

//
template <class T>
T sin(T a)					{ return ::sinf((float)a); }

template <>
double sin(double a)		{ return ::sin(a); }

//
template <class T>
T cos(T a)					{ return ::cosf((float)a); }

template <>
double cos(double a)		{ return ::cos(a); }

template <class T>
void sincos(T& sin, T& cos, const T& a)
{
	sin = ::sinf(a);
	cos = ::cosf(a);
}

///
///Returns the min(a,b)
///
template <class T>
T pick_min(const T& a, const T& b)
{
	return a<b ? a : b;
}

///
///Returns the max(a,b)
///
template <class T>
T pick_max(const T& a, const T& b)
{
	return a>b ? a : b;
}

SGE_END_MATH_NAMESPACE

