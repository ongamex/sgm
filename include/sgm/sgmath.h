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

template <typename T>
T Pi(void)
{
	return (T)(3.141592653589793);
}

template <typename T>
T HalfPi(void)
{
	return Pi<T>() * (T)(0.5);
}

template <typename T>
T TwoPi(void)
{
	return Pi<T>() * (T)(2.0);
}

template <typename T>
T InvPi(void)
{
	return (T)(1.0) / Pi<T>();
}

template <typename T>
T InvTwoPi(void)
{
	return (T)(1.0) / TwoPi<T>();
}

template<class T>
T ToRadian(const T& degree)
{
	return degree * (T)(0.01745329251994);
}

template<class T>
T ToDegree(const float radian)
{
	return radian * (T)(57.2957795130785);
}

//[TODO]
template <class T>
T abs(T a)					{ return ::fabsf(a); }

template <class T>
T sign(T a)					{ return (a >= 0.f) ? 1.f : -1.f; }

template <class T>
T sqrt(T a)					{ return ::sqrtf(a); }

template <class T>
T recipSqrt(T a)			{ return 1.f/::sqrtf(a); }

template <class T>
T sin(T a)					{ return ::sinf(a); }

template <class T>
T cos(T a)					{ return ::cosf(a); }

template <class T>
void sincos(T& sin, T& cos, const T& a)
{
	sin = ::sinf(a);
	cos = ::cosf(a);
}

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

SGE_END_MATH_NAMESPACE

