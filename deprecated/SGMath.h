/////////////////////////////////////////////////////////////////////////////////////////////
//Include that header to use SGMath, The file CONTAINS CONFIGURATION properties
//Description:
//The SGVector class represents the primitve data structure of the math library
//Vector3 Vector4 Quaretnon are all represent by that SGVector.
//Functions prefixed (vec3 vec4 quat) will define how the SGVector is treated.
//The library supports both RowMajor and ColumnMajor default transform representations.
//Keep in mind that RowMajor is main implementation.
/////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////
//Configurations

//uncomment to enable SSE impementation
#define SGE_MATH_USE_SSE 

//[TODO]uncomment to enable SSE aligned Allocators
//#define SGE_MATH_USE_ALIGNED_ALLOC

//uncomment to enable default + / * x= ect operators
//#define SGE_MATH_VECTOR_OPERATORS 

/////////////////////////////////////////////////////////////////////////////////////////////

//Includes
#include <SGECore.h>

namespace Math
{
	SGE_EXTERN_CONST float PI = 3.14159265359f;
	SGE_EXTERN_CONST float TWO_PI = PI * 2.f;
	SGE_EXTERN_CONST float HALF_PI = PI * 0.5f;

	SGE_FORCE_INLINE float ToRadian(const float degree) { return degree * 0.01745329251994f; }
	SGE_FORCE_INLINE float ToDegree(const float radian) { return radian * 57.2957795130785f; }

	template<typename T>
	T sqr(const T& a) { return a * a; }

	template<typename T>
	T lerp(const T&a, const T&b, const float t) { return a + t*(b-a); }

};

#include "SGVector.h"
#include "SGMatrix34.h"
#include "SGMatrix4.h"




