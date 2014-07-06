#pragma once

#include <SGECore.h>
#include "Abstract/AlignedNew.h"

#include <math.h>

#if defined(SGE_MATH_USE_SSE)
	#include <xmmintrin.h>
#endif 

//SIMD Macrosd
#if defined(SGE_MATH_USE_SSE)
	#define SGE_SIMD_SHUFFLE(x, y, z, w) ((w)<<6 | (z)<<4 | (y)<<2 | (x))
#endif

//default SIMD type

#if defined(SGE_MATH_USE_SSE)
	typedef __m128 SGE_M128;
#endif

//----------------------------------------------------------------------------------------------------
#if defined(SGE_MATH_USE_SSE)
	extern const __m128 SGE_SSE_MASK_FFF0;
	extern const __m128 SGE_SSE_MASK_QUAT_CONJURGATE;
#endif

#if defined(SGE_MATH_USE_SSE)
	//--------------------------------------------------------------------------
	#define SGE_SIMD_Splat(_v, _i)		(_mm_shuffle_ps((_v), (_v), SGE_SIMD_SHUFFLE( (_i), (_i), (_i), (_i))))

	SGE_FORCE_INLINE float SGE_SIMD_ExtractComponent(const __m128& v, const int i)
	{
	#if defined(_MSC_VER)
		return v.m128_f32[i];
	#else
		v[i];
	#endif
	}

	//--------------------------------------------------------------------------
	//s? is 1 then the ? component sign will be changed
	template<const int sx, const int sy, const int sz, const int sw>
	SGE_FORCE_INLINE __m128 SGE_SIMD_ChangeSign(const __m128& v)
	{
		const __m128 MASK = _mm_castsi128_ps(_mm_setr_epi32(
		sx ? 0x80000000 : 0 , 
		sy ? 0x80000000 : 0 ,
		sz ? 0x80000000 : 0 , 
		sw ? 0x80000000 : 0));
		return _mm_xor_ps(v, MASK);
	}

	//--------------------------------------------------------------------------
	SGE_FORCE_INLINE float SGE_SIMD_sum(const __m128& a)
	{
		__m128 t = _mm_hadd_ps(a, a);
		return SGE_SIMD_ExtractComponent(_mm_hadd_ps(t, t), 0);
	}

#endif

struct SGVector;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will threat SGVector as a 3D vector
//[NOTE] Those functions may change the _w component of the vector
//Designed to be inlined in SGVector.h
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGVector			vec3_add(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			vec3_sub(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			vec3_scale(const SGVector& a, const float s);
SGE_FORCE_INLINE SGVector			vec3_inverse(const SGVector& a);
SGE_FORCE_INLINE SGVector			vec3_mul(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE float			 	vec3_dot(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			vec3_dot3(const SGVector& V, const SGVector& a, const SGVector& b, const SGVector& c);
SGE_FORCE_INLINE float				vec3_lenghtSqr(const SGVector& a);
SGE_FORCE_INLINE float			 	vec3_lenght(const SGVector& a);
SGE_FORCE_INLINE SGVector			vec3_cross(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE float			 	vec3_triple(const SGVector& a, const SGVector& b, const SGVector& c);
SGE_FORCE_INLINE SGVector			vec3_normalized(const SGVector& a);
SGE_FORCE_INLINE void			 	vec3_normalize(SGVector& a);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will threat SGVector as a 4D vector
//[NOTE] Those functions may change the _w component of the vector
//Designed to be inlined in SGVector.h
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGVector			vec4_add(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			vec4_sub(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			vec4_scale(const SGVector& a, const float s);
SGE_FORCE_INLINE SGVector			vec4_inverse(const SGVector& a);
SGE_FORCE_INLINE SGVector			vec4_mul(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE float				vec4_dot(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE float				vec4_lenghtSqr(const SGVector& a);
SGE_FORCE_INLINE float				vec4_lenght(const SGVector& a);
SGE_FORCE_INLINE SGVector			vec4_normalized(const SGVector& a);
SGE_FORCE_INLINE void				vec4_normalize(SGVector& a);
SGE_FORCE_INLINE SGVector			vec4_negative(const SGVector& a);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will threat SGVector as a quaterion
//[NOTE]The quaterion layout is defined by:
//(x, y, z) representing the imaginary part of the quat and (w) the real part
//Designed to be inlined in SGVector.h
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGVector			quat_from_axisRotation(const SGVector& axis, const float theta);
SGE_FORCE_INLINE SGVector			quat_mul(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			quat_scale(const SGVector& a, const float s);
SGE_FORCE_INLINE SGVector			quat_add(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			quat_sub(const SGVector& a, const SGVector& b);
SGE_FORCE_INLINE SGVector			quat_conjurgate(const SGVector& a);
SGE_FORCE_INLINE float				quat_normSqr(const SGVector& a);
SGE_FORCE_INLINE float				quat_norm(const SGVector& a);
SGE_FORCE_INLINE SGVector			quat_inverse(const SGVector& a);
SGE_FORCE_INLINE SGVector			quat_normalized(const SGVector& a);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//struct VectorBase
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct SGVector
#if defined( SGE_MATH_USE_SSE ) && 0 //inherit the aligned new operator if using SIMD
	 :public TAlignedNew<SGVector>
#endif
{
	//SSE specifics
#if defined(SGE_MATH_USE_SSE)
	public :
	 union 
	 {
		 SGE_M128	m_M128;
		 float		m_floats[4];
		 struct		{ float x, y, z, w; };
	 };

#else
	 //when no SIMD
	public :
	 union 
	 {
		 float		m_floats[4];
		 struct		{ float x, y, z, w; };
	 }; 
#endif

	 //------------------------------------------------------------------
	 //Constructors
	 //------------------------------------------------------------------
	 SGVector()  {}

	 SGVector(const float _x, const float _y, const float _z, const float _w) :
		 x(_x), y(_y), z(_z), w(_w)
	 {}

	  SGVector(const float _s) :
		 x(_s), y(_s), z(_s), w(_s)
	 {}

	  //------------------------------------------------------------------
	  //BEGIN Copy constructor
#if defined(SGE_MATH_USE_SSE)
	  SGVector(const SGVector& other)
	  {
		m_M128 = other.m_M128;
	  }

	  //------------------------------------------------------------------
	  SGVector& operator=(const SGVector& other)
	  {
		m_M128 = other.m_M128;
		return (*this);
	  }
#else
	  SGVector(const SGVector& other)
	  {
		x = other.x;
		y = other.y;
		z = other.z;
		w = other.w;
	  }

	  //------------------------------------------------------------------
	  SGVector& operator=(const SGVector& other)
	  {
		x = other.x;
		y = other.y;
		z = other.z;
		w = other.w;
		return (*this);
	  }
#endif
	  //END Copy constructor
	  //------------------------------------------------------------------

#if defined(SGE_MATH_USE_SSE)
	 explicit SGVector(const SGE_M128& m128) :
		 m_M128(m128)
	 {}
#endif
	
	void setValue(const float _x, const float _y, const float _z, const float _w)
	{
#if defined(SGE_MATH_USE_SSE)
		m_M128 = _mm_setr_ps(_x, _y, _z, _w);
#else
		x = _x;
		y = _y;
		z = _z;
		w = _w;
#endif
	}

	float at(const int i) const
	{
		return m_floats[i];
	}

	float& at(const int i)
	{
		return m_floats[i];
	}

	//------------------------------------------------------------------
	template<const int A, const int B, const int C, const int D>
	SGVector Permute() const
	{
#if defined(SGE_MATH_USE_SSE)
		return SGVector(_mm_shuffle_ps(m_M128, m_M128, SGE_SIMD_SHUFFLE(A, B, C, D)));
#else
		return SGVector(m_floats[A], m_floats[B], m_floats[C], m_floats[D]);
#endif
	}

	//------------------------------------------------------------------
	//changes the signs in the vector
	//s? is 1 then the ? component sign will be changed
	template<const int sx, const int sy, const int sz, const int sw>
	SGVector ChangeSign() const
	{
		return vec_changeSign<sx, sy, sz, sw>(*this);
	}

	//------------------------------------------------------------------

public :

	//Some commonly used vectors
	static const SGVector V0000;
	static const SGVector V1111;
	static const SGVector V1000;
	static const SGVector V0100;
	static const SGVector V0010;
	static const SGVector V0001;

public :

	//------------------------------------------------------------------
	//Methods that will threat the instance as a Vector3
	SGE_FORCE_INLINE SGVector		Normalized3D() const					{ return vec3_normalized(*this); }

	SGE_FORCE_INLINE float			LengthV3() const						{ return vec3_lenght(*this); }
	SGE_FORCE_INLINE float			LengthSqrV3() const						{ return vec3_lenghtSqr(*this); }

	SGE_FORCE_INLINE float			DotV3(const SGVector& other) const		{ return vec3_dot(*this, other); }
	SGE_FORCE_INLINE SGVector		CrossV3(const SGVector& other) const	{ return vec3_cross(*this, other); }
	
	SGE_FORCE_INLINE SGVector		Dot3V3(const SGVector& a, const SGVector& b, const SGVector& c) const		
	{ return vec3_dot3(*this, a, b, c); }

	//------------------------------------------------------------------
	//Methods that will threat the instance as a Vector4
	SGE_FORCE_INLINE SGVector		NormalizedV4() const					{ return vec4_normalized(*this); }
	SGE_FORCE_INLINE float			LengthV4() const						{ return vec4_lenght(*this); }
	SGE_FORCE_INLINE float			LengthSqrV4() const						{ return vec4_lenghtSqr(*this); }
	SGE_FORCE_INLINE float			DotV4(const SGVector& other) const		{ return vec4_dot(*this, other); }

	//------------------------------------------------------------------
	//Methods that will threat the instance as a quaternion

	SGE_FORCE_INLINE SGVector		ConjurgateQ() const						{ return quat_conjurgate(*this); }
	SGE_FORCE_INLINE SGVector		InverseQ() const						{ return quat_inverse(*this); }
	SGE_FORCE_INLINE SGVector		NormalizedQ() const						{ return quat_normalized(*this); }

	//---------------------------------------------------------------------------------------------
	//[NOTE]Operators will threat the vector as a 4D vector
	//[CAUTION]Only implementations that are suiteable Quaternions, Vector3 and Vector4 at the same time
	//must be implemented!!!!!

	SGVector operator+() const
	{
		return *this;
	}

	SGVector operator-() const
	{
		return ChangeSign<1,1,1,1>();
	}

	SGVector operator+(const SGVector& v)
	{
		return vec4_add(*this, v);
	}

	SGVector operator-(const SGVector& v)
	{
		return vec4_sub(*this, v);
	}

	SGVector operator*(const float s)
	{
		return vec4_scale(*this, s);
	}

	SGVector& operator+=(const SGVector& v)
	{
		*this = vec4_add(*this, v);
		return *this;
	}

	SGVector operator-=(const SGVector& v)
	{
		*this = vec4_sub(*this, v);
		return *this;
	}

	SGVector operator*=(const float s)
	{
		*this = vec4_scale(*this, s);
		return *this;
	}

};


//------------------------------------------------------------------
//changes the signs in the vector
//s? is 1 then the ? component sign will be changed
template<const int sx, const int sy, const int sz, const int sw>
SGVector vec_changeSign(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	return SGVector(SGE_SIMD_ChangeSign<sx, sy, sz, sw>(a.m_M128));
#else
	return SGVector(
		(sx ? 1.f : -1.f) * a.m_floats[0], 
		(sy ? 1.f : -1.f) * a.m_floats[1],
		(sz ? 1.f : -1.f) * a.m_floats[2], 
		(sw ? 1.f : -1.f) * a.m_floats[3]
	);
#endif
}


SGE_FORCE_INLINE float vec_sum(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 t = _mm_hadd_ps(a.m_M128, a.m_M128);
	return SGE_SIMD_ExtractComponent(_mm_hadd_ps(t, t), 0);
#else
	return a.x + a.y + a.z + a.w;
#endif
}

#include "SGVector_vec3.inl"
#include "SGVector_vec4.inl"
#include "SGVector_quat.inl"