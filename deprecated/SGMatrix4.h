#pragma once

#include "SGVector.h"
#include "SGMatrix34.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//struct SGMatrix4
//[NOTE] each SGVector represents a single row
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct SGMatrix4 
#if defined (SGE_MATH_USE_ALIGNED_ALLOC)
	: public TAlignedNew<SGMatrix4>
#endif
{
	SGVector m_rows[4];	
	
	//------------------------------------------------------------------
	//Constructors
	//------------------------------------------------------------------
	SGMatrix4() {}

	SGMatrix4(	const float _11, const float _12, const float _13, const float _14,
				const float _21, const float _22, const float _23, const float _24,
				const float _31, const float _32, const float _33, const float _34,
				const float _41, const float _42, const float _43, const float _44)

	{
		m_rows[0] = SGVector(_11, _12, _13, _14);
		m_rows[1] = SGVector(_21, _22, _23, _24);
		m_rows[2] = SGVector(_31, _32, _33, _34);
		m_rows[3] = SGVector(_41, _42, _43, _44);
	}

	SGMatrix4(const SGVector& r0, const SGVector& r1, const SGVector& r2, const SGVector& r3)
	{
		m_rows[0] = (r0);
		m_rows[1] = (r1);
		m_rows[2] = (r2);
		m_rows[3] = (r3);
	}


	explicit SGMatrix4(const float s)
	{
		m_rows[0] = SGVector(s, 0.f, 0.f, 0.f);
		m_rows[1] = SGVector(0.f, s, 0.f, 0.f);
		m_rows[2] = SGVector(0.f, 0.f, s, 0.f);
		m_rows[3] = SGVector(0.f, 0.f, 0.f, s);
	}

	explicit SGMatrix4(const SGMatrix34& m)
	{
		m_rows[0] = m.m_rows[0];
		m_rows[1] = m.m_rows[1];
		m_rows[2] = m.m_rows[2];
		m_rows[3] = SGVector::V0001;
	}

#if defined(SGE_MATH_USE_SSE)
	SGMatrix4(const __m128& r0, const __m128& r1, const __m128& r2, const __m128& r3)
	{
		m_rows[0].m_M128 = (r0);
		m_rows[1].m_M128 = (r1);
		m_rows[2].m_M128 = (r2);
		m_rows[3].m_M128 = (r3);
	}
#endif
	//------------------------------------------------------------------
	SGVector GetColumn(const int i) const
	{
		return SGVector(m_rows[0].m_floats[i], 
			m_rows[1].m_floats[i], 
			m_rows[2].m_floats[i], 
			m_rows[3].m_floats[i]);
	}

	//------------------------------------------------------------------
	SGE_FORCE_INLINE float at(const int r, const int c) const { return m_rows[r].m_floats[c]; }

	//get scalar row0
	SGE_FORCE_INLINE float _00() const { return m_rows[0].x; }
	SGE_FORCE_INLINE float _01() const { return m_rows[0].y; }
	SGE_FORCE_INLINE float _02() const { return m_rows[0].z; }
	SGE_FORCE_INLINE float _03() const { return m_rows[0].w; }
				 
	//get scalar row1			 
	SGE_FORCE_INLINE float _10() const { return m_rows[1].x; }
	SGE_FORCE_INLINE float _11() const { return m_rows[1].y; }
	SGE_FORCE_INLINE float _12() const { return m_rows[1].z; }
	SGE_FORCE_INLINE float _13() const { return m_rows[1].w; }

	//get scalar row2			 
	SGE_FORCE_INLINE float _20() const { return m_rows[2].x; }
	SGE_FORCE_INLINE float _21() const { return m_rows[2].y; }
	SGE_FORCE_INLINE float _22() const { return m_rows[2].z; }
	SGE_FORCE_INLINE float _23() const { return m_rows[2].w; }

	//get scalar row3			 
	SGE_FORCE_INLINE float _30() const { return m_rows[3].x; }
	SGE_FORCE_INLINE float _31() const { return m_rows[3].y; }
	SGE_FORCE_INLINE float _32() const { return m_rows[3].z; }
	SGE_FORCE_INLINE float _33() const { return m_rows[3].w; }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will thread SGMatrix4 as a 4x4 matrix
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGMatrix4 mat4_add(const SGMatrix4& a, const SGMatrix4& b)
{
	return SGMatrix4(	vec4_add( a.m_rows[0], a.m_rows[0]),
						vec4_add( a.m_rows[1], a.m_rows[1]),
						vec4_add( a.m_rows[2], a.m_rows[2]),
						vec4_add( a.m_rows[3], a.m_rows[3]));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_sub(const SGMatrix4& a, const SGMatrix4& b)
{
	return SGMatrix4(	vec4_sub( a.m_rows[0], a.m_rows[0]),
						vec4_sub( a.m_rows[1], a.m_rows[1]),
						vec4_sub( a.m_rows[2], a.m_rows[2]),
						vec4_sub( a.m_rows[3], a.m_rows[3]));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_scale(const SGMatrix4& m, const float s)
{
	return SGMatrix4(vec4_scale(m.m_rows[0], s),
		vec4_scale(m.m_rows[1], s),
		vec4_scale(m.m_rows[2], s),
		vec4_scale(m.m_rows[3], s));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_getTranspose(const SGMatrix4& m)
{
	const SGVector c0 = m.GetColumn(0);
	const SGVector c1 = m.GetColumn(1);
	const SGVector c2 = m.GetColumn(2);
	const SGVector c3 = m.GetColumn(3);

	return SGMatrix4(c0, c1, c2, c3);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float mat4_determinant(const SGMatrix4& m)
{
	//unrolled by the minors defined by the 1st row
#if defined(SGE_MATH_USE_SSE)
	SGVector m0, m1, m2;
	
	//minor 0
	m0.m_M128 = _mm_shuffle_ps( m.m_rows[1].m_M128,  m.m_rows[1].m_M128, SGE_SIMD_SHUFFLE(1, 2 ,3, 0));// (Y Z W) ,X component is unused
	m1.m_M128 = _mm_shuffle_ps( m.m_rows[2].m_M128,  m.m_rows[2].m_M128, SGE_SIMD_SHUFFLE(1, 2 ,3, 0));// (Y Z W) ,X component is unused
	m2.m_M128 = _mm_shuffle_ps( m.m_rows[3].m_M128,  m.m_rows[3].m_M128, SGE_SIMD_SHUFFLE(1, 2 ,3, 0));// (Y Z W) ,X component is unused

	float retval = vec3_triple(m0, m1, m2) *  m.m_rows[0].x;

	//minor 1
	m0.m_M128 = _mm_shuffle_ps( m.m_rows[1].m_M128,  m.m_rows[1].m_M128, SGE_SIMD_SHUFFLE(0, 2 ,3, 0));// (X Z W) ,Y component is unused
	m1.m_M128 = _mm_shuffle_ps( m.m_rows[2].m_M128,  m.m_rows[2].m_M128, SGE_SIMD_SHUFFLE(0, 2 ,3, 0));// (X Z W) ,Y component is unused
	m2.m_M128 = _mm_shuffle_ps( m.m_rows[3].m_M128,  m.m_rows[3].m_M128, SGE_SIMD_SHUFFLE(0, 2 ,3, 0));// (X Z W) ,Y component is unused

	retval -= vec3_triple(m0, m1, m2) *  m.m_rows[0].y;

	//minor 2
	m0.m_M128 = _mm_shuffle_ps( m.m_rows[1].m_M128,  m.m_rows[1].m_M128, SGE_SIMD_SHUFFLE(0, 1 ,3, 0));// (X Y W) ,Z component is unused
	m1.m_M128 = _mm_shuffle_ps( m.m_rows[2].m_M128,  m.m_rows[2].m_M128, SGE_SIMD_SHUFFLE(0, 1 ,3, 0));// (X Y W) ,Z component is unused
	m2.m_M128 = _mm_shuffle_ps( m.m_rows[3].m_M128,  m.m_rows[3].m_M128, SGE_SIMD_SHUFFLE(0, 1 ,3, 0));// (X Y W) ,Z component is unused

	retval += vec3_triple(m0, m1, m2) *  m.m_rows[0].z;

	//minor 3
	m0.m_M128 = _mm_shuffle_ps( m.m_rows[1].m_M128,  m.m_rows[1].m_M128, SGE_SIMD_SHUFFLE(0, 1 ,2, 0));// (X Y Z) ,W component is unused
	m1.m_M128 = _mm_shuffle_ps( m.m_rows[2].m_M128,  m.m_rows[2].m_M128, SGE_SIMD_SHUFFLE(0, 1 ,2, 0));// (X Y Z) ,W component is unused
	m2.m_M128 = _mm_shuffle_ps( m.m_rows[3].m_M128,  m.m_rows[3].m_M128, SGE_SIMD_SHUFFLE(0, 1 ,2, 0));// (X Y Z) ,W component is unused

	retval -= vec3_triple(m0, m1, m2) *  m.m_rows[0].w;

	return retval;
#else
	//minor 0
	float retval =	((m._11()*m._22()*m._33() + m._21()*m._32()*m._13() + m._12()*m._23()*m._31()) -
					(m._31()*m._22()*m._13() + m._32()*m._23()*m._11() + m._21()*m._12()*m._33())) * m._00();

	//minor 1
	retval +=		-((m._10()*m._22()*m._33() + m._20()*m._32()*m._13() + m._12()*m._23()*m._30()) -
					(m._30()*m._22()*m._13() + m._32()*m._23()*m._10() + m._20()*m._12()*m._33())) * m._01(); 

	//minor 2
	retval +=		((m._10()*m._21()*m._33() + m._20()*m._31()*m._13() + m._11()*m._23()*m._30()) - 
					(m._30()*m._21()*m._13() + m._31()*m._23()*m._10() + m._20()*m._11()*m._33())) * m._02();

	//minor 3
	retval +=		-((m._10()*m._21()*m._32() + m._20()*m._31()*m._12() + m._11()*m._22()*m._30()) - 
					(m._30()*m._21()*m._12() + m._31()*m._22()*m._10() + m._20()*m._11()*m._32())) * m._03();

	return retval;
#endif
}

//-------------------------------------------------------------------------------
//[NOTE][TODO] I think that the compiler will handle mat3 inverse better then me
//Just need to profile that
SGE_FORCE_INLINE float mat4_cofactor(const SGMatrix4& m, const int r, const int c)
{
	const int lookups[4][3] = {
		{ 1, 2, 3 }, // 0
		{ 0, 2, 3 }, // 1
		{ 0, 1, 3 }, // 2
		{ 0, 1, 2 }, // 3
	};

	const float sgn[4][4] =
	{
		{ 1.f, -1.f, 1.f, -1.f },
		{ -1.f, 1.f, -1.f, 1.f },
		{ 1.f, -1.f, 1.f, -1.f },
		{ -1.f, 1.f, -1.f, 1.f },
	};

	//take the rows
	SGVector r0 = m.m_rows[lookups[r][0]];
	SGVector r1 = m.m_rows[lookups[r][1]];
	SGVector r2 = m.m_rows[lookups[r][2]];

	//shiffle the columns in the rows
	r0.x = r0.m_floats[lookups[c][0]];
	r0.y = r0.m_floats[lookups[c][1]];
	r0.z = r0.m_floats[lookups[c][2]];

	r1.x = r1.m_floats[lookups[c][0]];
	r1.y = r1.m_floats[lookups[c][1]];
	r1.z = r1.m_floats[lookups[c][2]];

	r2.x = r2.m_floats[lookups[c][0]];
	r2.y = r2.m_floats[lookups[c][1]];
	r2.z = r2.m_floats[lookups[c][2]];

	return (vec3_triple(r0, r1, r2)) * (sgn[r][c]);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_inverse(const SGMatrix4& m)
{
	SGMatrix4 result;

	float determinant = 0.f;

	for(int s = 0; s < 4; ++s) //row
	for(int t = 0; t < 4; ++t) // col
	{
		const float cofactor = mat4_cofactor(m, s, t);
		if (s == 0) {
			determinant += cofactor * m.at(s, t);
		}

		result.m_rows[t].m_floats[s] = cofactor;
	}

	return mat4_scale(result, 1.f / determinant);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_mul_mat4(const SGVector& v, const SGMatrix4& m)
{
#if defined(SGE_MATH_USE_SSE)
	SGVector xxxx( SGE_SIMD_Splat(v.m_M128, 0) );
	SGVector yyyy( SGE_SIMD_Splat(v.m_M128, 1) );
	SGVector zzzz( SGE_SIMD_Splat(v.m_M128, 2) );
	SGVector wwww( SGE_SIMD_Splat(v.m_M128, 3) );
#else
	SGVector xxxx( v.x );
	SGVector yyyy( v.y );
	SGVector zzzz( v.z );
	SGVector wwww( v.w );
#endif

	xxxx = vec4_mul(xxxx, m.m_rows[0]);
	yyyy = vec4_mul(yyyy, m.m_rows[1]);
	zzzz = vec4_mul(zzzz, m.m_rows[2]);
	wwww = vec4_mul(wwww, m.m_rows[3]);

	xxxx = vec4_add(xxxx, yyyy);
	xxxx = vec4_add(xxxx, zzzz);
	xxxx = vec4_add(xxxx, wwww);

	return SGVector(xxxx);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector mat4_mul_vec4(const SGMatrix4& m, const SGVector& v)
{
#if defined(SGE_MATH_USE_SSE)
	SGVector xxxx( SGE_SIMD_Splat(v.m_M128, 0) );
	SGVector yyyy( SGE_SIMD_Splat(v.m_M128, 1) );
	SGVector zzzz( SGE_SIMD_Splat(v.m_M128, 2) );
	SGVector wwww( SGE_SIMD_Splat(v.m_M128, 3) );
#else
	SGVector xxxx( v.x );
	SGVector yyyy( v.y );
	SGVector zzzz( v.z );
	SGVector wwww( v.w );
#endif

	xxxx = vec4_mul(xxxx, m.GetColumn(0));
	yyyy = vec4_mul(yyyy, m.GetColumn(1));
	zzzz = vec4_mul(zzzz, m.GetColumn(2));
	wwww = vec4_mul(wwww, m.GetColumn(3));

	xxxx = vec4_add(xxxx, yyyy);
	xxxx = vec4_add(xxxx, zzzz);
	xxxx = vec4_add(xxxx, wwww);

	return SGVector(xxxx);
}

//-------------------------------------------------------------------------------
//[NOTE] that function assumes that the w component of the vector is ONE
SGE_FORCE_INLINE SGVector coord3_mul_mat4(const SGVector& v, const SGMatrix4& m)
{
#if defined(SGE_MATH_USE_SSE)
	SGVector xxxx( SGE_SIMD_Splat(v.m_M128, 0) );
	SGVector yyyy( SGE_SIMD_Splat(v.m_M128, 1) );
	SGVector zzzz( SGE_SIMD_Splat(v.m_M128, 2) );
#else
	SGVector xxxx( v.x );
	SGVector yyyy( v.y );
	SGVector zzzz( v.z );
#endif

	xxxx = vec4_mul(xxxx, m.m_rows[0]);
	yyyy = vec4_mul(yyyy, m.m_rows[1]);
	zzzz = vec4_mul(zzzz, m.m_rows[2]);

	xxxx = vec4_add(xxxx, yyyy);
	xxxx = vec4_add(xxxx, zzzz);
	xxxx = vec4_add(xxxx, m.m_rows[3]);

	return SGVector(xxxx);
}

//-------------------------------------------------------------------------------
//[NOTE] that function assumes that the w component of the vector is ONE
SGE_FORCE_INLINE SGVector mat4_mul_coord3(const SGMatrix4& m, const SGVector& v)
{
#if defined(SGE_MATH_USE_SSE)
	SGVector xxxx( SGE_SIMD_Splat(v.m_M128, 0) );
	SGVector yyyy( SGE_SIMD_Splat(v.m_M128, 1) );
	SGVector zzzz( SGE_SIMD_Splat(v.m_M128, 2) );
#else
	SGVector xxxx( v.x );
	SGVector yyyy( v.y );
	SGVector zzzz( v.z );
#endif

	xxxx = vec4_mul(xxxx, m.GetColumn(0));
	yyyy = vec4_mul(yyyy, m.GetColumn(1));
	zzzz = vec4_mul(zzzz, m.GetColumn(2));

	xxxx = vec4_add(xxxx, yyyy);
	xxxx = vec4_add(xxxx, zzzz);
	xxxx = vec4_add(xxxx, m.GetColumn(3));

	return SGVector(xxxx);
}

//-------------------------------------------------------------------------------
//[NOTE] that function assumes that the w component of the vector is ZERO
SGE_FORCE_INLINE SGVector norm3_mul_mat4(const SGVector& v, const SGMatrix4& m)
{
#if defined(SGE_MATH_USE_SSE)
	SGVector xxxx( SGE_SIMD_Splat(v.m_M128, 0) );
	SGVector yyyy( SGE_SIMD_Splat(v.m_M128, 1) );
	SGVector zzzz( SGE_SIMD_Splat(v.m_M128, 2) );
#else
	SGVector xxxx( v.x );
	SGVector yyyy( v.y );
	SGVector zzzz( v.z );
#endif

	xxxx = vec4_mul(xxxx, m.m_rows[0]);
	yyyy = vec4_mul(yyyy, m.m_rows[1]);
	zzzz = vec4_mul(zzzz, m.m_rows[2]);

	xxxx = vec4_add(xxxx, yyyy);
	xxxx = vec4_add(xxxx, zzzz);

	return SGVector(xxxx);
}

//-------------------------------------------------------------------------------
//[NOTE] that function assumes that the w component of the vector is ZERO
SGE_FORCE_INLINE SGVector norm3_mul_mat4(const SGMatrix4& m, const SGVector& v)
{
#if defined(SGE_MATH_USE_SSE)
	SGVector xxxx( SGE_SIMD_Splat(v.m_M128, 0) );
	SGVector yyyy( SGE_SIMD_Splat(v.m_M128, 1) );
	SGVector zzzz( SGE_SIMD_Splat(v.m_M128, 2) );

#else
	SGVector xxxx( v.x );
	SGVector yyyy( v.y );
	SGVector zzzz( v.z );
#endif

	xxxx = vec4_mul(xxxx, m.GetColumn(0));
	yyyy = vec4_mul(yyyy, m.GetColumn(1));
	zzzz = vec4_mul(zzzz, m.GetColumn(2));

	xxxx = vec4_add(xxxx, yyyy);
	xxxx = vec4_add(xxxx, zzzz);

	return SGVector(xxxx);
}

SGE_FORCE_INLINE SGMatrix4 mat4_mul(const SGMatrix4& a, const SGMatrix4& b)
{
	return SGMatrix4(
					vec4_mul_mat4(a.m_rows[0], b),
					vec4_mul_mat4(a.m_rows[1], b),
					vec4_mul_mat4(a.m_rows[2], b),
					vec4_mul_mat4(a.m_rows[3], b)	);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Some common matrix transforms
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE void mat4_loadIdentity(SGMatrix4& m)
{
	m.m_rows[0] = SGVector::V1000;
	m.m_rows[1] = SGVector::V0100;
	m.m_rows[2] = SGVector::V0010;
	m.m_rows[3] = SGVector::V0001;
}

#include "SGMatrix4CommonTransforms_RM.h"