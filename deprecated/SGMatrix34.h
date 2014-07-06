#pragma once

#include "SGVector.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//struct SGMatrix34
//[NOTE]: each SGVector represents a single row. mat3 functions will threat that Structure as s 3x3 matrix
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct SGMatrix34
{
	SGVector m_rows[3];

	//------------------------------------------------------------------
	//Constructors
	//------------------------------------------------------------------

	SGMatrix34() {}

	SGMatrix34(	const float _11, const float _12, const float _13, const float _14,
				const float _21, const float _22, const float _23, const float _24,
				const float _31, const float _32, const float _33, const float _34)

	{
		m_rows[0] = SGVector(_11, _12, _13, _14);
		m_rows[1] = SGVector(_21, _22, _23, _24);
		m_rows[2] = SGVector(_31, _32, _33, _34);
	}

	SGMatrix34( const float _11, const float _12, const float _13,
				const float _21, const float _22, const float _23,
				const float _31, const float _32, const float _33)
	{
		m_rows[0] = SGVector(_11, _12, _13, 0.f);
		m_rows[1] = SGVector(_21, _22, _23, 0.f);
		m_rows[2] = SGVector(_31, _32, _33, 0.f);
	}

	SGMatrix34(const SGVector& r0, const SGVector& r1, const SGVector& r2)
	{
		m_rows[0] = (r0);
		m_rows[1] = (r1);
		m_rows[2] = (r2);
	}

#if defined(SGE_MATH_USE_SSE)
	SGMatrix34(const __m128& r0, const __m128& r1, const __m128& r2)
	{
		m_rows[0].m_M128 = (r0);
		m_rows[1].m_M128 = (r1);
		m_rows[2].m_M128 = (r2);
	}
#endif

	//------------------------------------------------------------------
	SGVector GetColumn(const int i) const
	{
		return SGVector(m_rows[0].m_floats[i], 
			m_rows[1].m_floats[i], 
			m_rows[2].m_floats[i], 
			0.f);
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
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will thread SGMatrix34 as a 3x3 matrix
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGMatrix34 mat3_add(const SGMatrix34& a, const SGMatrix34& b)
{
	return SGMatrix34(	vec4_add( a.m_rows[0], a.m_rows[0]),
						vec4_add( a.m_rows[1], a.m_rows[1]),
						vec4_add( a.m_rows[2], a.m_rows[2]));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_sub(const SGMatrix34& a, const SGMatrix34& b)
{
	return SGMatrix34(	vec4_sub( a.m_rows[0], a.m_rows[0]),
						vec4_sub( a.m_rows[1], a.m_rows[1]),
						vec4_sub( a.m_rows[2], a.m_rows[2]));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_scale(const SGMatrix34& m, const float s)
{
	return SGMatrix34(vec4_scale(m.m_rows[0], s),
		vec4_scale(m.m_rows[1], s),
		vec4_scale(m.m_rows[2], s));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_getTranspose(const SGMatrix34& m)
{
	const SGVector c0 = m.GetColumn(0);
	const SGVector c1 = m.GetColumn(1);
	const SGVector c2 = m.GetColumn(2);

	return SGMatrix34(c0, c1, c2);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float mat3_determinant(const SGMatrix34& m)
{
	return vec3_triple(m.m_rows[0], m.m_rows[1], m.m_rows[2]);  
}

//-------------------------------------------------------------------------------
//[NOTE][TODO] I think that the compiler will handle mat3 inverse better then me
//Just need to profile that
SGE_FORCE_INLINE float mat3_cofactor(const SGMatrix34& m, const int r, const int c)
{
	const int lookups[3][2] = {
		{ 1, 2 }, // 0
		{ 0, 2 }, // 1
		{ 0, 1 }, // 2
	};

	const float sgn[3][3] =
	{
		{ 1.f, -1.f, 1.f },
		{ -1.f, 1.f, -1.f },
		{ 1.f, -1.f, 1.f },
	};

	const SGVector& r0 = m.m_rows[lookups[r][0]];
	const SGVector& r1 = m.m_rows[lookups[r][1]];

	return	(r0.m_floats[lookups[c][0]] * r1.m_floats[lookups[c][1]] - 
			 r0.m_floats[lookups[c][1]] * r1.m_floats[lookups[c][0]]) * sgn[r][c];
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_inverse(const SGMatrix34& m)
{
	SGMatrix34 result;

	float determinant = 0.f;

	for(int s = 0; s < 3; ++s) //row
	for(int t = 0; t < 3; ++t) // col
	{
		const float cofactor = mat3_cofactor(m, s, t);
		if (s == 0) {
			determinant += cofactor * m.at(s, t);
		}

		result.m_rows[t].m_floats[s] = cofactor;
	}

	return mat3_scale(result, 1.f / determinant);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_mul_mat3(const SGVector& v, const SGMatrix34& m)
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

	xxxx = vec3_mul(xxxx, m.m_rows[0]);
	yyyy = vec3_mul(yyyy, m.m_rows[1]);
	zzzz = vec3_mul(zzzz, m.m_rows[2]);

	xxxx = vec3_add(xxxx, yyyy);
	xxxx = vec3_add(xxxx, zzzz);

	return SGVector(xxxx);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector mat4_mul_vec4(const SGMatrix34& m, const SGVector& v)
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

	xxxx = vec3_mul(xxxx, m.GetColumn(0));
	yyyy = vec3_mul(yyyy, m.GetColumn(1));
	zzzz = vec3_mul(zzzz, m.GetColumn(2));

	xxxx = vec3_add(xxxx, yyyy);
	xxxx = vec3_add(xxxx, zzzz);

	return SGVector(xxxx);
}

SGE_FORCE_INLINE SGMatrix34 mat3_mul(const SGMatrix34& a, const SGMatrix34& b)
{
	return SGMatrix34(
					vec3_mul_mat3(a.m_rows[0], b),
					vec3_mul_mat3(a.m_rows[1], b),
					vec3_mul_mat3(a.m_rows[2], b) );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Some common matrix transforms
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE void mat34_loadIdentity(SGMatrix34& m)
{
	m.m_rows[0] = SGVector::V1000;
	m.m_rows[1] = SGVector::V0100;
	m.m_rows[2] = SGVector::V0010;
}

SGE_FORCE_INLINE void mat3_loadIdentity(SGMatrix34& m)
{
	mat34_loadIdentity(m);
}

#include "SGMatrix3CommonTransforms_RM.h"