/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will threat SGVector as a 4D vector
//[NOTE] Those functions may change the _w component of the vector
//Designed to be inlined in SGVector.h
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGVector vec4_negative(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	return SGVector(_mm_sub_ps(SGVector::V0000.m_M128, a.m_M128));
#else
	return SGVector(-a.x, -a.y, -a.z, -a.w);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_add(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	 return SGVector(_mm_add_ps(a.m_M128, b.m_M128));
#else
	 return SGVector(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_sub(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	 return SGVector(_mm_sub_ps(a.m_M128, b.m_M128));
#else
	 return SGVector(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_scale(const SGVector& a, const float s)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vs = _mm_set1_ps(s);
	return SGVector(_mm_mul_ps(a.m_M128, vs));
#else
	 return SGVector(a.x * s, a.y * s, a.z * s, a.w * s);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_inverse(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	return SGVector(_mm_div_ps(SGVector::V1111.m_M128 ,a.m_M128));
#else
	 return SGVector(1.f / a.x, 1.f / a.y, 1.f / a.z, 1.f / a.w);
#endif
}


//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_mul(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	 return SGVector(_mm_mul_ps(a.m_M128, b.m_M128));
#else
	 return SGVector(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float vec4_dot(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, b.m_M128);	//x*x, y*y, z*z, w*w
	vd = _mm_hadd_ps(vd, vd);
	vd = _mm_hadd_ps(vd, vd);

	return SGE_SIMD_ExtractComponent(vd, 0);
#else
	 return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float vec4_lenghtSqr(const SGVector& a)
{
	return vec4_dot(a, a);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float vec4_lenght(const SGVector& a)
{
	return sqrtf(vec4_lenghtSqr(a));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec4_normalized(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, a.m_M128); //x*x, y*y, z*z, w*w
	vd = _mm_hadd_ps(vd, vd);					// v = x + y, z + w, x + y, z + w
	vd = _mm_hadd_ps(vd, vd);					// all components are equal to x + y + z + w (equal to the a.length^2)
	vd = _mm_rsqrt_ps(vd);						// 1 / sqrt(v.?)
	
	return SGVector( _mm_mul_ps(a.m_M128, vd) );
#else
	const float len = vec3_lenght(a);
	const float invLen = 1.f / len;

	return SGVector(a.x * invLen, a.y * invLen, a.z * invLen, a.w * invLen);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE void vec4_normalize(SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, a.m_M128); //x*x, y*y, z*z, w*w
	vd = _mm_hadd_ps(vd, vd);					// v = x + y, z + w, x + y, z + w
	vd = _mm_hadd_ps(vd, vd);					// all components are equal to x + y + z + w (equal to the a.length^2)
	vd = _mm_rsqrt_ps(vd);						// 1 / sqrt(v.?)
	
	a.m_M128 = _mm_mul_ps(a.m_M128, vd);
#else
	float len = vec3_lenght(a);
	float invLen = 1.f / len;

	a.x *= invLen;
	a.y *= invLen;
	a.z *= invLen;
	a.w *= invLen;

#endif
}