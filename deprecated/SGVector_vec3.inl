/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will threat SGVector as a 3D vector
//[NOTE] Those functions may change the _w component of the vector
//Designed to be inlined in SGVector.h
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGVector vec3_add(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	 return SGVector(_mm_add_ps(a.m_M128, b.m_M128));
#else
	 return SGVector(a.x + b.x, a.y + b.y, a.z + b.z, 0.f); //skip the w component
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_sub(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	 return SGVector(_mm_sub_ps(a.m_M128, b.m_M128));
#else
	 return SGVector(a.x - b.x, a.y - b.y, a.z - b.z, 0.f); //skip the w component
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_scale(const SGVector& a, const float s)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vs = _mm_set1_ps(s);
	return SGVector(_mm_mul_ps(a.m_M128, vs));
#else
	 return SGVector(a.x * s, a.y * s, a.z * s, 0.f); //skip the w component
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_inverse(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	return SGVector(_mm_div_ps(SGVector::V1111.m_M128 ,a.m_M128));
#else
	 return SGVector(1.f / a.x, 1.f / a.y, 1.f / a.z, 0.f); //skip the w component
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_mul(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	 return SGVector(_mm_mul_ps(a.m_M128, b.m_M128));
#else
	 return SGVector(a.x * b.x, a.y * b.y, a.z * b.z, 0.f); //skip the w component
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float vec3_dot(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, b.m_M128);	//x*x, y*y, z*z, w*w
	vd = _mm_and_ps(vd, SGE_SSE_MASK_FFF0);			//set the w = 0
	vd = _mm_hadd_ps(vd, vd);
	vd = _mm_hadd_ps(vd, vd);

	return SGE_SIMD_ExtractComponent(vd, 0);
#else
	 return (a.x * b.x) + (a.y * b.y) + (a.z * b.z); //skip the w component
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_dot3(const SGVector& V, const SGVector& a, const SGVector& b, const SGVector& c)
{
	const float x = vec3_dot(V, a);
	const float y = vec3_dot(V, b);
	const float z = vec3_dot(V, c);

	return SGVector(x, y, z, 0);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float vec3_lenghtSqr(const SGVector& a)
{
	return vec3_dot(a, a);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float vec3_lenght(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, a.m_M128); //x*x, y*y, z*z, w*w
	vd = _mm_and_ps(vd, SGE_SSE_MASK_FFF0);		//set the w = 0
	vd = _mm_hadd_ps(vd, vd);					// v = x + y, z + w,  x + y, z + w
	vd = _mm_hadd_ps(vd, vd);					// all components are equal to x + y + z + w
	vd = _mm_sqrt_ps(vd);						// sqrt(v.?)

	return SGE_SIMD_ExtractComponent(vd, 0);
#else
	return sqrtf(vec3_dot(a, a));
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_cross(const SGVector& a, const SGVector& b)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 T = _mm_shuffle_ps(a.m_M128, a.m_M128, SGE_SIMD_SHUFFLE(1, 2, 0, 3));	//(Y Z X 0)
	__m128 V = _mm_shuffle_ps(b.m_M128, b.m_M128, SGE_SIMD_SHUFFLE(1, 2, 0, 3));	//(Y Z X 0)

	//i(ay*bz - by*az)	 +	j(bx*az - ax*bz)	 +		k(ax*by - bx*ay)
	T = _mm_mul_ps(T, b.m_M128);//bx * ay,		by * az,		bz * ax
	V = _mm_mul_ps(V, a.m_M128);//ax * by,		ay * bz,		az * bx
	V = _mm_sub_ps(V, T);

	V = _mm_shuffle_ps(V, V, SGE_SIMD_SHUFFLE(1, 2, 0, 3));
	return SGVector(V);
#else
	const float x = (a.y*b.z) - (b.y*a.z);
	const float y = (b.x*a.z) - (a.x*b.z);
	const float z = (a.x*b.y) - (b.x*a.y);

	return SGVector(x, y, z, 0.f);
#endif
}

//-------------------------------------------------------------------------------
//[NOTE] a dot (b cross c)
SGE_FORCE_INLINE float vec3_triple(const SGVector& a, const SGVector& b, const SGVector& c)
{
	const SGVector V = vec3_cross(b, c);
	return vec3_dot(a, V);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector vec3_normalized(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, a.m_M128); //x*x, y*y, z*z, w*w
	vd = _mm_and_ps(vd, SGE_SSE_MASK_FFF0);		//set the w = 0
	vd = _mm_hadd_ps(vd, vd);					// v = x + y, z + w, x + y, z + w
	vd = _mm_hadd_ps(vd, vd);					// all components are equal to x + y + z + w (equal to the a.length^2)
	vd = _mm_rsqrt_ps(vd);						// 1 / sqrt(v.?)
	
	return SGVector( _mm_mul_ps(a.m_M128, vd));
#else
	float len = vec3_lenght(a);
	float invLen = 1.f / len;

	return SGVector(a.x * invLen, a.y * invLen, a.z * invLen, 0.f);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE void vec3_normalize(SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, a.m_M128); //x*x, y*y, z*z, w*w
	vd = _mm_and_ps(vd, SGE_SSE_MASK_FFF0);		//set the w = 0
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
#endif
}