/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operations that will threat SGVector as a quaterion
//[NOTE]The quaterion layout is defined by:
//(x, y, z) representing the imaginary part of the quat and (w) the real part
//Designed to be inlined in SGVector.h
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGVector quat_from_axisRotation(const SGVector& axis, const float theta)
{
	const float cosTheta = cosf(theta * 0.5f);
	const float sinTheta = sinf(theta * 0.5f);

	const SGVector a(sinTheta, sinTheta, sinTheta, cosTheta);
	const SGVector b(axis.x, axis.y, axis.z, 1.f);

	return vec4_mul(a, b);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_scale(const SGVector& a, const float s)
{
	return vec4_scale(a, s);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_add(const SGVector& a, const SGVector& b)
{
	return vec4_add(a, b);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_sub(const SGVector& a, const SGVector& b)
{
	return vec4_sub(a, b);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_conjurgate(const SGVector& a)
{
	return a.ChangeSign<1, 1, 1, 0>();
//#if defined (SGE_MATH_USE_SSE)
//	return SGVector(_mm_xor_ps(a.m_M128, SGE_SSE_MASK_QUAT_CONJURGATE));
//#else
//	return SGVector(-a.x, -a.y, -a.z, a.w);
//#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_mul(const SGVector& a, const SGVector& b)
{
#if defined (SGE_MATH_USE_SSE)
	__m128 bt0 = _mm_shuffle_ps(b.m_M128, b.m_M128, SGE_SIMD_SHUFFLE(3, 2, 1, 0));//wzyx
	__m128 bt1 = _mm_shuffle_ps(b.m_M128, b.m_M128, SGE_SIMD_SHUFFLE(2, 3, 0, 1));//zwxy
	__m128 bt2 = _mm_shuffle_ps(b.m_M128, b.m_M128, SGE_SIMD_SHUFFLE(1, 0, 3, 2));//yxwz
	//__m128 bt3 ;//just b

	bt0 = _mm_mul_ps(a.m_M128, bt0);
	bt1 = _mm_mul_ps(a.m_M128, bt1);
	bt2 = _mm_mul_ps(a.m_M128, bt2);
	__m128 bt3 = _mm_mul_ps(a.m_M128, b.m_M128);

	bt0 = SGE_SIMD_ChangeSign<0, 1, 0, 0>(bt0);
	bt1 = SGE_SIMD_ChangeSign<0, 0, 1, 0>(bt1);
	bt2 = SGE_SIMD_ChangeSign<1, 0, 0, 0>(bt2);
	bt3 = SGE_SIMD_ChangeSign<1, 1, 1, 0>(bt3);

	return SGVector(SGE_SIMD_sum(bt0), SGE_SIMD_sum(bt1), SGE_SIMD_sum(bt2), SGE_SIMD_sum(bt3));
#else
	SGVector bt0 = b.Permute<3, 2, 1, 0>();//wzyx
	SGVector bt1 = b.Permute<2, 3, 0, 1>();//zwxy
	SGVector bt2 = b.Permute<1, 0, 3, 2>();//yxwz
	//SGVector bt3 = b;//just b

	bt0 = vec4_mul(a, bt0);
	bt1 = vec4_mul(a, bt1);
	bt2 = vec4_mul(a, bt2);
	SGVector bt3 = vec4_mul(a, b);

	bt0 = vec_changeSign<0, 0, 1, 1>(bt0);
	bt1 = vec_changeSign<1, 0, 0, 1>(bt1);
	bt2 = vec_changeSign<0, 1, 0, 1>(bt2);
	bt3 = vec_changeSign<0, 0, 0, 0>(bt3);

	return SGVector(vec_sum(bt0), vec_sum(bt1), vec_sum(bt2), vec_sum(bt3));
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float quat_normSqr(const SGVector& a)
{
	return vec4_lenghtSqr(a);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE float quat_norm(const SGVector& a)
{
	return sqrtf(quat_normSqr(a));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_inverse(const SGVector& a)
{
#if defined(SGE_MATH_USE_SSE)
	__m128 vd = _mm_mul_ps(a.m_M128, a.m_M128);
	vd = _mm_hadd_ps(vd, vd);					
	vd = _mm_hadd_ps(vd, vd);					
	return SGVector( _mm_div_ps(quat_conjurgate(a).m_M128, vd) );
#else
	const float invNormSqr = 1.f / quat_normSqr(a);
	return vec4_scale(quat_conjurgate(a), invNormSqr);
#endif
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_normalized(const SGVector& a)
{
	return vec4_normalized(a);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGVector quat_lerpAndNorm(const SGVector& a, const SGVector& b, const float t)
{
	return quat_normalized( quat_add(quat_scale(a, 1.f - t), quat_scale(b, t)) );
}

//-------------------------------------------------------------------------------
//[TODO]
//Inpterpolates 2 quaternions. if the angel between the axes is lower than tolerance
//a linear interpolation will be performed
SGE_FORCE_INLINE SGVector quat_slerp(const SGVector& q1, const SGVector& q2, const float t, const float tolerance)
{
	SGVector q3;
	float dot = vec4_dot(q1, q2);

	/*	dot = cos(theta)
		if (dot < 0), q1 and q2 are more than 90 degrees apart,
		so we can invert one to reduce spinning	*/
	if (dot < 0)
	{
		dot = -dot;
		q3 = -q2;
	} else q3 = q2;
		
	if (dot < 0.95f)
	{
		float angle = acosf(dot);
		return quat_scale(quat_scale(q1,sinf(angle*(1-t))) + quat_scale(q3,sinf(angle*t)),1.f/sinf(angle));
	} else // if the angle is small, use linear interpolation								
		return quat_lerpAndNorm(q1,q3,t);		
}