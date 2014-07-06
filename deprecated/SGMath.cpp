#include "SGMath.h"

#if defined(SGE_MATH_USE_SSE)
	const __m128 SGE_SSE_MASK_FFF0 = _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000));
	const __m128 SGE_SSE_MASK_QUAT_CONJURGATE = _mm_castsi128_ps(_mm_setr_epi32(0x80000000, 0x80000000, 0x80000000, 0));
#endif

const SGVector SGVector::V0000 =	SGVector(0.f, 0.f, 0.f, 0.f);
const SGVector SGVector::V1111 =	SGVector(1.f, 1.f, 1.f, 1.f);
const SGVector SGVector::V1000 =	SGVector(1.f, 0.f, 0.f, 0.f);
const SGVector SGVector::V0100 =	SGVector(0.f, 1.f, 0.f, 0.f);
const SGVector SGVector::V0010 =	SGVector(0.f, 0.f, 1.f, 0.f);
const SGVector SGVector::V0001 =	SGVector(0.f, 0.f, 0.f, 1.f);


