#pragma once

#include "sgmath.h"

SGE_BEGIN_MATH_NAMESPACE

template<typename TDATA_TYPE>
struct mat44
{
	typedef typename TDATA_TYPE DATA_TYPE;
	typedef mat44 SELF_TYPE;
	typedef vec4<DATA_TYPE> VEC_TYPE;

	static const unsigned int NUM_ROW = 4;
	static const unsigned int NUM_COL = 4;

	static const unsigned int VEC_SIZE = NUM_COL;
	static const unsigned int NUM_VECS = NUM_ROW;

	VEC_TYPE axis[NUM_VECS];

	#include "mat_common.inl"

	//-------------------------------------------------------------------------------
	SELF_TYPE inverse()
	{
		//stolen for now
		DATA_TYPE s[6];
		DATA_TYPE c[6];

		s[0] = M[0][0]*M[1][1] - M[1][0]*M[0][1];
		s[1] = M[0][0]*M[1][2] - M[1][0]*M[0][2];
		s[2] = M[0][0]*M[1][3] - M[1][0]*M[0][3];
		s[3] = M[0][1]*M[1][2] - M[1][1]*M[0][2];
		s[4] = M[0][1]*M[1][3] - M[1][1]*M[0][3];
		s[5] = M[0][2]*M[1][3] - M[1][2]*M[0][3];

		c[0] = M[2][0]*M[3][1] - M[3][0]*M[2][1];
		c[1] = M[2][0]*M[3][2] - M[3][0]*M[2][2];
		c[2] = M[2][0]*M[3][3] - M[3][0]*M[2][3];
		c[3] = M[2][1]*M[3][2] - M[3][1]*M[2][2];
		c[4] = M[2][1]*M[3][3] - M[3][1]*M[2][3];
		c[5] = M[2][2]*M[3][3] - M[3][2]*M[2][3];

		DATA_TYPE idet = DATA_TYPE(1.0)/( s[0]*c[5]-s[1]*c[4]+s[2]*c[3]+s[3]*c[2]-s[4]*c[1]+s[5]*c[0] );

		T[0][0] = ( M[1][1] * c[5] - M[1][2] * c[4] + M[1][3] * c[3]) * idet;
		T[0][1] = (-M[0][1] * c[5] + M[0][2] * c[4] - M[0][3] * c[3]) * idet;
		T[0][2] = ( M[3][1] * s[5] - M[3][2] * s[4] + M[3][3] * s[3]) * idet;
		T[0][3] = (-M[2][1] * s[5] + M[2][2] * s[4] - M[2][3] * s[3]) * idet;

		T[1][0] = (-M[1][0] * c[5] + M[1][2] * c[2] - M[1][3] * c[1]) * idet;
		T[1][1] = ( M[0][0] * c[5] - M[0][2] * c[2] + M[0][3] * c[1]) * idet;
		T[1][2] = (-M[3][0] * s[5] + M[3][2] * s[2] - M[3][3] * s[1]) * idet;
		T[1][3] = ( M[2][0] * s[5] - M[2][2] * s[2] + M[2][3] * s[1]) * idet;

		T[2][0] = ( M[1][0] * c[4] - M[1][1] * c[2] + M[1][3] * c[0]) * idet;
		T[2][1] = (-M[0][0] * c[4] + M[0][1] * c[2] - M[0][3] * c[0]) * idet;
		T[2][2] = ( M[3][0] * s[4] - M[3][1] * s[2] + M[3][3] * s[0]) * idet;
		T[2][3] = (-M[2][0] * s[4] + M[2][1] * s[2] - M[2][3] * s[0]) * idet;

		T[3][0] = (-M[1][0] * c[3] + M[1][1] * c[1] - M[1][2] * c[0]) * idet;
		T[3][1] = ( M[0][0] * c[3] - M[0][1] * c[1] + M[0][2] * c[0]) * idet;
		T[3][2] = (-M[3][0] * s[3] + M[3][1] * s[1] - M[3][2] * s[0]) * idet;
		T[3][3] = ( M[2][0] * s[3] - M[2][1] * s[1] + M[2][2] * s[0]) * idet;
	}

	//-------------------------------------------------------------------------------
	friend mat44 matrix_translation(const VEC_TYPE_PREV& v)
	{
		mat44 result;

		for(unsigned int t = 0; t < NUM_VECS - 1; ++t)
		{
			r.identify_axis(t);
		}

		for(unsigned int t = 0; t < VEC_SIZE - 1; ++t)
		{
			r.axis[NUM_VECS - 1][t] = v[t];
		}

		r.axis[NUM_VECS - 1][VEC_SIZE - 1] = 1.f;

		return result;
	}
};




SGE_END_MATH_NAMESPACE