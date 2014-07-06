#pragma once

#include "sgmath.h"
#include "vec.h"

SGE_BEGIN_MATH_NAMESPACE

////////////////////////////////////////////////////////////////////////
//struct mat33: A row major storage matrix type
////////////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct mat33
{
	typedef typename TDATA_TYPE DATA_TYPE;
	typedef mat33 SELF_TYPE;
	typedef vec3<DATA_TYPE> VEC_TYPE;
	typedef vec3<DATA_TYPE> VEC_TYPE_PRE;

	static const unsigned int NUM_ROW = 3;
	static const unsigned int NUM_COL = 3;

	static const unsigned int VEC_SIZE = NUM_COL;
	static const unsigned int NUM_VECS = NUM_ROW;

	VEC_TYPE data[NUM_VECS];
	
	#include "mat_common.inl"

	//---------------------------------------------------
	//inverse
	//---------------------------------------------------
	SELF_TYPE inverse() const
	{
		SELF_TYPE result;

		const DATA_TYPE invDet = (DATA_TYPE)1.0 / triple(data[0], data[1], data[2]);

		result.at(0,0) =  (at(1,1)*at(2,2) - at(2,1)*at(1,2)) * invDet;							
		result.at(0,1) = -(at(0,1)*at(2,2) - at(2,1)*at(0,2)) * invDet;
		result.at(0,2) =  (at(0,1)*at(1,2) - at(0,2)*at(1,1)) * invDet;

		result.at(1,0) = -(at(1,0)*at(2,2) - at(1,2)*at(2,0)) * invDet;
		result.at(1,1) =  (at(0,0)*at(2,2) - at(0,2)*at(2,0)) * invDet;
		result.at(1,2) = -(at(0,0)*at(1,2) - at(0,2)*at(1,0)) * invDet;

		result.at(2,0) =  (at(1,0)*at(2,1) - at(1,1)*at(2,0)) * invDet;
		result.at(2,1) = -(at(0,0)*at(2,1) - at(0,1)*at(2,0)) * invDet;
		result.at(2,2) =  (at(0,0)*at(1,1) - at(1,0)*at(0,1)) * invDet;

		return result;
	}

	friend SELF_TYPE inverse(const SELF_TYPE& m)
	{
		return m.inverse();
	}
};

////////////////////////////////////////////////////////////////////////
//struct mat44 A row major storage matrix type
////////////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct mat44
{
	typedef typename TDATA_TYPE DATA_TYPE;
	typedef mat44 SELF_TYPE;
	typedef vec4<DATA_TYPE> VEC_TYPE;
	typedef vec4<DATA_TYPE> VEC_TYPE_PRE;

	static const unsigned int NUM_ROW = 4;
	static const unsigned int NUM_COL = 4;

	static const unsigned int VEC_SIZE = NUM_COL;
	static const unsigned int NUM_VECS = NUM_ROW;

	VEC_TYPE data[NUM_VECS];

	#include "mat_common.inl"

	//---------------------------------------------------
	//inverse
	//---------------------------------------------------
	SELF_TYPE inverse() const
	{
		const DATA_TYPE s[6] = 
		{
			data[0][0]*data[1][1] - data[1][0]*data[0][1],
			data[0][0]*data[1][2] - data[1][0]*data[0][2],
			data[0][0]*data[1][3] - data[1][0]*data[0][3],
			data[0][1]*data[1][2] - data[1][1]*data[0][2],
			data[0][1]*data[1][3] - data[1][1]*data[0][3],
			data[0][2]*data[1][3] - data[1][2]*data[0][3],
		};

		const DATA_TYPE c[6] = 
		{
			data[2][0]*data[3][1] - data[3][0]*data[2][1],
			data[2][0]*data[3][2] - data[3][0]*data[2][2],
			data[2][0]*data[3][3] - data[3][0]*data[2][3],
			data[2][1]*data[3][2] - data[3][1]*data[2][2],
			data[2][1]*data[3][3] - data[3][1]*data[2][3],
			data[2][2]*data[3][3] - data[3][2]*data[2][3],
		};

		DATA_TYPE invDet = (DATA_TYPE)(1.0)/(s[0]*c[5]-s[1]*c[4]+s[2]*c[3]+s[3]*c[2]-s[4]*c[1]+s[5]*c[0]);

		SELF_TYPE result;

		result.at(0,0) = ( data[1][1] * c[5] - data[1][2] * c[4] + data[1][3] * c[3]) * invDet;
		result.at(0,1) = (-data[0][1] * c[5] + data[0][2] * c[4] - data[0][3] * c[3]) * invDet;
		result.at(0,2) = ( data[3][1] * s[5] - data[3][2] * s[4] + data[3][3] * s[3]) * invDet;
		result.at(0,3) = (-data[2][1] * s[5] + data[2][2] * s[4] - data[2][3] * s[3]) * invDet;

		result.at(1,0) = (-data[1][0] * c[5] + data[1][2] * c[2] - data[1][3] * c[1]) * invDet;
		result.at(1,1) = ( data[0][0] * c[5] - data[0][2] * c[2] + data[0][3] * c[1]) * invDet;
		result.at(1,2) = (-data[3][0] * s[5] + data[3][2] * s[2] - data[3][3] * s[1]) * invDet;
		result.at(1,3) = ( data[2][0] * s[5] - data[2][2] * s[2] + data[2][3] * s[1]) * invDet;

		result.at(2,0) = ( data[1][0] * c[4] - data[1][1] * c[2] + data[1][3] * c[0]) * invDet;
		result.at(2,1) = (-data[0][0] * c[4] + data[0][1] * c[2] - data[0][3] * c[0]) * invDet;
		result.at(2,2) = ( data[3][0] * s[4] - data[3][1] * s[2] + data[3][3] * s[0]) * invDet;
		result.at(2,3) = (-data[2][0] * s[4] + data[2][1] * s[2] - data[2][3] * s[0]) * invDet;

		result.at(3,0) = (-data[1][0] * c[3] + data[1][1] * c[1] - data[1][2] * c[0]) * invDet;
		result.at(3,1) = ( data[0][0] * c[3] - data[0][1] * c[1] + data[0][2] * c[0]) * invDet;
		result.at(3,2) = (-data[3][0] * s[3] + data[3][1] * s[1] - data[3][2] * s[0]) * invDet;
		result.at(3,3) = ( data[2][0] * s[3] - data[2][1] * s[1] + data[2][2] * s[0]) * invDet;

		return result;
	}

	friend SELF_TYPE inverse(const SELF_TYPE& m)
	{
		return m.inverse();
	}
};

SGE_END_MATH_NAMESPACE