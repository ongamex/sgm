#pragma once

#include "sgm_base.h"
#include "trigonometry.h"
#include "vec.h"
#include "quat.h"

SGE_BEGIN_MATH_NAMESPACE

////////////////////////////////////////////////////////////////////////
//struct matrix33: A column major storage matrix type
////////////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct matrix33
{
	typedef typename TDATA_TYPE DATA_TYPE;
	typedef matrix33 SELF_TYPE;
	typedef vec3<DATA_TYPE> VEC_TYPE;

	static const unsigned int NUM_ROW = 3;
	static const unsigned int NUM_COL = 3;

	static const unsigned int VEC_SIZE = NUM_COL;
	static const unsigned int NUM_VECS = NUM_ROW;

	//keep this named like this in order to be abe to quickly add row major
	VEC_TYPE data[NUM_VECS];
	
	#include "matrix_common.inl"

	//---------------------------------------------------
	//inverse
	//---------------------------------------------------
	SELF_TYPE inverse() const
	{
		SELF_TYPE result;

		const DATA_TYPE invDet = (DATA_TYPE)1.0 / triple(data[0], data[1], data[2]);

		result.data[0][0] =  (data[1][1]*data[2][2] - data[2][1]*data[1][2]) * invDet;							
		result.data[0][1] = -(data[0][1]*data[2][2] - data[2][1]*data[0][2]) * invDet;
		result.data[0][2] =  (data[0][1]*data[1][2] - data[0][2]*data[1][1]) * invDet;

		result.data[1][0] = -(data[1][0]*data[2][2] - data[1][2]*data[2][0]) * invDet;
		result.data[1][1] =  (data[0][0]*data[2][2] - data[0][2]*data[2][0]) * invDet;
		result.data[1][2] = -(data[0][0]*data[1][2] - data[0][2]*data[1][0]) * invDet;

		result.data[2][0] =  (data[1][0]*data[2][1] - data[1][1]*data[2][0]) * invDet;
		result.data[2][1] = -(data[0][0]*data[2][1] - data[0][1]*data[2][0]) * invDet;
		result.data[2][2] =  (data[0][0]*data[1][1] - data[1][0]*data[0][1]) * invDet;

		return result;
	}

	friend SELF_TYPE inverse(const SELF_TYPE& m)
	{
		return m.inverse();
	}

	////////////////////////////////////////////////////////////////////////
	//Commonly used transformations
	////////////////////////////////////////////////////////////////////////

	//----------------------------------------------------------------
	//zero matrix
	//----------------------------------------------------------------
	friend void matrix_identity(SELF_TYPE& result)
	{
		result.identify_axis(0);
		result.identify_axis(1);
		result.identify_axis(2);
	}

	//----------------------------------------------------------------
	//identtiy
	//----------------------------------------------------------------
	friend void matrix_zero(SELF_TYPE& result)
	{
		result.data[0] = VEC_TYPE::get_zero();
		result.data[1] = VEC_TYPE::get_zero();
		result.data[2] = VEC_TYPE::get_zero();
	}

	//----------------------------------------------------------------
	//2d translation
	//----------------------------------------------------------------
	friend void matrix_translation_2d(SELF_TYPE& result, const DATA_TYPE& tx, const DATA_TYPE& ty)
	{
		result.data[0] = VEC_TYPE::get_axis(0);
		result.data[1] = VEC_TYPE::get_axis(1);
		result.data[2] = VEC_TYPE(tx, ty, (DATA_TYPE)1.0);
	}

	//----------------------------------------------------------------
	//scaling
	//----------------------------------------------------------------
	friend void matrix_scaling(SELF_TYPE& result, const DATA_TYPE& sx, const DATA_TYPE& sy, const DATA_TYPE& sz)
	{
		result.data[0] = VEC_TYPE::get_axis(0, sx);
		result.data[1] = VEC_TYPE::get_axis(1, sy);
		result.data[2] = VEC_TYPE::get_axis(2, sz);
	}

	//----------------------------------------------------------------
	//X rotation matrix
	//----------------------------------------------------------------
	friend void  matrix_rotation_x(SELF_TYPE& result, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(s, c, angle);

		SELF_TYPE result;
		result.data[0] = VEC_TYPE::get_axis(0);
		result.data[1] = VEC_TYPE(0,  c, s);
		result.data[2] = VEC_TYPE(0, -s, c);
	}

	//----------------------------------------------------------------
	//Y rotation matrix
	//----------------------------------------------------------------
	friend void matrix_rotation_y(SELF_TYPE& result, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(s, c, angle);

		result.data[0] = VEC_TYPE(c, 0, -s);
		result.data[1] = VEC_TYPE::get_axis(1);
		result.data[2] = VEC_TYPE(s, 0,  c);
	}

	//----------------------------------------------------------------
	//Z rotation matrix
	//----------------------------------------------------------------
	friend void matrix_rotation_z(SELF_TYPE& result, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(s, c, angle);

		result.data[0] = VEC_TYPE( c, s, 0);
		result.data[1] = VEC_TYPE(-s, c, 0);
		result.data[2] = VEC_TYPE::get_axis(2);
	}

	//----------------------------------------------------------------
	//Matrix form quaternion
	//----------------------------------------------------------------
	/*friend void matrix_form_quat(SELF_TYPE& result, const quat<DATA_TYPE>& q)
	{
		typedef quat<DATA_TYPE> q_type;

		q_type q0 = q + q;
		q_type q1 = cmul(q0, q);

		q_type v0(q1.y, q1.x, q1.x, 0);
		q_type v1(q1.z, q1.z, q1.y, 0);
		q_type r0 = q_type(1.0, 1.0, 1.0, 0.0) - v0;

		v0 = q_type(q.x, q.x, q.y, q.w);
		v1 = q_type(q0.z, q0.y, q0.z, q0.w);
		v0 = cmul(v0, v1);

		v1 = q_type(q.w, q.w, q.w, q.w);
		q_type v2 = q_type(q0.y, q0.z, q0.x, q0.w);
		v1 = cmul(v1, v2);

		q_type r1 = v0 + v1;
		q_type r2 = v0 - v1;

		v0 = q_type(r1.y, r2.x, r2.y, r1.z);
		v1 = q_type(r1.x, r2.z, r1.x, r2.z);

	}*/

};

SGE_END_MATH_NAMESPACE