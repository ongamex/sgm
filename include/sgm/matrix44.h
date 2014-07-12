#pragma once

#include "matrix33.h"

SGE_BEGIN_MATH_NAMESPACE

////////////////////////////////////////////////////////////////////////
//struct mat44 A column major storage matrix type
////////////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct matrix44
{
	typedef typename TDATA_TYPE DATA_TYPE;
	typedef matrix44 SELF_TYPE;

	static const unsigned int NUM_ROW = 4;
	static const unsigned int NUM_COL = 4;

	static const unsigned int VEC_SIZE = NUM_COL;
	static const unsigned int NUM_VECS = NUM_ROW;

	typedef vec4<DATA_TYPE> VEC_TYPE;

	VEC_TYPE data[NUM_VECS];

	#include "matrix_common.inl"

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

		const DATA_TYPE invDet = (DATA_TYPE)(1.0)/(s[0]*c[5]-s[1]*c[4]+s[2]*c[3]+s[3]*c[2]-s[4]*c[1]+s[5]*c[0]);

		SELF_TYPE result;

		result.data[0][0] = ( data[1][1] * c[5] - data[1][2] * c[4] + data[1][3] * c[3]) * invDet;
		result.data[0][1] = (-data[0][1] * c[5] + data[0][2] * c[4] - data[0][3] * c[3]) * invDet;
		result.data[0][2] = ( data[3][1] * s[5] - data[3][2] * s[4] + data[3][3] * s[3]) * invDet;
		result.data[0][3] = (-data[2][1] * s[5] + data[2][2] * s[4] - data[2][3] * s[3]) * invDet;

		result.data[1][0] = (-data[1][0] * c[5] + data[1][2] * c[2] - data[1][3] * c[1]) * invDet;
		result.data[1][1] = ( data[0][0] * c[5] - data[0][2] * c[2] + data[0][3] * c[1]) * invDet;
		result.data[1][2] = (-data[3][0] * s[5] + data[3][2] * s[2] - data[3][3] * s[1]) * invDet;
		result.data[1][3] = ( data[2][0] * s[5] - data[2][2] * s[2] + data[2][3] * s[1]) * invDet;

		result.data[2][0] = ( data[1][0] * c[4] - data[1][1] * c[2] + data[1][3] * c[0]) * invDet;
		result.data[2][1] = (-data[0][0] * c[4] + data[0][1] * c[2] - data[0][3] * c[0]) * invDet;
		result.data[2][2] = ( data[3][0] * s[4] - data[3][1] * s[2] + data[3][3] * s[0]) * invDet;
		result.data[2][3] = (-data[2][0] * s[4] + data[2][1] * s[2] - data[2][3] * s[0]) * invDet;

		result.data[3][0] = (-data[1][0] * c[3] + data[1][1] * c[1] - data[1][2] * c[0]) * invDet;
		result.data[3][1] = ( data[0][0] * c[3] - data[0][1] * c[1] + data[0][2] * c[0]) * invDet;
		result.data[3][2] = (-data[3][0] * s[3] + data[3][1] * s[1] - data[3][2] * s[0]) * invDet;
		result.data[3][3] = ( data[2][0] * s[3] - data[2][1] * s[1] + data[2][2] * s[0]) * invDet;

		return result;
	}

	friend SELF_TYPE inverse(const SELF_TYPE& m)
	{
		return m.inverse();
	}

	//----------------------------------------------------------------
	//3d translation
	//----------------------------------------------------------------
	friend void matrix_translation_3d(SELF_TYPE& result, const DATA_TYPE& tx, const DATA_TYPE& ty, const DATA_TYPE& tz)
	{
		result.data[0] = VEC_TYPE::get_axis(0);
		result.data[1] = VEC_TYPE::get_axis(1);
		result.data[2] = VEC_TYPE::get_axis(2);
		result.data[3] = VEC_TYPE(tx, ty, tz, (DATA_TYPE)1.0);
	}

	friend void matrix_translation_3d(SELF_TYPE& result, const vec3<DATA_TYPE>& t)
	{
		result.data[0] = VEC_TYPE::get_axis(0);
		result.data[1] = VEC_TYPE::get_axis(1);
		result.data[2] = VEC_TYPE::get_axis(2);
		result.data[3] = VEC_TYPE(t.x, t.y, t.z, (DATA_TYPE)1.0);
	}

	//----------------------------------------------------------------
	//scaling
	//----------------------------------------------------------------
	friend void matrix_scaling(SELF_TYPE& result, const DATA_TYPE& sx, const DATA_TYPE& sy, const DATA_TYPE& sz)
	{
		result.data[0] = VEC_TYPE::get_axis(0, sx);
		result.data[1] = VEC_TYPE::get_axis(1, sy);
		result.data[2] = VEC_TYPE::get_axis(2, sz);
		result.data[2] = VEC_TYPE::get_axis(3);
	}

	friend void matrix_scaling(SELF_TYPE& result, const vec3<DATA_TYPE>& s)
	{
		result.data[0] = VEC_TYPE::get_axis(0, s.x);
		result.data[1] = VEC_TYPE::get_axis(1, s.y);
		result.data[2] = VEC_TYPE::get_axis(2, s.z);
		result.data[2] = VEC_TYPE::get_axis(3);
	}

	//----------------------------------------------------------------
	//X rotation matrix
	//----------------------------------------------------------------
	friend void  matrix_rotation_x(SELF_TYPE& result, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(angle, s, c);

		result.data[0] = VEC_TYPE::get_axis(0);
		result.data[1] = VEC_TYPE(0,  c, s, 0);
		result.data[2] = VEC_TYPE(0, -s, c, 0);
		result.data[3] = VEC_TYPE::get_axis(3);
	}

	//----------------------------------------------------------------
	//Y rotation matrix
	//----------------------------------------------------------------
	friend void matrix_rotation_y(SELF_TYPE& result, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(angle, s, c);

		result.data[0] = VEC_TYPE(c, 0, -s, 0);
		result.data[1] = VEC_TYPE::get_axis(1);
		result.data[2] = VEC_TYPE(s, 0,  c, 0);
		result.data[3] = VEC_TYPE::get_axis(3);
	}

	//----------------------------------------------------------------
	//Z rotation matrix
	//----------------------------------------------------------------
	friend void matrix_rotation_z(SELF_TYPE& result, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(s, c, angle);

		result.data[0] = VEC_TYPE( c, s, 0, 0);
		result.data[1] = VEC_TYPE(-s, c, 0, 0);
		result.data[2] = VEC_TYPE::get_axis(2);
		result.data[3] = VEC_TYPE::get_axis(3);
	}

	//----------------------------------------------------------------
	//Quaternion rotation
	//----------------------------------------------------------------
	friend void matrix_rotation_quat(SELF_TYPE& result, const quat<DATA_TYPE>& quat)
	{
		const DATA_TYPE& x = q[0];
		const DATA_TYPE& y = q[1];
		const DATA_TYPE& z = q[2];
		const DATA_TYPE& w = q[3];

		const DATA_TYPE one = (DATA_TYPE)1.0;
		const DATA_TYPE two = (DATA_TYPE)2.0;

		result.data[0] = VEC_TYPE(	one - two*(y*y - z*z), 
									two*(x*y + z*w),
									two*(x*z - y*w),
									0);

		result.data[1] = VEC_TYPE(	two*(x*y - z*w),
									one - two*(x*x - z*z),
									two*(y*z + x*w),
									0);

		result.data[2] = VEC_TYPE(	two*(x*z + y*w),
									two*(y*z - x*w),
									one - two*(x*x - y*y),
									0);

		result.data[3] = VEC_TYPE::get_axis(3);
	}

	//----------------------------------------------------------------
	//perspective projection d3d right-handed
	//----------------------------------------------------------------
	friend void matrix_perspective_fov_rh(
		SELF_TYPE& result,
		const DATA_TYPE& fovY,
		const DATA_TYPE& widthByHeight,
		const DATA_TYPE& nZ,
		const DATA_TYPE& fZ)
	{
		const float 
		const float yScale = sgm::tanf(pi<float>()- (fovY * (DATA_TYPE)(0.5)));
		const float xScale = yScale / widthByHeight;
		const float fRange = fZ / (nZ - fZ);

		result.data[0] = VEC_TYPE(xScale, 0, 0, 0);
		result.data[1] = VEC_TYPE(0, yScale, 0, 0);
		result.data[2] = VEC_TYPE(0, 0, fRange, (DATA_TYPE)-1.0);
		result.data[3] = VEC_TYPE(0, 0, nZ * fRange, 0);
	}

	//----------------------------------------------------------------
	//look at view d3d right-handed
	//----------------------------------------------------------------
	friend void matrix_lookat_rh(
		SELF_TYPE& result,
		vec3<DATA_TYPE>& eye,
		vec3<DATA_TYPE>& lookDir,
		vec3<DATA_TYPE>& up)
	{
		const vec3<DATA_TYPE> zaxis = lookDir;
		const vec3<DATA_TYPE> xaxis = cross(up, zaxis);
		const vec3<DATA_TYPE> yaxis = cross(zaxis, xaxis);

		const vec3<DATA_TYPE> transl(-dot(eye, xaxis), -dot(eye, yaxis), -dot(eye, zaxis));

		result.data[0] = VEC_TYPE(xaxis[0], yaxis[0], zaxis[0], 0);
		result.data[1] = VEC_TYPE(xaxis[1], yaxis[1], zaxis[1], 0);
		result.data[2] = VEC_TYPE(xaxis[2], yaxis[2], zaxis[2], 0);
		result.data[3] = VEC_TYPE(transl, (DATA_TYPE)(1.0));
	}
};

SGE_END_MATH_NAMESPACE